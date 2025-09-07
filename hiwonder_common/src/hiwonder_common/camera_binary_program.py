#!/usr/bin/python3
# coding=utf8
# from contextlib import ExitStack

# pyright: reportImplicitOverride=false

import sys
import time
import math
import operator
import argparse
import numpy as np
import cv2

import hiwonder_common.statistics_tools as st
from hiwonder_common.program import Program, main, range_rgb
import hiwonder_common.program  # modifies PATH

# import after path modification
import Camera  # type: ignore

# typing
from typing import Any

# path = '/home/pi/TurboPi/'
THRESHOLD_CFG_PATH = '/home/pi/TurboPi/lab_config.yaml'
SERVO_CFG_PATH = '/home/pi/TurboPi/servo_config.yaml'


dict_names = Program.dict_names
dict_names |= {'preview_size', 'target_color', 'lab_cfg_path', 'servo_cfg_path', 'lab_data', 'servo_data', 'detection_log', 'boolean_detection_averager'}  # noqa: E501


def rgb2bgr(rgb):
    return tuple(reversed(rgb[:3])) + rgb[3:]


class RangeBGR:
    @staticmethod
    def __getitem__(key):
        return rgb2bgr(range_rgb[key])


range_bgr = RangeBGR()


class CameraBinaryProgram(Program):
    dict_names = dict_names

    def __init__(self, args, post_init=True, board=None, name=None, disable_logging=False) -> None:
        super().__init__(args, post_init=False, board=board, name=name, disable_logging=disable_logging)
        self.preview_size = (640, 480)

        self.target_color = ('green')

        self.camera: Camera.Camera | None = None

        self.lab_cfg_path = getattr(args, 'lab_cfg_path', THRESHOLD_CFG_PATH)
        self.servo_cfg_path = getattr(args, 'servo_cfg_path', SERVO_CFG_PATH)

        self.lab_data: dict[str, Any]
        self.servo_data: dict[str, Any]
        self.load_lab_config(self.lab_cfg_path)
        self.load_servo_config(self.servo_cfg_path)

        self.servo1: int
        self.servo2: int
        self.detected = False
        self.boolean_detection_averager = st.Average(10)
        self.moves_this_frame = []
        self.history = []  # movement history

        self.show = self.can_show_windows()
        if not self.show:
            print("Failed to create test window.")
            print("I'll assuming you're running headless; I won't show image previews.")

        if post_init:
            self.startup_beep()

    @staticmethod
    def can_show_windows():
        img = np.zeros((100, 100, 3), np.uint8)
        try:
            cv2.imshow('headless_test', img)
            cv2.imshow('headless_test', img)
            _key = cv2.waitKey(1)
            cv2.destroyAllWindows()
        except Exception as err:
            if "Can't initialize GTK backend" in err.msg:  # type: ignore
                return False
            else:
                raise
        else:
            return True

    def load_lab_config(self, threshold_cfg_path):
        self.lab_data = self.get_yaml_data(threshold_cfg_path)

    def stop(self, exit=True, silent=False):
        if self.camera:
            self.camera.camera_close()
        self.set_rgb('None')
        cv2.destroyAllWindows()
        super().stop(False, True)
        if exit:
            Program.stop(self, True, silent)

    def control(self):
        self.set_rgb('green' if bool(self.smoothed_detected) else 'red')
        if self.smoothed_detected:  # smoothed_detected is a low-pass filtered detection
            self.move(100, 90, -0.5)  # Control robot movement function
        else:
            self.move(100, 90, 0.5)

    def control_wrapper(self):
        self.control()
        if self.detection_log:
            self.history.append([time.time_ns(), self.detected, self.smoothed_detected, self.moves_this_frame])
            self.log_detection()

    def log_detection(self):
        t, detected, smoothed_detected, moves_this_frame = self.history[-1]
        self.detection_log += f"{t}\t{int(detected)}\t{int(bool(smoothed_detected))}\t{repr(moves_this_frame)}\n"

    def log_detection_header(self):
        n = self.boolean_detection_averager.n
        self.detection_log += f"time_ns\tdetected [0, 1]\tsmoothed_detected [0, 1] ({n})\tmoves [(v, d, w), ...]\n"

    def main_loop(self):
        self.moves_this_frame = []
        avg_fps = self.fps_averager(self.fps)  # feed the averager
        raw_img = self.camera.frame  # This camera outputs BGR color
        if raw_img is None:
            time.sleep(0.01)
            return

        # prep a resized, blurred version of the frame for contour detection
        frame = raw_img.copy()
        frame_clean = cv2.resize(frame, self.preview_size, interpolation=cv2.INTER_NEAREST)
        frame_clean = cv2.GaussianBlur(frame_clean, (3, 3), 3)
        frame_clean = cv2.cvtColor(frame_clean, cv2.COLOR_BGR2LAB)  # convert to LAB space

        # prep a copy to be annotated
        annotated_image = raw_img.copy()

        # If we're calling target_contours() multiple times, some args will
        # be the same. Let's put them here to re-use them.
        contour_args = {
            'open_kernel': np.ones((3, 3), np.uint8),
            'close_kernel': np.ones((3, 3), np.uint8),
        }
        # extract the LAB threshold
        threshold = (tuple(self.lab_data[self.target_color][key]) for key in ['min', 'max'])
        # breakpoint()
        # run contour detection
        target_contours = self.color_contour_detection(
            frame_clean,
            tuple(threshold),  # type: ignore
            **contour_args
        )
        # The output of color_contour_detection() is sorted highest to lowest
        biggest_contour, biggest_contour_area = target_contours[0] if target_contours else (None, 0)
        self.detected: bool = biggest_contour_area > 300  # did we detect something of interest?

        self.smoothed_detected = self.boolean_detection_averager(self.detected)  # feed the averager

        self.control_wrapper()  # ################################

        # draw annotations of detected contours
        if self.detected:
            self.draw_fitted_rect(annotated_image, biggest_contour, range_bgr[self.target_color])
            self.draw_text(annotated_image, range_bgr[self.target_color], self.target_color)
        else:
            self.draw_text(annotated_image, range_bgr['black'], 'None')
        self.draw_fps(annotated_image, range_bgr['black'], avg_fps)
        frame_resize = cv2.resize(annotated_image, (320, 240))
        if self.show:
            cv2.imshow('frame', frame_resize)
            key = cv2.waitKey(1)
            if key == 27:
                return
        else:
            time.sleep(1E-3)

    def main(self):
        self.camera = Camera.Camera()
        self.camera.camera_open(correction=True)  # Enable distortion correction, not enabled by default
        super().main()

    @staticmethod
    def color_contour_detection(
        frame,
        threshold: tuple[tuple[int, int, int], tuple[int, int, int]],
        open_kernel: np.array = None,
        close_kernel: np.array = None,
    ):
        # Image Processing
        # mask the colors we want
        threshold = [tuple(li) for li in threshold]  # cast to tuple to make cv2 happy
        frame_mask = cv2.inRange(frame, *threshold)  # type: ignore
        # Perform an opening and closing operation on the mask
        # https://youtu.be/1owu136z1zI?feature=shared&t=34
        frame = frame_mask.copy()
        if open_kernel is not None:
            frame = cv2.morphologyEx(frame, cv2.MORPH_OPEN, open_kernel)
        if close_kernel is not None:
            frame = cv2.morphologyEx(frame, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))
        # find contours (blobs) in the mask
        contours = cv2.findContours(frame, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]
        areas = [math.fabs(cv2.contourArea(contour)) for contour in contours]
        # zip to provide pairs of (contour, area)
        zipped = zip(contours, areas)
        # return largest-to-smallest contour
        return sorted(zipped, key=operator.itemgetter(1), reverse=True)

    @staticmethod
    def draw_fitted_rect(img, contour, color):
        # draw rotated fitted rectangle around contour
        rect = cv2.minAreaRect(contour)
        box = np.int0(cv2.boxPoints(rect))  # type: ignore
        cv2.drawContours(img, [box], -1, color, 2)

    @staticmethod
    def draw_text(img, color, name):
        # Print the detected color on the screen
        cv2.putText(img, f"Color: {name}", (10, img.shape[0] - 10),
            cv2.FONT_HERSHEY_SIMPLEX, 0.65, color, 2)

    @staticmethod
    def draw_fps(img, color, fps):
        # Print the detected color on the screen
        cv2.putText(img, f"fps: {fps:.3}", (10, 20),
            cv2.FONT_HERSHEY_SIMPLEX, 0.65, color, 2)


def get_parser(parser, subparsers=None):
    return hiwonder_common.program.get_parser(parser, subparsers)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser, subparsers = get_parser(parser)
    args = parser.parse_args()
    program = Program(args, disable_logging=args.nolog)
    main(args, program)
