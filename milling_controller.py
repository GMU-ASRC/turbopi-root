#!/usr/bin/python3
# coding=utf8
# from contextlib import ExitStack

# pyright: reportImplicitOverride=false

import time
import random
import socket
import argparse

import cv2
import numpy as np

import hiwonder_common.program
import hiwonder_common.statistics_tools as st
from hiwonder_common.camera_binary_program import range_bgr
import hiwonder_common.camera_binary_program as camera_binary_program

SPECIAL_FUNCTION = "__function__"
dict_names = camera_binary_program.dict_names
dict_names -= {"target_color", "boolean_detection_averager"}
dict_names |= {
    "frn_boolean_detection_averager",
    "foe_boolean_detection_averager",
    "frn_detect_color",
    "foe_detect_color",
    "random_walk_time",
    "random_turn_time",
    "turn_orientation",
}


class UDP_Listener(hiwonder_common.program.UDP_Listener):
    def act(self, data, addr):
        split = data.split(b"cmd:\n", 1)
        try:
            cmd = split[1]
        except IndexError:
            return

        if b"halt" in cmd or b"stop" in cmd:
            self._run = False
            self.app._run = False
            self.app._stop_soon = True
            return
        if b"unpause" in cmd or b"resume" in cmd:
            self.app.resume()
        elif b"pause" in cmd:
            self.app.pause()
        elif b"switch" in cmd:
            program.mode = cmd.strip().removeprefix(b"switch ").decode().strip()
        elif b"random" in cmd:
            program.random_walk = not program.random_walk


class SandmanProgram(camera_binary_program.CameraBinaryProgram):
    UDP_LISTENER_CLASS = UDP_Listener
    name = "SandmanProgram"
    dict_names = dict_names

    def __init__(self, args, post_init=True, board=None, name=None, disable_logging=False) -> None:
        super().__init__(args, post_init=False, board=board, name=name, disable_logging=disable_logging)

        self.frn_boolean_detection_averager = st.Average(10)
        self.foe_boolean_detection_averager = st.Average(10)

        self.frn_detect_color = "green"
        self.foe_detect_color = "red"

        self.random_walk_time = 0
        self.random_turn_time = 0
        self.turn_orientation = random.randint(-1, 1)

        self.control_modes = {
            'idle': [(0, 0, 0), (0, 0, 0)],
            'mill': [(100, 90, -0.5), (100, 90, 0.5)],
            'follow_leader': [(75, 90, 0), (75, 90, -0.5)],
            'disperse': [(100, 90, -2), (100, 90, 0)],
            'diffuse': [(50, 270, 0), (0, 270, 2)],
            'straight': [(50, 90, 0), (50, 90, 0)],
            'circle': [(50, 90, 0.5), (50, 90, 0.5)],
            'random': [(100, 90, 0), self.random_walk],
        }
        self._mode = None
        self.mode = 'idle' if args.mode is None else args.mode

        if post_init:
            self.startup_beep()

    @property
    def mode(self):
        return self._mode

    @mode.setter
    def mode(self, mode):
        if mode not in self.control_modes:
            print(f"invalid mode: {mode}")
        else:
            self._mode = mode

    def control_wrapper(self):
        self.control()
        if self.detection_log:
            self.history.append([
                time.time_ns(),
                self.frn_detected,
                self.smoothed_frn_detected,
                self.foe_detected,
                self.smoothed_foe_detected,
                self.moves_this_frame,
            ])
            self.log_detection()

    def log_detection(self):
        r = self.history[-1]
        self.detection_log += f"{r[0]}\t{int(r[1])}\t{int(bool(r[2]))}\t{repr(r[3])}\t{repr(r[4])}\t{repr(r[5])}\n"

    def log_detection_header(self):
        n = self.boolean_detection_averager.n
        self.detection_log += f"time_ns\tfriend_detected [0, 1]\tfriend_smoothed_detected [0, 1] ({n})\tfoe_detected [0, 1]\tfoe_smoothed_detected [0, 1] ({n})\tmoves [(v, d, w), ...]\n"

    def random_walk(self):
        if self.random_walk_time:
            self.move(100, 90, 0)
            self.random_walk_time -= 1
        elif self.random_turn_time:
            self.move(0, 90, self.turn_orientation * 2)
            self.random_turn_time -= 1
        else:
            self.random_walk_time = random.randint(50, 250)
            self.random_turn_time = random.randint(50, 250)
            self.turn_orientation = random.randint(-1, 1)
            while self.turn_orientation == 0:
                self.turn_orientation = random.randint(-1, 1)

    def control(self):
        if self.smoothed_foe_detected:
            self.set_rgb("red")
        elif self.smoothed_frn_detected:
            self.set_rgb("green")
        else:
            self.set_rgb("blue")

        def move_or_call(move_or_function):
            if callable(move_or_function):
                move_or_function()
                return
            if isinstance(move_or_function, (list, tuple)):
                self.move(*move_or_function)

        if self.mode not in self.control_modes:
            return  # ======================================
        actions = self.control_modes[self.mode]
        if isinstance(actions, (list, tuple)) and len(actions) == 2:
            # mode is a simple binary mode
            detected_action, nodetect_action = actions
            if self.smoothed_foe_detected:  # smoothed_detected is a low-pass filtered detection
                move_or_call(detected_action)
            else:
                move_or_call(nodetect_action)
        elif callable(actions):
            actions()  # this handles what to do regardless of if detected or not


    def main_loop(self):
        avg_fps = self.fps_averager(self.fps)  # feed the averager
        raw_img = self.camera.frame
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
            "open_kernel": np.ones((3, 3), np.uint8),
            "close_kernel": np.ones((3, 3), np.uint8),
        }
        # extract the LAB threshold
        frn_threshold = (tuple(self.lab_data[self.frn_detect_color][key]) for key in ["min", "max"])
        foe_threshold = (tuple(self.lab_data[self.foe_detect_color][key]) for key in ["min", "max"])

        # run contour detection
        frn_contours = self.color_contour_detection(frame_clean, tuple(frn_threshold), **contour_args)
        foe_contours = self.color_contour_detection(frame_clean, tuple(foe_threshold), **contour_args)
        # The output of color_contour_detection() is sorted highest to lowest
        frn_biggest_contour, frn_biggest_contour_area = frn_contours[0] if frn_contours else (None, 0)
        foe_biggest_contour, foe_biggest_contour_area = foe_contours[0] if foe_contours else (None, 0)
        self.frn_detected: bool = frn_biggest_contour_area > 300
        self.foe_detected: bool = foe_biggest_contour_area > 300  # did we detect something of interest?

        self.smoothed_frn_detected = self.frn_boolean_detection_averager(self.frn_detected)  # feed the averager
        self.smoothed_foe_detected = self.foe_boolean_detection_averager(self.foe_detected)
        # print(bool(smoothed_detected), smoothed_detected)

        self.control_wrapper()  # ################################

        # draw annotations of detected contours
        if self.foe_detected:
            self.draw_fitted_rect(annotated_image, foe_biggest_contour, range_bgr[self.frn_detect_color])
            self.draw_text(annotated_image, range_bgr[self.frn_detect_color], self.frn_detect_color)
        else:
            self.draw_text(annotated_image, range_bgr["black"], "None")
        if foe_biggest_contour_area > 100:
            self.draw_fitted_rect(annotated_image, foe_biggest_contour, range_bgr[self.foe_detect_color])
        self.draw_fps(annotated_image, range_bgr["black"], avg_fps)
        frame_resize = cv2.resize(annotated_image, (320, 240))
        if self.show:
            cv2.imshow("frame", frame_resize)
            key = cv2.waitKey(1)
            if key == 27:
                return
        else:
            time.sleep(1e-3)


def get_parser(parser, subparsers=None):
    parser, subparsers = camera_binary_program.get_parser(parser, subparsers)
    parser: argparse.ArgumentParser
    parser.add_argument('--mode', help="mode to start in")
    return parser, subparsers


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    get_parser(parser)
    args = parser.parse_args()

    program = SandmanProgram(args)
    camera_binary_program.main(args, program)
