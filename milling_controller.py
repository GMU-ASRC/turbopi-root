#!/usr/bin/python3
# coding=utf8
# from contextlib import ExitStack

# pyright: reportImplicitOverride=false

import argparse
import numpy as np

import hiwonder_common.statistics_tools as st
from hiwonder_common.camera_binary_program import range_bgr
import hiwonder_common.camera_binary_program as camera_binary_program


class SandmanProgram(camera_binary_program.CameraBinaryProgram):
    name = "SandmanProgram"

    def __init__(self, args, post_init=True, board=None, name=None, disable_logging=False) -> None:
        super().__init__(args, post_init=False, board=board, name=name, disable_logging=disable_logging)
        self.boolean_detection_averager = st.Average(10)
        self.boolean_detection_averager2 = st.Average(10)

        self.random_walk = False
        self.random_walk_time = 0
        self.random_turn_time = 0
        self.turn_orientation = random.randint(-1, 1)

        self.control_modes = {
            "pause": [(0, 0, 0), (0, 0, 0)],
            "mill": [(100, 90, -0.5), (100, 90, 0.5)],
            "follow_leader": [(75, 90, 0),(75, 90, -0.5)],
            "disperse": [(100, 90, -2),(100, 90, 0)],
            "diffuse": [(50, 270, 0),(0, 270, 2)],
            "straight": [(50, 90, 0), (50, 90, 0)],
            "circle": [(50, 90, 0.5), (50, 90, 0.5)]
        }
        self.mode = "pause"

    def control(self):
        if mode in self.control_modes.keys():
            self.cur_mode = mode

        print(self.cur_mode)
        try:
            velocities = self.control_modes[self.cur_mode]
        except KeyError:
            velocities = self.control_modes['pause']
            print("Invalid mode, pausing.")
        detected_vel = velocities[0]
        undetected_vel = velocities[1]

        # self.set_rgb ('green' if bool(self.smoothed_detected) or bool(self.smoothed_detected2) else 'blue')
        if self.smoothed_detected2:
            self.set_rgb('red')
        elif self.smoothed_detected:
            self.set_rgb('green')
        else:
            self.set_rgb('blue')

        if not self.dry_run:
            if self.smoothed_detected2:  # smoothed_detected is a low-pass filtered detection
                self.chassis.set_velocity(100, 90, 0)  # Control robot movement function
                        # linear speed 50 (0~100), direction angle 90 (0~360), yaw angular speed 0 (-2~2)
            # elif self.smoothed_detected and self.smoothed_detected2:
            #     self.chassis.set_velocity(100, 90, 0)
            else:
                if self.random_walk:
                    if self.random_walk_time:
                        self.chassis.set_velocity(100, 90, 0)
                        self.random_walk_time -= 1
                    elif self.random_turn_time:
                        self.chassis.set_velocity(0, 90, self.turn_orientation * 2)
                        self.random_turn_time -= 1
                    else:
                        self.random_walk_time = random.randint(50, 250)
                        self.random_turn_time = random.randint(50, 250)
                        self.turn_orientation = random.randint(-1, 1)
                        while self.turn_orientation == 0:
                            self.turn_orientation = random.randint(-1, 1)
                else:    
                    if self.smoothed_detected:  # smoothed_detected is a low-pass filtered detection
                        self.chassis.set_velocity(*detected_vel)
                    else:
                        self.chassis.set_velocity(*undetected_vel)
    
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
            'open_kernel': np.ones((3, 3), np.uint8),
            'close_kernel': np.ones((3, 3), np.uint8),
        }
        # extract the LAB threshold
        threshold = (tuple(self.lab_data[self.target_color][key]) for key in ['min', 'max'])
        threshold2 = (tuple(self.lab_data[self.target_color2][key]) for key in ['min', 'max'])
        # breakpoint()
        # run contour detection
        target_contours = self.color_contour_detection(
            frame_clean,
            tuple(threshold),
            **contour_args
        )
        target_contours2 = self.color_contour_detection(
            frame_clean,
            tuple(threshold2),
            **contour_args
        )
        # The output of color_contour_detection() is sorted highest to lowest
        biggest_contour, biggest_contour_area = target_contours[0] if target_contours else (None, 0)
        biggest_contour2, biggest_contour_area2 = target_contours2[0] if target_contours2 else (None, 0)
        self.detected: bool = biggest_contour_area > 300
        self.detected2: bool = biggest_contour_area2 > 300  # did we detect something of interest?

        self.smoothed_detected = self.boolean_detection_averager(self.detected)  # feed the averager
        self.smoothed_detected2 = self.boolean_detection_averager2(self.detected2)
        # print(bool(smoothed_detected), smoothed_detected)

        self.control(self.cur_mode)  # ################################


        # draw annotations of detected contours
        if self.detected:
            self.draw_fitted_rect(annotated_image, biggest_contour, range_bgr[self.target_color])
            self.draw_text(annotated_image, range_bgr[self.target_color], self.target_color)
        else:
            self.draw_text(annotated_image, range_bgr['black'], 'None')
        if biggest_contour_area2 > 100:
            self.draw_fitted_rect(annotated_image, biggest_contour2, range_bgr[self.target_color2])
        self.draw_fps(annotated_image, range_bgr['black'], avg_fps)
        frame_resize = cv2.resize(annotated_image, (320, 240))
        if self.show:
            cv2.imshow('frame', frame_resize)
            key = cv2.waitKey(1)
            if key == 27:
                return
        else:
            time.sleep(1E-3)
 



def get_parser(parser, subparsers=None):
    return camera_binary_program.get_parser(parser, subparsers)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    get_parser(parser)
    args = parser.parse_args()

    program = SandmanProgram(args)
    camera_binary_program.main(args, program)
