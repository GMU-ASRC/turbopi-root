#!/usr/bin/python3
# coding=utf8

# pyright: reportImplicitOverride=false

import argparse

import hiwonder_common.camera_binary_program as camera_binary_program
from hiwonder_common.color_change import ColorChange


class BlueDispersalProgram(camera_binary_program.CameraBinaryProgram):

    # Tuned from TurboPi_Sensing_Characterization.xlsx: the fleet detects a ball
    # out to 1.06 m at the default 300 px^2. Apparent area falls off as 1/d^2, so
    # threshold = 300 * (1.06 / range)^2; 1350 puts the trigger near 0.5 m. This
    # stops greens across the arena from holding the robot in a permanent turn.
    DETECT_MIN_AREA = 1350
    # The default 10-frame filter needs 5 clear frames to release, by which point
    # a turning robot has swept past the gap it was looking for. 3 still rejects
    # single-frame dropouts but releases in 2.
    SMOOTHING_WINDOW = 3

    def __init__(self, args):
        super().__init__(args)
        self.target_colors = ['blue', 'green']
        self.detect_min_area = self.DETECT_MIN_AREA
        self.set_smoothing_window(self.SMOOTHING_WINDOW)
        self.color = ColorChange()
        self.color.change_color('blue')

    def control(self):
        if self.smoothed_detected['green']:
            self.move(100, 180, 0)
        else:
            self.move(0, 90, 0.75)


def get_parser(parser, subparsers=None):
    return camera_binary_program.get_parser(parser, subparsers)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    get_parser(parser)
    args = parser.parse_args()

    program = BlueDispersalProgram(args)
    camera_binary_program.main(args, program)
