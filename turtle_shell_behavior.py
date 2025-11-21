#!/usr/bin/python3
# coding=utf8
# from contextlib import ExitStack

# pyright: reportImplicitOverride=false

import argparse
import numpy as np

import hiwonder_common.statistics_tools as st
from hiwonder_common.camera_binary_program import range_rgb
import hiwonder_common.camera_binary_program as camera_binary_program
import hiwonder_common.camera_binary_program_red as camera_binary_program_red


# class ShellingProgram(camera_binary_program.CameraBinaryProgram,camera_binary_program_red.CameraBinaryProgram_red):
class ShellingProgram(camera_binary_program_red.CameraBinaryProgram_red):

    def control(self):
        # self.set_rgb('green' if bool(self.smoothed_detected) else 'blue')
        # self.set_rgb('red' if bool(self.smoothed_detected_red) else 'blue')
        # if self.smoothed_detected_red:  # smoothed_detected is a low-pass filtered detection
        #     self.move(100, 90, -0.5)  # Control robot movement function
        # elif self.smoothed_detected:
        #     self.move(100, 90, -0.25)
        # else:
        #     self.move(100, 90, 0.25)

        self.set_rgb('red' if bool(self.smoothed_detected_red) else 'blue')
        if self.smoothed_detected_red:  # smoothed_detected is a low-pass filtered detection
            self.move(100, 90, 0)  # Control robot movement function
        else:
            self.move(100, 90, 0.5)


def get_parser(parser, subparsers=None):
    return (
        # camera_binary_program.get_parser(parser, subparsers),
        camera_binary_program_red.get_parser(parser, subparsers)
    )



if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    get_parser(parser)
    args = parser.parse_args()

    program = ShellingProgram(args)
    # camera_binary_program.main(args, program)
    camera_binary_program_red.main(args, program)
