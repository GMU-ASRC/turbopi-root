#!/usr/bin/python3
# coding=utf8
# from contextlib import ExitStack

# pyright: reportImplicitOverride=false

import argparse
import numpy as np

import hiwonder_common.statistics_tools as st
from hiwonder_common.camera_binary_program import range_bgr
import hiwonder_common.camera_binary_program as camera_binary_program


class MillingProgram(camera_binary_program.CameraBinaryProgram):
    name = "MillingProgram"

    def control(self):
        self.set_rgb('green' if bool(self.smoothed_detected) else 'red')
        if self.smoothed_detected:  # smoothed_detected is a low-pass filtered detection
            self.move(100, 90, -0.5)  # Control robot movement function
        else:
            self.move(100, 90, 0.5)


def get_parser(parser, subparsers=None):
    return camera_binary_program.get_parser(parser, subparsers)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    get_parser(parser)
    args = parser.parse_args()

    program = MillingProgram(args)
    camera_binary_program.main(args, program)
