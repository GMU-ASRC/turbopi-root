#!/usr/bin/python3
# coding=utf8
# from contextlib import ExitStack

# pyright: reportImplicitOverride=false

import argparse
import numpy as np

import hiwonder_common.statistics_tools as st
from hiwonder_common.camera_binary_program import range_rgb
import hiwonder_common.camera_binary_program as camera_binary_program

import hiwonder_common.color_change as color

class MillingProgram(camera_binary_program.CameraBinaryProgram):

    def __init__(self, args):
        super().__init__(args)
        self.color = color.ColorChange()

    def control(self):
        # self.set_rgb('green' if bool(self.smoothed_detected) else 'red')
        if self.smoothed_detected['green']:  # smoothed_detected is a low-pass filtered detection
            self.color.change_color('green')
        
        elif self.smoothed_detected['red']:
            self.color.change_color('red')
        
        elif self.smoothed_detected['blue']:
            self.color.change_color('blue')
        
        else:
            self.color.change_color('off')


def get_parser(parser, subparsers=None):
    return camera_binary_program.get_parser(parser, subparsers)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    get_parser(parser)
    args = parser.parse_args()

    program = MillingProgram(args)
    camera_binary_program.main(args, program)
