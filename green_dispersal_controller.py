#!/usr/bin/python3
# coding=utf8

# pyright: reportImplicitOverride=false

import argparse

import hiwonder_common.camera_binary_program as camera_binary_program
from hiwonder_common.color_change import ColorChange


class GreenDispersalProgram(camera_binary_program.CameraBinaryProgram):

    def __init__(self, args):
        super().__init__(args)
        self.target_colors = ['green']
        self.color = ColorChange()
        self.color.change_color('green')

    def control(self):
        if self.smoothed_detected['green']:
            self.move(100, 90, 2.0)
        else:
            self.move(100, 90, 0)


def get_parser(parser, subparsers=None):
    return camera_binary_program.get_parser(parser, subparsers)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    get_parser(parser)
    args = parser.parse_args()

    program = GreenDispersalProgram(args)
    camera_binary_program.main(args, program)
