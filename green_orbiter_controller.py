#!/usr/bin/python3
# coding=utf8

# pyright: reportImplicitOverride=false

import argparse

import hiwonder_common.camera_binary_program as camera_binary_program
from hiwonder_common.color_change import ColorChange


class GreenOrbiterProgram(camera_binary_program.CameraBinaryProgram):

    def __init__(self, args):
        super().__init__(args)
        self.target_colors = ['red', 'green']
        self.color = ColorChange()
        self.color.change_color('green')

    def control(self):
        if self.smoothed_detected['red'] and not self.smoothed_detected['green']:
            self.move(100, 90, -0.5) #sees red with no crowding so orbit normally
        elif self.smoothed_detected['red'] and self.smoothed_detected['green']:
            #half speed so it falls behind since it sees red and green so its too close to another orbiter
            self.move(50, 90, -0.5)
        else:
            self.move(30, 90, 1.5) #slow down and turn faster to reaquire red


def get_parser(parser, subparsers=None):
    return camera_binary_program.get_parser(parser, subparsers)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    get_parser(parser)
    args = parser.parse_args()

    program = GreenOrbiterProgram(args)
    camera_binary_program.main(args, program)