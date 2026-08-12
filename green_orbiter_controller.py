#!/usr/bin/python3
# coding=utf8

# pyright: reportImplicitOverride=false

import sys
import random
import argparse

sys.path.append('/home/pi/TurboPi/')
import HiwonderSDK.Sonar as Sonar

import hiwonder_common.statistics_tools as st
import hiwonder_common.camera_binary_program as camera_binary_program
from hiwonder_common.color_change import ColorChange


class GreenOrbiterProgram(camera_binary_program.CameraBinaryProgram):

    def __init__(self, args):
        super().__init__(args)
        self.target_colors = ['red']
        self.averagers['red'] = st.Average(5)  # faster reaction to gaining/losing red
        self.color = ColorChange()
        self.color.change_color('green')

        self.sonar = Sonar.Sonar()
        self.distance = float('nan')
        self.avoid_distance = 200  # mm
        self.avoid_timer = 0

    def read_distance(self):
        distance = self.sonar.getDistance()
        if round(distance) == 5000:
            distance = float('inf')
        elif distance > 5000:
            distance = float('nan')
        self.distance = distance

    def control(self):
        self.read_distance()

        if self.avoid_timer > 0:
            self.move(-40, 90, random.choice([-0.8, 0.8]))
            self.avoid_timer -= 1
            return

        if self.distance < self.avoid_distance:
            self.avoid_timer = 15
            self.move(-40, 90, random.choice([-0.8, 0.8]))
            return

        if self.smoothed_detected['red']:
            self.move(100, 90, -0.5)
        else:
            self.move(100, 90, 0.5)


def get_parser(parser, subparsers=None):
    return camera_binary_program.get_parser(parser, subparsers)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    get_parser(parser)
    args = parser.parse_args()

    program = GreenOrbiterProgram(args)
    camera_binary_program.main(args, program)