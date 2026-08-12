#!/usr/bin/python3
# coding=utf8

# pyright: reportImplicitOverride=false

import argparse
import random

import hiwonder_common.camera_binary_program as camera_binary_program
from hiwonder_common.color_change import ColorChange


class RedLeaderProgram(camera_binary_program.CameraBinaryProgram):

    def __init__(self, args):
        super().__init__(args)
        self.target_colors = ['red']
        self.color = ColorChange()
        self.color.change_color('red')

        self.sonar = Sonar.Sonar()
        self.distance = float('nan')
        self.avoid_distance = 200  # mm, react before getting too close
        self.avoid_timer = 0

        self.walk_timer = 0
        self.turn_timer = 0
        self.current_turn = 0.5

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
            self.move(-40, 90, self.current_turn)
            self.avoid_timer -= 1
            return

        if self.distance < self.avoid_distance:
            self.avoid_timer = 20
            self.current_turn = random.choice([-0.8, 0.8])
            self.move(-40, 90, self.current_turn)
            return

        if self.walk_timer > 0:
            self.move(100, 90, 0)
            self.walk_timer -= 1
        elif self.turn_timer > 0:
            self.move(60, 90, self.current_turn)
            self.turn_timer -= 1
        else:
            self.walk_timer = random.randint(30, 80)
            self.turn_timer = random.randint(20, 50)
            self.current_turn = random.choice([-0.8, -0.5, 0.5, 0.8])

    
def get_parser(parser, subparsers=None):
    return camera_binary_program.get_parser(parser, subparsers)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    get_parser(parser)
    args = parser.parse_args()

    program = RedLeaderProgram(args)
    camera_binary_program.main(args, program)