#!/usr/bin/python3
# coding=utf8

# pyright: reportImplicitOverride=false

import random
import argparse

import hiwonder_common.camera_binary_program as camera_binary_program
from hiwonder_common.color_change import ColorChange


INITIAL_SPIRAL_TURN_RATE = 0.7


class BlueSortingProgram(camera_binary_program.CameraBinaryProgram):

    # Framework default. The species markers are ping pong balls, and the 1.06 m
    # detection range in TurboPi_Sensing_Characterization.xlsx was measured on a
    # ball, so it transfers directly. Apparent area falls off as 1/d^2, so
    # threshold = 300 * (1.06 / range)^2. 1350 put the trigger near 0.5 m, which
    # in the 4.3 x 3.7 m arena left the robots effectively blind to each other:
    # all six drove straight into the walls without ever turning.
    DETECT_MIN_AREA = 300
    # Framework default. A 3-frame window was chosen to release quickly while
    # turning away from a crowd. At this arena size and fleet count sightings are
    # sparse and brief, so the risk is missing one, not latching on a stale one;
    # the 10-frame window's noise rejection is worth the extra release lag.
    SMOOTHING_WINDOW = 10

    def __init__(self, args):
        super().__init__(args)
        self.target_colors = ['blue', 'green']
        self.detect_min_area = self.DETECT_MIN_AREA
        self.set_smoothing_window(self.SMOOTHING_WINDOW)
        # Spiral search state, ported from non_chasing_controller.spiral1. The
        # source rolls turn_orientation with randint(-1, 1), which can return 0
        # and pin the spiral to a straight line; choice((-1, 1)) keeps it turning.
        self.turn_orientation = random.choice((-1, 1))
        self.spiral_turn_rate = INITIAL_SPIRAL_TURN_RATE
        self.color = ColorChange()
        self.color.change_color('blue')

    def spiral_search(self):
        # Widening arc: the turn rate decays until it resets, so the robot covers
        # ground instead of closing a fixed circle, but still curves off walls.
        self.move(60, 90, self.spiral_turn_rate * self.turn_orientation)
        self.spiral_turn_rate -= 0.001
        if self.spiral_turn_rate < 0.35:
            self.spiral_turn_rate = INITIAL_SPIRAL_TURN_RATE

    def control(self):
        if self.smoothed_detected['green']:
            self.move(60, 90, 1.0)
        elif self.smoothed_detected['blue']:
            self.move(60, 90, -0.5)
        else:
            self.spiral_search()


def get_parser(parser, subparsers=None):
    return camera_binary_program.get_parser(parser, subparsers)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    get_parser(parser)
    args = parser.parse_args()

    program = BlueSortingProgram(args)
    camera_binary_program.main(args, program)
