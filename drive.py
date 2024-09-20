#!/usr/bin/python3
# coding=utf8
# from contextlib import ExitStack

# pyright: reportImplicitOverride=false

import time
import argparse
import numpy as np

from hiwonder_common.camera_binary_program import range_bgr
import hiwonder_common.program as program


class ConstantSpeedProgram(program.Program):
    name = "ConstantSpeedTest"
    def __init__(self, args, post_init=True, board=None, name=None, disable_logging=True) -> None:
        super().__init__(args, post_init=False, board=board, name=name, disable_logging=True)
        self.outputs = (args.forward_velocity, args.direction_vector, args.turning_rate)

        if post_init:
            self.startup_beep()

    def control(self):
        self.set_rgb('blue')
        self.move(*self.outputs)

    def main_loop(self):
        self.moves_this_frame = []
        _avg_fps = self.fps_averager(self.fps)  # feed the averager
        self.control_wrapper()
        time.sleep(1e-3)


def get_parser(parser: argparse.ArgumentParser, subparsers=None):
    parser.add_argument("-v", "--forward_velocity", type=float, default=0)
    parser.add_argument("-d", "--direction_vector", type=float, default=90)
    parser.add_argument("-w", "--turning_rate", type=float, default=0)
    parser.add_argument("--enable_logging", action="store_true")
    return parser, subparsers


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser, subparsers = program.get_parser(parser)
    parser, subparsers = get_parser(parser, subparsers)
    args = parser.parse_args()
    disable_logging = args.nolog or not args.enable_logging

    app = ConstantSpeedProgram(args, disable_logging=args.nolog)
    print("CTRL+C to stop and exit.")
    program.main(args, app)
