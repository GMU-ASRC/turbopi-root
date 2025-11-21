#!/usr/bin/python3
# coding=utf8
# from contextlib import ExitStack

# pyright: reportImplicitOverride=false

import time
import argparse
import numpy as np

from hiwonder_common.camera_binary_program import range_rgb
import hiwonder_common.program as program


class ConstantSpeedProgram(program.Program):
    name = "ConstantSpeedTest"
    def __init__(self, args, post_init=True, board=None, name=None, disable_logging=False) -> None:
        super().__init__(args, post_init=False, board=board, name=name, disable_logging=disable_logging)
        self.outputs = (args.forward_velocity, args.direction_vector, args.turning_rate)

        if post_init:
            self.startup_beep()

    def control(self):
        self.set_rgb('blue')

        def my_move(v, dt):
            self.move(v, 90, 0)
            self.detection_log += f"{time.time_ns()}\t{repr(self.moves_this_frame[-1])}\n"
            time.sleep(dt)

        run_time = 10
        for v in reversed(range(50, 100+1, 10)):
            my_move(v, run_time)
            my_move(0, 3)
            my_move(-v, run_time)
            my_move(0, 3)

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
