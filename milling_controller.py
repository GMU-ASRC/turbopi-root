#!/usr/bin/python3
# coding=utf8
# from contextlib import ExitStack

# pyright: reportImplicitOverride=false

import argparse
import numpy as np
import cv2

import hiwonder_common.statistics_tools as st
from hiwonder_common.camera_binary_program import range_rgb
import hiwonder_common.camera_binary_program as camera_binary_program


class MillingProgram(camera_binary_program.CameraBinaryProgram):
    dict_names = camera_binary_program.dict_names | {'record'}

    def __init__(self, args, post_init=True, board=None, name=None, disable_logging=False) -> None:
        super().__init__(args, post_init=post_init, board=board, name=name, disable_logging=disable_logging)
        self.record = getattr(args, 'record', False)

    def main(self):
        if self.record:
            self.writer = cv2.VideoWriter('output.mp4', cv2.VideoWriter_fourcc(*'mp4v'), 30, (640, 480))
        super().main()

    def main_loop(self):
        super().main_loop()
        if self.record:
            raw_img = self.camera.frame
            if raw_img is not None:
                frame = cv2.resize(raw_img, self.preview_size)
                self.writer.write(frame)

    def stop(self, exit=True, silent=False):
        if self.record and getattr(self, 'writer', None):
            self.writer.release()
        super().stop(exit, silent)

    def control(self):
        self.set_rgb('green' if bool(self.smoothed_detected) else 'red')
        if self.smoothed_detected:  # smoothed_detected is a low-pass filtered detection
            self.move(100, 90, -0.5)  # Control robot movement function
        else:
            self.move(100, 90, 0.5)


def get_parser(parser, subparsers=None):
    parser.add_argument('--record', action='store_true', help="Record camera feed to output.mp4 (default: off)")
    return camera_binary_program.get_parser(parser, subparsers)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    get_parser(parser)
    args = parser.parse_args()

    program = MillingProgram(args)
    camera_binary_program.main(args, program)
