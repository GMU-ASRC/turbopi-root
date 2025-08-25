#!/usr/bin/python3
# coding=utf8

import time
import argparse
import cv2
import numpy as np
import apriltag

import hiwonder_common.camera_binary_program as camera_binary_program


class AprilTagDetector(camera_binary_program.CameraBinaryProgram):
    name = "AprilTagDetector"

    def __init__(self, args, post_init=True, board=None, name=None, disable_logging=False):
        super().__init__(args, post_init, board, name, disable_logging)
        self.apriltag_detector = apriltag.Detector()

    def control(self):
        self.set_rgb('green' if self.detected else 'red')

    def main_loop(self):
        self.moves_this_frame = []
        avg_fps = self.fps_averager(self.fps)
        raw_img = self.camera.frame
        if raw_img is None:
            time.sleep(0.01)
            return

        gray = cv2.cvtColor(raw_img, cv2.COLOR_BGR2GRAY)
        tags = self.apriltag_detector.detect(gray)

        self.detected = len(tags) > 0
        self.smoothed_detected = self.boolean_detection_averager(self.detected)

        self.control_wrapper()

        annotated_image = raw_img.copy()
        for tag in tags:
            corners = np.int32(tag.corners)
            cv2.polylines(annotated_image, [corners], isClosed=True, color=(0, 255, 0), thickness=2)
            center = tuple(np.mean(corners, axis=0, dtype=int))
            cv2.putText(annotated_image, f"ID: {tag.tag_id}", center, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        frame_resize = cv2.resize(annotated_image, (320, 240))
        if self.show:
            cv2.imshow('AprilTag Detection', frame_resize)
            key = cv2.waitKey(1)
            if key == 27:
                return
        else:
            time.sleep(1E-3)


def get_parser(parser, subparsers=None):
    return camera_binary_program.get_parser(parser, subparsers)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    get_parser(parser)
    args = parser.parse_args()

    program = AprilTagDetector(args)
    camera_binary_program.main(args, program)
