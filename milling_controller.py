#!/usr/bin/python3
# coding=utf8
# from contextlib import ExitStack

# pyright: reportImplicitOverride=false

import sys
import time
import random
import socket
import argparse

import cv2
import numpy as np
from statemachine import StateMachine, State

sys.path.append('/home/pi/TurboPi/')
import HiwonderSDK.Sonar as Sonar

import hiwonder_common.program
import hiwonder_common.statistics_tools as st
from hiwonder_common.camera_binary_program import range_bgr
import hiwonder_common.camera_binary_program as camera_binary_program

SPECIAL_FUNCTION = "__function__"
dict_names = camera_binary_program.dict_names
dict_names -= {"target_color", "boolean_detection_averager"}
dict_names |= {
    "frn_boolean_detection_averager",
    "foe_boolean_detection_averager",
    "frn_detect_color",
    "foe_detect_color",
    "random_walk_time",
    "random_turn_time",
    "turn_orientation",
}


INITIAL_SPIRAL_TURN_RATE = 0.7


class TrackingState(StateMachine):
    search = State(initial=True)
    stuck = State()
    chase = State()
    reacquire = State()

    def __init__(self, robot, stuck_distance=100, unstuck_time=1.5, foe_lost_time=5,):
        self.robot: SandmanProgram = robot

        self.stuck_distance = stuck_distance
        self.unstuck_time = unstuck_time
        self.t_stuck = 0
        self.foe_lost_time = foe_lost_time
        self.t_foe_lost = 0

        super().__init__()

    cycle = (
        stuck.from_(search, cond='is_stuck')
        | chase.from_(search, stuck, cond='see')
        | reacquire.to(search, cond='lost')
        | stuck.to(search, cond='stuck_elapsed')
        | chase.to(reacquire, unless='see')
        | chase.to.itself(internal=True)
        | reacquire.to.itself(internal=True)
        | stuck.to.itself(internal=True)
        | search.to.itself(internal=True)
    )

    def is_stuck(self, distance, see_foe):
        return distance < self.stuck_distance  # NB: comparing int to 'nan' is always false

    def stuck_elapsed(self, distance, see_foe):
        return (time.time() - self.t_stuck) > self.unstuck_time

    def see(self, distance, see_foe):
        return see_foe

    def lost(self, distance, see_foe):
        return (time.time() - self.t_foe_lost) > self.foe_lost_time

    def on_cycle(self):
        s = self.current_state
        if s == self.search:
            self.robot.search()
        if s == self.chase:
            self.robot.chase()
        if s == self.reacquire:
            self.robot.move_towards_foe_lastseen()
        if s == self.stuck:
            self.robot.turn()

    def on_enter_stuck(self):
        self.t_stuck = time.time()

    def on_enter_reacquire(self):
        self.t_foe_lost = time.time()


class UDP_Listener(hiwonder_common.program.UDP_Listener):
    def act(self, data, addr):
        split = data.split(b"cmd:\n", 1)
        try:
            cmd = split[1]
        except IndexError:
            return

        if b"halt" in cmd or b"stop" in cmd:
            self._run = False
            self.app._run = False
            self.app._stop_soon = True
            return
        if b"unpause" in cmd or b"resume" in cmd:
            self.app.resume()
        elif b"pause" in cmd:
            self.app.pause()
        elif b"switch" in cmd:
            program.mode = cmd.strip().removeprefix(b"switch ").decode().strip()
        elif b"random" in cmd:
            program.random_walk = not program.random_walk


class SandmanProgram(camera_binary_program.CameraBinaryProgram):
    UDP_LISTENER_CLASS = UDP_Listener
    name = "SandmanProgram"
    dict_names = dict_names

    def __init__(self, args, post_init=True, board=None, name=None, disable_logging=False) -> None:
        super().__init__(args, post_init=False, board=board, name=name, disable_logging=disable_logging)

        self.sonar = Sonar.Sonar()
        self.distance = float('nan')
        self.stuck = TrackingState(self)
        self.tracking_pid = st.PID(1.0, 0.01, 0.0)

        self.frn_boolean_detection_averager = st.Average(10)
        self.foe_boolean_detection_averager = st.Average(10)

        self.frn_detect_color = "blue"
        self.foe_detect_color = "green"

        self.random_walk_time = 0
        self.random_turn_time = 0
        self.turn_orientation = random.randint(-1, 1)
        self.spiral_turn_rate = INITIAL_SPIRAL_TURN_RATE

        self.foe_position = None
        self.t_foe_last_detected = None

        self.search_modes = {
            'idle': [(0, 0, 0), (0, 0, 0)],
            'mill': [(100, 90, -0.5), (100, 90, 0.5)],
            'follow_leader': [(75, 90, 0), (75, 90, -0.5)],
            'disperse': [(100, 90, -2), (100, 90, 0)],
            'diffuse': [(50, 270, 0), (0, 270, 2)],
            'straight': [(50, 90, 0), (50, 90, 0)],
            'circle': [(50, 90, 0.5), (50, 90, 0.5)],
            'spin': [(0, 90, 1.5), (0, 90, 1.5)],
            'spiral': self.spiral1,
            'random': self.random_walk
        }
        self._mode = None
        self.mode = args.mode if args.mode in self.search_modes else 'idle'

        if post_init:
            self.startup_beep()

    @property
    def mode(self):
        return self._mode

    @mode.setter
    def mode(self, mode):
        if mode not in self.search_modes:
            print(f"invalid mode: {mode}")
        else:
            self._mode = mode

    def control_wrapper(self):
        self.control()
        if self.detection_log:
            self.history.append([
                time.time_ns(),
                self.frn_detected,
                self.smoothed_frn_detected,
                self.foe_detected,
                self.smoothed_foe_detected,
                self.moves_this_frame,
            ])
            self.log_detection()

    def log_detection(self):
        r = self.history[-1]
        self.detection_log += f"{r[0]}\t{int(r[1])}\t{int(bool(r[2]))}\t{repr(r[3])}\t{repr(r[4])}\t{repr(r[5])}\n"

    def log_detection_header(self):
        n = self.boolean_detection_averager.n
        self.detection_log += f"time_ns\tfriend_detected [0, 1]\tfriend_smoothed_detected [0, 1] ({n})\tfoe_detected [0, 1]\tfoe_smoothed_detected [0, 1] ({n})\tmoves [(v, d, w), ...]\n"

    def random_walk(self):
        if self.random_walk_time:
            self.move(100, 90, 0)
            self.random_walk_time -= 1
        elif self.random_turn_time:
            self.move(0, 90, self.turn_orientation * 2)
            self.random_turn_time -= 1
        else:
            self.random_walk_time = random.randint(50, 250)
            self.random_turn_time = random.randint(50, 250)
            self.turn_orientation = random.randint(-1, 1)
            while self.turn_orientation == 0:
                self.turn_orientation = random.randint(-1, 1)

    def spiral1(self):
        self.move(100, 90, self.spiral_turn_rate * self.turn_orientation)
        self.spiral_turn_rate -= 0.001
        if self.spiral_turn_rate < 0.35:
            self.spiral_turn_rate = INITIAL_SPIRAL_TURN_RATE

    def move_towards_foe_lastseen(self):
        if self.foe_position is None:
            # self.random_walk()
            self.move(50, 90, 0)
        elif -0.3 < self.foe_position < 0.3:  # straight
            self.move(50, 90, 0)
        elif self.foe_position < -0.3:  # left
            self.move(50, 90, -0.5)
        elif self.foe_position > 0.3:  # right
            self.move(50, 90, 0.5)

    def chase(self):
        p = self.foe_position
        e = self.tracking_pid(p)
        # print(f"pos: {p:8.2}\t error: {e:8.2}")
        self.move(50, 90, e)  # p controller

    def turn(self):
        self.move(0, 90, 0.5)

    def search(self):

        def move_or_call(move_or_function):
            if callable(move_or_function):
                move_or_function()
                return
            if isinstance(move_or_function, (list, tuple)):
                self.move(*move_or_function)

        if self.mode not in self.search_modes:
            return  # ======================================
        actions = self.search_modes[self.mode]
        if isinstance(actions, (list, tuple)) and len(actions) == 2:
            # mode is a simple binary mode
            index = int(self.smoothed_frn_detected)# smoothed_detected is a low-pass filtered detection
            move_or_call(actions[index])
        elif callable(actions):
            actions()  # this handles what to do regardless of if detected or not

    def control(self):
        if self.smoothed_foe_detected:
            self.set_rgb("yellow")
        elif self.smoothed_frn_detected:
            self.set_rgb("green")
        else:
            self.set_rgb("blue")

        self.stuck.cycle(self.distance, self.smoothed_foe_detected)
        # print(self.stuck.current_state)
        # print(f"{'█' if self.stuck. else ' '}  {(self.distance / 1000):5.3f}")

    def main_loop(self):
        avg_fps = self.fps_averager(self.fps)  # feed the averager
        raw_img = self.camera.frame
        if raw_img is None:
            time.sleep(0.01)
            return

        # prep a resized, blurred version of the frame for contour detection
        frame = raw_img.copy()
        frame_clean = cv2.resize(frame, self.preview_size, interpolation=cv2.INTER_NEAREST)
        frame_clean = cv2.GaussianBlur(frame_clean, (3, 3), 3)
        frame_clean = cv2.cvtColor(frame_clean, cv2.COLOR_BGR2LAB)  # convert to LAB space

        # prep a copy to be annotated
        annotated_image = raw_img.copy()

        # If we're calling target_contours() multiple times, some args will
        # be the same. Let's put them here to re-use them.
        contour_args = {
            "open_kernel": np.ones((3, 3), np.uint8),
            "close_kernel": np.ones((3, 3), np.uint8),
        }
        # extract the LAB threshold
        frn_threshold = (tuple(self.lab_data[self.frn_detect_color][key]) for key in ["min", "max"])
        foe_threshold = (tuple(self.lab_data[self.foe_detect_color][key]) for key in ["min", "max"])

        # run contour detection
        frn_contours = self.color_contour_detection(frame_clean, tuple(frn_threshold), **contour_args)
        foe_contours = self.color_contour_detection(frame_clean, tuple(foe_threshold), **contour_args)
        # The output of color_contour_detection() is sorted highest to lowest
        frn_biggest_contour, frn_biggest_contour_area = frn_contours[0] if frn_contours else (None, 0)
        foe_biggest_contour, foe_biggest_contour_area = foe_contours[0] if foe_contours else (None, 0)
        self.frn_detected: bool = frn_biggest_contour_area > 300
        self.foe_detected: bool = foe_biggest_contour_area > 300  # did we detect something of interest?
        frame_size = frame_clean.shape[1]
        if self.foe_detected:
            M = cv2.moments(foe_biggest_contour)
            if M['m00'] != 0:
                cx = int(M['m10'] / M['m00'])
            self.foe_position = (cx / frame_size) - 0.5

        self.smoothed_frn_detected = self.frn_boolean_detection_averager(self.frn_detected)  # feed the averager
        self.smoothed_foe_detected = self.foe_boolean_detection_averager(self.foe_detected)
        # print(bool(smoothed_detected), smoothed_detected)

        distance = self.sonar.getDistance()
        if round(distance) == 5000:
            distance = float('inf')
        elif distance > 5000:
            distance = float('nan')
        self.distance = distance

        self.control_wrapper()  # ################################

        # draw annotations of detected contours
        if self.foe_detected:
            self.draw_fitted_rect(annotated_image, foe_biggest_contour, range_bgr[self.frn_detect_color])
            self.draw_text(annotated_image, range_bgr[self.frn_detect_color], self.frn_detect_color)
        else:
            self.draw_text(annotated_image, range_bgr["black"], "None")
        if foe_biggest_contour_area > 100:
            self.draw_fitted_rect(annotated_image, foe_biggest_contour, range_bgr[self.foe_detect_color])
        self.draw_fps(annotated_image, range_bgr["black"], avg_fps)
        frame_resize = cv2.resize(annotated_image, (320, 240))
        if self.show:
            cv2.imshow("frame", frame_resize)
            key = cv2.waitKey(1)
            if key == 27:
                return
        else:
            time.sleep(1e-3)


def get_parser(parser, subparsers=None):
    parser, subparsers = camera_binary_program.get_parser(parser, subparsers)
    parser: argparse.ArgumentParser
    parser.add_argument('--mode', help="mode to start in", default='idle')
    return parser, subparsers


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    get_parser(parser)
    args = parser.parse_args()

    program = SandmanProgram(args)
    camera_binary_program.main(args, program)
