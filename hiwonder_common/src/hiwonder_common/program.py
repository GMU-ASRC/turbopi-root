#!/usr/bin/python3
# coding=utf8
# from contextlib import ExitStack
import sys

import RPi.GPIO as GPIO
sys.path.append('/home/pi/TurboPi/')
sys.path.append('/home/pi/boot/')
import os
import time
import math
import signal
import yaml
import numpy as np
import operator
import argparse
import socket
import threading
import platform

# import yaml_handle
import HiwonderSDK.Board as Board
import HiwonderSDK.mecanum as mecanum

import hiwonder_common.statistics_tools as st
import hiwonder_common.project as project
import hiwonder_common.env_tools as envt

# typing
from typing import Any

import warnings
try:
    import buttonman as buttonman
    buttonman.TaskManager.register_stoppable()
except ImportError:
    buttonman = None
    warnings.warn("buttonman was not imported, so no processes can be registered. This means the process can't be stopped by buttonman.",  # noqa: E501
                  ImportWarning, stacklevel=2)


KEY1_PIN = 33  # board numbering
KEY2_PIN = 16
KDN = GPIO.LOW
KUP = GPIO.HIGH
BUZZER_PIN = 31
# path = '/home/pi/TurboPi/'
SERVO_CFG_PATH = '/home/pi/TurboPi/servo_config.yaml'

UDP_PORT = 27272
MAGIC = b'pi__F00#VML'

listener_run = True


def udp_listener(program):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # create UDP socket
    s.settimeout(1)
    s.bind(('', UDP_PORT))

    def act(data, addr):
        split = data.split(b"cmd:\n", 1)
        try:
            cmd = split[1]
        except IndexError:
            return

        if b'halt' in cmd or b'stop' in cmd:
            global listener_run
            listener_run = False
            program._run = False
            program._stop_soon = True
            return True
        if b'unpause' in cmd or b'resume' in cmd:
            program.resume()
        elif b'pause' in cmd:
            program.pause()

    while listener_run:
        try:
            data, addr = s.recvfrom(1024)  # wait for a packet
        except socket.timeout:
            continue
        if data.startswith(MAGIC):
            stop = act(data[len(MAGIC):], addr)
            if stop:
                return


range_bgr = {
    'red': (0, 0, 255),
    'green': (0, 255, 0),
    'blue': (255, 0, 0),
    'black': (0, 0, 0),
    'white': (255, 255, 255),
}


class Program:
    name = "Program"
    dict_names = {'servo_cfg_path', 'servo_data', 'servo1', 'servo2', 'detection_log', 'dry_run', 'start_time'}

    def __init__(self, args, post_init=True) -> None:
        self._run = not args.pause
        self._stop_soon = False
        self.listener = None

        self.chassis = mecanum.MecanumChassis()

        self.servo_cfg_path = getattr(args, 'servo_cfg_path', SERVO_CFG_PATH)

        self.servo_data: dict[str, Any]
        self.load_servo_config(self.servo_cfg_path)

        if args.noproj:
            self.p = self.detection_log = None
        else:
            self.p = project.make_default_project(args.project, args.root)
            self.p.make_root_interactive()
            self.detection_log = project.Logger(self.p.root / f"io.tsv")

        self.board = Board if args.board is None else args.board

        self.servo1: int
        self.servo2: int

        self.dry_run = args.dry_run
        self.fps = 0.0
        self.fps_averager = st.Average(10)

        self.start_time = time.time_ns()
        self.moves_this_frame = []
        self.history = []  # movement history

        GPIO.setup(KEY1_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

        self.buttonman = buttonman
        if buttonman:
            self.key1_debouncer = buttonman.ButtonDebouncer(KEY1_PIN, self.btn1, bouncetime=50)
            self.key1_debouncer.start()
            GPIO.add_event_detect(KEY1_PIN, GPIO.BOTH, callback=self.key1_debouncer)

        if post_init:
            self.startup_beep()

        self.exit_on_stop = args.exit_on_stop

    def save_artifacts(self):
        self.p.save_yaml_artifact("runinfo.yaml", self)
        return True

    def as_config_dict(self):
        d = {key: getattr(self, key) for key in self.dict_names}
        return {
            "self": {
                'paused': self._run,
                **d
            },
            "env_info": self.get_env_info(),
        }

    def get_env_info(self):
        d = {
            "uname": platform.uname()._asdict(),
            "python_version": platform.python_version(),
            "cwd": os.getcwd(),
            ".dependencies": {},
        }
        try:
            d.update({
                "branch": envt.get_branch_name('.'),
                "HEAD": envt.git_hash('.'),
                "status": [s.strip() for s in envt.git_porcelain('.').split('\n')],
            })
        except Exception:
            d.update({"branch": None})
        return d

    def startup_beep(self):
        self.buzzfor(0.05)

    def btn1(self, channel, event):
        if event == KUP:
            if self.dry_run or not self._run:
                self.dry_run = False
                self._run = True
            else:
                self.dry_run = True
                self.kill_motors()

    @staticmethod
    def get_yaml_data(yaml_file):
        with open(yaml_file, 'r', encoding='utf-8') as file:
            file_data = file.read()

        return yaml.load(file_data, Loader=yaml.FullLoader)

    def init_move(self):
        servo_data = self.get_yaml_data(SERVO_CFG_PATH)
        self.servo1 = int(servo_data['servo1'])
        self.servo2 = int(servo_data['servo2'])
        Board.setPWMServoPulse(1, self.servo1, 1000)
        Board.setPWMServoPulse(2, self.servo2, 1000)

    def load_servo_config(self, servo_cfg_path):
        self.servo_data = self.get_yaml_data(servo_cfg_path)

    def kill_motors(self):
        self.chassis.set_velocity(0, 0, 0)

    def pause(self):
        self._run = False
        self.chassis.set_velocity(0, 0, 0)
        print(f"Program Paused w/ PID: {os.getpid()}")

    def resume(self):
        self._run = True
        print("Program Resumed")

    def stop(self):
        global listener_run
        self._run = False
        self.chassis.set_velocity(0, 0, 0)
        self.set_rgb('None')
        listener_run = False
        print("Program Stop")
        if buttonman:
            buttonman.TaskManager.unregister()
        if self.exit_on_stop:
            sys.exit()  # exit the python script immediately

    @staticmethod
    def buzzer(value):
        GPIO.output(BUZZER_PIN, bool(value))

    @classmethod
    def buzzfor(cls, dton, dtoff=0.0):
        cls.buzzer(1)
        time.sleep(dton)
        cls.buzzer(0)
        time.sleep(dtoff)

    def set_rgb(self, color):
        # Set the RGB light color of the expansion board to match the color you want to track
        if color not in range_bgr:
            color = "black"
        b, g, r = range_bgr[color]
        self.board.RGB.setPixelColor(0, self.board.PixelColor(r, g, b))
        self.board.RGB.setPixelColor(1, self.board.PixelColor(r, g, b))
        self.board.RGB.show()

    def move(self, v, a, w):
        # linear power [0, 100], direction angle [0, 360] (90 is forwards), yaw angular speed [-2,2]
        if self.dry_run:
            return
        # move and log
        self.moves_this_frame.append((v, a, w))
        self.chassis.set_velocity(v, a, w)

    def control(self):
        self.set_rgb('blue')
        self.move(100, 90, 0.5)

    def control_wrapper(self):
        self.control()
        if self.detection_log:
            self.history.append([time.time_ns(), self.moves_this_frame])
            self.log_detection()

    def log_detection(self):
        t, moves_this_frame = self.history[-1]
        self.detection_log += f"{t}\t{repr(moves_this_frame)}\n"

    def main_loop(self):
        self.moves_this_frame = []
        _avg_fps = self.fps_averager(self.fps)  # feed the averager

    def main(self):

        def sigint_handler(sig, frame):
            self.stop()

        def sigtstp_handler(sig, frame):
            self.pause()

        def sigcont_handler(sig, frame):
            self.resume()

        signal.signal(signal.SIGINT, sigint_handler)
        signal.signal(signal.SIGTERM, sigint_handler)
        signal.signal(signal.SIGTSTP, sigtstp_handler)
        signal.signal(signal.SIGCONT, sigcont_handler)

        self.init_move()

        self.listener = threading.Thread(target=udp_listener, args=(self,))
        self.listener.start()

        def loop():
            t_start = time.time_ns()
            self.main_loop()
            frame_ns = time.time_ns() - t_start
            frame_time = frame_ns / (10 ** 9)
            self.fps = 1 / frame_time
            # print(self.fps)

        if self.p:
            self.save_artifacts()

        errors = 0
        while errors < 5:
            if self._run:
                try:
                    loop()
                except KeyboardInterrupt:
                    print('Received KeyboardInterrupt')
                    self.exit_on_stop = True
                    self.stop()
                except BaseException as err:
                    errors += 1
                    suffix = ('th', 'st', 'nd', 'rd', 'th')
                    heck = "An" if errors == 1 else f"A {errors}{suffix[min(errors, 4)]}"
                    print(heck + " error occurred but we're going to ignore it and try again...")
                    print(err)
                    self.exit_on_stop = False
                    self.stop()
                    raise
            else:
                self.kill_motors()
                time.sleep(0.01)
            if self._stop_soon:
                self.stop()

        self.stop()


def get_parser(parser, subparsers=None):
    parser.add_argument("--dry_run", action='store_true')
    parser.add_argument("--startpaused", action='store_true')
    parser.add_argument("project", nargs='?', help="Path or name of project directory. Include a slash to specify a path.")
    parser.add_argument("--root", help="Path or name of project root directory.")
    parser.add_argument("--nolog", action='store_true')
    return parser, subparsers


def main(args, program):
    args.exit_on_stop = True
    program.main(args)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    get_parser(parser)
    args = parser.parse_args()
    program = Program(dry_run=args.dry_run, pause=args.startpaused, noproj=args.noproj)
    main(args, program)
