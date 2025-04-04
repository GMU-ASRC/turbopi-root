#!/usr/bin/python3
# coding=utf8

import sys
import time
import random
import socket
import argparse
import math #here

import cv2
import numpy as np
from statemachine import StateMachine, State

# This would be imported in the actual robot
# sys.path.append('/home/pi/TurboPi/')
# import HiwonderSDK.Sonar as Sonar

# Simulating imports for demonstration
class Sonar:
    def getDistance(self):
        return 500

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
    "rotate_distance",  
    "rotate_speed",     
}

INITIAL_SPIRAL_TURN_RATE = 0.7
DEFAULT_ROTATE_DISTANCE = 300  
DEFAULT_ROTATE_SPEED = 50      #for circling

class TrackingState(StateMachine):
    search = State(initial=True)
    stuck = State()
    rotate = State()  
    reacquire = State()

    def __init__(self, robot, stuck_distance=100, unstuck_time=1.5, foe_lost_time=2):
        self.robot = robot
        self.stuck_distance = stuck_distance
        self.unstuck_time = unstuck_time
        self.t_stuck = 0
        self.foe_lost_time = foe_lost_time
        self.t_foe_lost = 0
        super().__init__()

    cycle = (
        stuck.from_(search, cond='is_stuck')
        | rotate.from_(search, stuck, cond='see')  
        | reacquire.to(search, cond='lost')
        | stuck.to(search, cond='stuck_elapsed')
        | rotate.to(reacquire, unless='see')  
        | rotate.to.itself(internal=True)    
        | reacquire.to.itself(internal=True)
        | stuck.to.itself(internal=True)
        | search.to.itself(internal=True)
    )

    def is_stuck(self, distance, see_foe):
        return distance < self.stuck_distance

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
        if s == self.rotate:  
            self.robot.rotate()  
        if s == self.reacquire:
            self.robot.move_towards_foe_lastseen()
        if s == self.stuck:
            self.robot.turn()

    def on_enter_stuck(self):
        self.t_stuck = time.time()

    def on_enter_reacquire(self):
        self.t_foe_lost = time.time()


class UDP_Listener:
    def __init__(self, app):
        self.app = app
        self._run = True

    def start(self):
        pass

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
            self.app.mode = cmd.strip().removeprefix(b"switch ").decode().strip()


class RotateProgram(camera_binary_program.CameraBinaryProgram):
    UDP_LISTENER_CLASS = UDP_Listener
    name = "RotateProgram"
    dict_names = dict_names

    def __init__(self, args, post_init=True, board=None, name=None, disable_logging=False):
        super().__init__(args, post_init=False, board=board, name=name, disable_logging=disable_logging)

        self.sonar = Sonar()
        self.distance = float('nan')
        self.stuck = TrackingState(self)
        self.tracking_pid = st.PID(1.0, 0.01, 0.0)

        self.frn_boolean_detection_averager = st.Average(10)
        self.foe_boolean_detection_averager = st.Average(10)

        self.frn_detect_color = "green"
        self.foe_detect_color = "red"
        self.current_state_name = "starting"

        self.random_walk_time = 0
        self.random_turn_time = 0
        self.turn_orientation = random.randint(-1, 1)
        self.spiral_turn_rate = INITIAL_SPIRAL_TURN_RATE

        self.foe_position = None
        self.foe_area = 0 
        self.t_foe_last_detected = None
        
        self.rotate_distance = DEFAULT_ROTATE_DISTANCE 
        self.rotate_speed = DEFAULT_ROTATE_SPEED 
        self.rotate_direction = 1  
        self.rotate_angle = 0 

        self.search_modes = {
            'idle': [(0, 0, 0), (0, 0, 0)],
            'mill': [(100, 90, -0.5), (100, 90, 0.5)],
            'follow_leader': [(75, 90, 0), (75, 90, -0.5)],
            'disperse': [(100, 90, -2), (100, 90, 0)],
            'diffuse': [(0, 270, 2), (50, 270, 0)],
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
        if hasattr(self, 'detection_log') and self.detection_log:
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
        self.current_state_name = "Reacquire"
        if self.foe_position is None:
            self.move(50, 90, 0)
        elif -0.3 < self.foe_position < 0.3:
            self.move(50, 90, 0)
        elif self.foe_position < -0.3:
            self.move(50, 90, -0.5)
        elif self.foe_position > 0.3:
            self.move(50, 90, 0.5)

    def rotate(self):
        self.current_state_name = "Rotate" 
        
        if self.foe_position is None: 
            self.move(0, 90, 0.5) 
            return 
    
        distance_factor = 0 
        if self.foe_area > 0:
            if self.foe_area > self.rotate_distance: 
                distance_factor = -0.5 
            elif self.foe_area < self.rotate_distance * 0.7:
                distance_factor = 0.5 
    
        turn_rate = 0 
        
        if -0.2 < self.foe_position < 0.2: 
            turn_rate = 0.8 * self.rotate_direction 
        elif self.foe_position < -0.2: 
            turn_rate = -0.5 + (self.foe_position * 2) 
        elif self.foe_position > 0.2:
            turn_rate = 0.5 + (self.foe_position * 2) 
            
        forward_speed = self.rotate_speed + (distance_factor * 20) 
        self.move(forward_speed, 90, turn_rate) 
        

    def turn(self):
        self.current_state_name = "Stuck"
        self.move(0, 90, 0.5)

    def search(self):
        self.current_state_name = "Search"
        def move_or_call(move_or_function):
            if callable(move_or_function):
                move_or_function()
                return
            if isinstance(move_or_function, (list, tuple)):
                self.move(*move_or_function)

        if self.mode not in self.search_modes:
            return
        actions = self.search_modes[self.mode]
        if isinstance(actions, (list, tuple)) and len(actions) == 2:
            index = int(self.smoothed_frn_detected)
            move_or_call(actions[index])
        elif callable(actions):
            actions()

    def control(self):
        if self.smoothed_foe_detected:
            self.set_rgb("yellow")
        elif self.smoothed_frn_detected:
            self.set_rgb("green")
        else:
            self.set_rgb("blue")

        self.stuck.cycle(self.distance, self.smoothed_foe_detected)

    def main_loop(self):
        raw_img = np.zeros((480, 640, 3), dtype=np.uint8)
        frame_clean = raw_img.copy() 
        
        self.foe_detected = True 
        self.foe_position = 0.1
        self.foe_area = 250 
        
        self.smoothed_foe_detected = self.foe_boolean_detection_averager(self.foe_detected) 
        
        distance = self.sonar.getDistance()
        if round(distance) == 5000:
            distance = float('inf')
        elif distance > 5000:
            distance = float('nan')
        self.distance = distance
        
        self.control_wrapper()

def get_parser(parser, subparsers=None):
    parser = argparse.ArgumentParser() if parser is None else parser
    parser.add_argument('--mode', help="mode to start in", default='idle')
    parser.add_argument('--nolog', action="store_true", help="disable logging") 
    return parser, subparsers

parser = argparse.ArgumentParser() 
parser, _ = get_parser(parser) 
args = parser.parse_args([]) 
args.nolog = True 
args.start_paused = False  
args.project = "rotate"    
args.root = "/home/pi"     
args.dry_run = False       
program = RotateProgram(args)

for _ in range(5): 
    program.main_loop() 
    time.sleep(0.1) 
