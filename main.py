#!/usr/bin/python
import csv
import json
from collections import namedtuple, deque
import numpy as np
from numpy.linalg import norm
from transformations import affine_matrix_from_points

from util import normalize_angle, NumpyEncoder

Waypoint = namedtuple('Waypoint', ['position'])

hit_radius = 0.05
cruise_speed = 0.2

def load_waypoint_file(waypoint_file):
    with open(waypoint_file) as csv_file:
        csv_reader = csv.reader(csv_file, delimiter=',')
        return [Waypoint(np.array([float(row[0]), float(row[1])])) for row in csv_reader]

class Logger:
    def __init__(self, fname):
        self.file = open(fname, 'w')

    def log(self, current):
        json.dump(current.__dict__, self.file, cls=NumpyEncoder)
        self.file.write('\n')

class Estimator:
    def __init__(self):
        self.max_points = 900
        self.marvelmind_buffer = np.zeros((2, self.max_points))
        self.realsense_buffer = np.zeros((2, self.max_points))
        self.buffer_i = 0
        self.npoints = 0
        self.transform = None
        self.angle = None
        self.last_update_time = 0

    def add_point(self, state):
        self.realsense_buffer[:,self.buffer_i] = state.realsense_position
        self.marvelmind_buffer[:,self.buffer_i] = state.marvelmind_position
        self.buffer_i = (self.buffer_i + 1) % self.max_points
        self.npoints += 1

    def update_matrix(self, state):
        buffer_len = min(self.max_points, self.npoints)
        self.transform = affine_matrix_from_points(self.realsense_buffer[:,:buffer_len], self.marvelmind_buffer[:,:buffer_len], shear=False, scale=True)
        self.angle = np.arctan2(self.transform[1,0], self.transform[0,0])

        state.rs_to_mm_scale = norm(self.transform[:2, 0])
        state.rs_to_mm_offset = self.transform[:2, 2]
        state.rs_to_mm_angle = self.angle

        self.last_update_time = state.time

    def update(self, state):
        self.add_point(state)

        if state.time - self.last_update_time > 1.0 and self.npoints > 20:
            self.update_matrix(state)
            
        if self.npoints >= 120:
            hpos = self.transform @ np.append(state.realsense_position, 1)
            state.position = hpos[:-1] / hpos[-1]
            state.heading = normalize_angle(state.realsense_heading + self.angle)
        else:
            state.position = None
            state.heading = None

def run_loop(robot, behavior, logger):
    robot.start()
    estimator = Estimator()

    try:
        while True:
            robot.update()
            estimator.update(robot.state)
            try:
                next(behavior)
            except StopIteration:
                break
            logger.log(robot.state)
    finally:
        robot.stop()

def drive_to_waypoint(state, waypoint):
    while norm(waypoint.position - state.position) > hit_radius:
        delta = waypoint.position - state.position
        bearing = np.arctan2(delta[1], delta[0])
        delta_heading = normalize_angle(bearing - state.heading)

        state.target_delta_heading = delta_heading
        state.target_bearing = bearing

        if np.abs(delta_heading) < np.pi/4:
            state.drive_speed = cruise_speed
        else:
            state.drive_speed = 0

        state.drive_angle = np.arctan(delta_heading) * 2
        yield

def drive_to_waypoints(state, waypoints, repeat):
    # Drive forward until estimator produces a position
    print("Starting calibration drive")
    while state.position is None:
        state.drive_speed = cruise_speed
        state.drive_angle = 0
        yield
    
    while True:
        for i, waypoint in enumerate(waypoints):
            print(f"Going to waypoint {i + 1}: {waypoint.position}")
            yield from drive_to_waypoint(state, waypoint)
        if not repeat:
            break

def forward_rs(state, dist):
    rs_initial = state.realsense_position
    while norm(rs_initial - state.realsense_position) < dist:
        state.drive_speed = cruise_speed
        state.drive_angle = 0
        yield

def turn_rs(state, angle):
    h_initial = state.realsense_heading
    while np.abs(normalize_angle(h_initial - state.realsense_heading)) < angle:
        state.drive_speed = 0
        state.drive_angle = np.pi / 2
        yield

def square_test(state):
    for i in range(16):
        print("Forward")
        yield from forward_rs(state, 1)
        print("Turn")
        yield from turn_rs(state, np.pi / 2)

def monitor(state):
    import curses, time
    try:
        scr = curses.initscr()
        curses.cbreak()
        curses.noecho()
        scr.keypad(1)
        scr.nodelay(True)

        while True:
            key = None
            while True:
                try:
                    key = scr.getkey()
                except curses.error:
                    break
            
            scr.addstr(0, 0, "Marvelmind: "
                + f"x:{state.marvelmind_position[0]:+2.3f} "
                + f"y:{state.marvelmind_position[1]:+2.3f} "
            )
            scr.addstr(1, 0, "Realsense:  "
                + f"x:{state.realsense_position[0]:+2.3f} "
                + f"y:{state.realsense_position[1]:+2.3f} "
                + f"a:{state.realsense_heading:+1.3f} "
                + f"tracker:{state.rs_tracker_confidence:+1.3f} "
                + f"mapper:{state.rs_mapper_confidence:+1.3f} "
            )
            if state.position is not None:
                scr.addstr(2, 0, f"Estimate:   x:{state.position[0]:+2.3f} y:{state.position[1]:+2.3f}, a:{state.heading:+1.3f}")

            state.drive_angle = 0
            state.drive_speed = 0
            if key == 'KEY_UP':
                state.drive_speed = cruise_speed
            elif key == 'KEY_DOWN':
                state.drive_speed = -cruise_speed
            elif key == 'KEY_LEFT':
                state.drive_angle = np.pi / 2
            elif key == 'KEY_RIGHT':
                state.drive_angle = -np.pi / 2
            yield
            time.sleep(1./25)
    finally:
        curses.nocbreak()
        curses.echo()
        curses.endwin()
    
 
if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--sim", help="Use simulated robot", action="store_true")
    parser.add_argument("--waypoints", help="waypoints CSV filename")
    parser.add_argument("--waypoints-repeat", help="waypoints CSV filename")
    parser.add_argument("--square")
    parser.add_argument("--datalog", help="output datalog csv filename")
    args = parser.parse_args()

    if args.sim:
        import sim
        robot = sim.SimRobot()
    else:
        import hardware
        robot = hardware.Robot()

    if args.waypoints or args.waypoints_repeat:
        waypoints = load_waypoint_file(args.waypoints or args.waypoints_repeat)
        behavior = drive_to_waypoints(robot.state, waypoints, bool(args.waypoints_repeat))
    elif args.square:
        behavior = square_test(robot.state)
    else:
        behavior = monitor(robot.state)

    logger = Logger(args.datalog)
    run_loop(robot, behavior, logger)
 