#!/usr/bin/python
import csv
import json
from collections import namedtuple, deque
import numpy as np
from numpy.linalg import norm
from transformations import affine_matrix_from_points

from util import normalize_angle, NumpyEncoder

Waypoint = namedtuple('Waypoint', ['position'])

hit_radius = 0.1
cruise_speed = 0.045

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

        if state.time - self.last_update_time > 1.0:
            self.update_matrix(state)
            
        if self.npoints >= 200:
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
        state.drive_speed = (cruise_speed if np.abs(delta_heading) < 0.02 else 0.0)
        state.drive_angle = delta_heading
        yield

def drive_to_waypoints(state, waypoints):
    # Drive forward until estimator produces a position
    while state.position is None:
        state.drive_speed = cruise_speed / 2
        state.drive_angle = 0
        yield
        
    for waypoint in waypoints:
        yield from drive_to_waypoint(state, waypoint)

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--sim", help="Use simulated robot", action="store_true")
    parser.add_argument("--waypoints", help="waypoints CSV filename")
    parser.add_argument("--datalog", help="output datalog csv filename")
    args = parser.parse_args()

    if args.sim:
        import sim
        robot = sim.SimRobot()
    else:
        import hardware
        robot = hardware.Robot()

    waypoints = load_waypoint_file(args.waypoints)
    behavior = drive_to_waypoints(robot.state, waypoints)
    logger = Logger(args.datalog)
    run_loop(robot, behavior, logger)
 