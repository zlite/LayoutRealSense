#!/usr/bin/python
import csv
import json
from collections import namedtuple, deque
import numpy as np
from numpy.linalg import norm

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
        self.buffer = deque(maxlen = 200)

    def update(self, state):
        self.buffer.append(state.realsense_position)

        if len(self.buffer) >= 200:
            # Fake, estimate is just taken from the simulator's real position
            state.position = state.sim_position
            state.heading = state.sim_heading
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
 