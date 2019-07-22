import numpy as np
from util import State, normalize_angle

class SimRobot:
    def __init__(self):
        self.state = State(
            drive_speed = 0,
            drive_angle = 0
        )
        self.position = self.initial_position = np.array([0, 0])
        self.heading = self.initial_heading = 0

        self.time = 0

    def update(self):
        dt = 1./15
        velocity = self.state.drive_speed * np.array([np.cos(self.heading), np.sin(self.heading)])
        self.position = self.position + velocity * dt
        self.heading = normalize_angle(self.heading + self.state.drive_angle * dt)
        self.time += dt

        self.state.time = self.time
        
        self.state.sim_position = self.position
        self.state.sim_heading = self.heading

        self.state.marvelmind_position = self.position + np.random.normal(0,0.1,2)

        self.state.realsense_position = self.position - self.initial_position
        self.state.realsense_heading  = normalize_angle(self.heading - self.initial_heading)

    def start(self):
        pass

    def stop(self):
        self.speed = 0
        self.drive_angle = 0

