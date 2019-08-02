import time
import numpy as np
import pyrealsense2
import transformations
from serial.tools.list_ports import comports

from roboclaw import Roboclaw
from marvelmind import MarvelmindHedge
from util import State

H_aeroRef_T265Ref = np.array([[0,0,-1,0],[1,0,0,0],[0,-1,0,0],[0,0,0,1]])
H_T265body_aeroBody = np.linalg.inv(H_aeroRef_T265Ref)

class RealSense:
    def __init__(self):
        self.pipe = pyrealsense2.pipeline()

    def start(self):
        cfg = pyrealsense2.config()
        cfg.enable_stream(pyrealsense2.stream.pose)
        profile = cfg.resolve(self.pipe)
        dev = profile.get_device()
        sensor = dev.first_pose_sensor()
        sensor.set_option(pyrealsense2.option.enable_pose_jumping, False)
        sensor.set_option(pyrealsense2.option.enable_relocalization, False)
        self.pipe.start(cfg)

    def stop(self):
        self.pipe.stop()

    def read(self):
        frames = self.pipe.wait_for_frames()
        pose = frames.get_pose_frame()
        
        if not pose:
            return None

        data = pose.get_pose_data()
        y = -1 * data.translation.x
        x = -1 * data.translation.z

        H_T265Ref_T265body = transformations.quaternion_matrix([data.rotation.w, data.rotation.x,data.rotation.y,data.rotation.z])
        H_aeroRef_aeroBody = H_aeroRef_T265Ref.dot( H_T265Ref_T265body.dot( H_T265body_aeroBody ))
        rpy_rad = np.array(transformations.euler_from_matrix(H_aeroRef_aeroBody, 'rxyz'))
        heading = -rpy_rad[2]

        return np.array([x, y]), heading, data.tracker_confidence, data.mapper_confidence


marvelmind_vid = 1155   # VID of Marvelmind device

class Marvelmind:
    def __init__(self, adr):
        for port in comports():
            if port.vid == marvelmind_vid:
                self.hedge = MarvelmindHedge(tty = port.device, adr=adr, debug=False)
                break
        else:
            raise IOError("Marvelmind position sensor not found")

        self.hedge.start()

    def read(self):
        hedge_id, x, y, z, angle, timestamp = self.hedge.position()
        return np.array([x, y])

    def stop(self):
        self.hedge.stop()

class Motors:
    def __init__(self, config):
        roboclaw_vid = 0x03EB   # VID of Roboclaw motor driver in hex
        for port in comports():
            if port.vid == roboclaw_vid:
                self.rc = Roboclaw(port.device, 0x80)
                break
        else:
            raise IOError("Roboclaw motor driver not found")

        self.rc.Open()
        self.address = 0x80
        version = self.rc.ReadVersion(self.address)

        self.l_ticks_per_m = config['ticks_per_m']['l'] #  number of left encoder ticks per m traveled
        self.r_ticks_per_m = config['ticks_per_m']['r'] #  number of right encoder ticks per m traveled
        self.track_width = config['track_width'] # width between the two tracks, in m

        self.mapping = config['mapping']
        if self.mapping not in ('rl', 'lr'):
            raise ValueError("Invalid motor mapping '{self.mapping}'")

        self.last_setpoint = None

        if version[0] == False:
            raise IOError("Roboclaw motor driver: GETVERSION failed")
        else:
            print(f"Roboclaw: {version[1]}")

    def drive(self, speed, angle):
        """
        Set the speed of the robot. They maintain this speed until stopped.
        speed: Forward speed in m/s
        angle: Clockwise angular rate in radians/s
        """

        max_angle = np.pi / 2
        angle = max(-max_angle, min(angle, max_angle))

        left_speed  = int((speed - self.track_width/2 * angle) * self.l_ticks_per_m)
        right_speed = int((speed + self.track_width/2 * angle) * self.r_ticks_per_m)

        if self.last_setpoint != (left_speed, right_speed):
            self.last_setpoint = (left_speed, right_speed)
            if self.mapping == "rl":
                self.rc.SpeedM1M2(self.address, right_speed, left_speed)
            else:
                self.rc.SpeedM1M2(self.address, left_speed, right_speed)
                

    def stop(self):
        self.last_setpoint = None
        self.rc.ForwardM1(self.address, 0)
        self.rc.ForwardM2(self.address, 0)

class Robot:
    def __init__(self, config):
        self.state = State(
            time = 0.0,
            drive_speed = 0.0,
            drive_angle = 0.0,
        )
        self.marvelmind = Marvelmind(config['marvelmind']['addr'])
        self.realsense = RealSense()
        self.motors = Motors(config['motors'])

    def start(self):
        self.realsense.start()
        self.start_time = time.time()

    def stop(self):
        self.motors.stop()
        self.realsense.stop()
        self.marvelmind.stop()

    def update(self):
        self.motors.drive(self.state.drive_speed, self.state.drive_angle)

        elapsed_time = (time.time() - self.start_time) - self.state.time
        loop_time = 1./20

        if loop_time > elapsed_time:
            time.sleep(loop_time - elapsed_time)

        self.state.time = time.time() - self.start_time

        self.state.marvelmind_position = self.marvelmind.read()
        (self.state.realsense_position,
         self.state.realsense_heading,
         self.state.rs_tracker_confidence,
         self.state.rs_mapper_confidence) = self.realsense.read()
