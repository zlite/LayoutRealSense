#!/usr/bin/python
# -*- coding: utf-8 -*-
## License: Apache 2.0. See LICENSE file in root directory.
## Copyright(c) 2019 Intel Corporation. All Rights Reserved.

#####################################################
##           librealsense T265 example             ##
#####################################################

# First import the library
import pyrealsense2 as rs
import serial
import time
ser = serial.Serial('/dev/ttyACM0', 9600)

waypoint_file = 'waypoints_office.csv'
waypoint_num = 0
waypoint=[[0 for j in range(2)] for i in range(1000)]  # dimension an array up to 1,000 waypoints

# Declare RealSense pipeline, encapsulating the actual device and sensors
pipe = rs.pipeline()

# Build config object and request pose data
cfg = rs.config()
cfg.enable_stream(rs.stream.pose)

# Start streaming with requested config
pipe.start(cfg)

with open(waypoint_file) as csv_file:  # change to whatever waypoint file you want
    csv_reader = csv.reader(csv_file, delimiter=',')
    line_count = 0
    for row in csv_reader:
        waypoint[line_count][0] = float(row[0]) + x_offset # x data
        waypoint[line_count][1] = float(row[1]) + y_offset # y data
        line_count += 1
    print('Loaded', line_count, 'waypoints')
    waypoint_total = line_count

def drive(left, right):
    left = str(left)
    right = str(right)
    ser.write(left)
    ser.write("L")
    ser.write(right)
    ser.write("R")

def constrain(value, min, max):
    if value < min :
        return min
    if value > max :
        return max
    else:
        return value

def dir(heading, desired_angle):
    if (heading >= desired_angle):
        if (heading - desired_angle) >= 180:
            direction = 1  # clockwise
        else:
            direction = -1  # counterclockwise
    else:
        if abs(heading - desired_angle) >= 180:
            direction = -1  # counterclockwise
        else:
            direction = 1  # clockwise
    direction = steering_dir * direction # reverse if needed for robot
    return direction


def navigate(x,y,heading):
    delta_x = waypoint[waypoint_num][0] - x  # calculate angle to target
    delta_y = waypoint[waypoint_num][1] - y
    range = math.sqrt(delta_y**2 + delta_x**2)
    desired_angle = math.degrees(math.atan2(delta_y,delta_x))  # all converted into degrees
    delta_x = x - old_x
    delta_y = y - old_y
    heading2 = math.degrees(math.atan2(delta_y,delta_x))  #  now get angle from position since last measurement
    old_x = x
    old_y = y
    direction = dir(heading, desired_angle)

try:
    while True:
        # Wait for the next set of frames from the camera
        frames = pipe.wait_for_frames()

        # Fetch pose frame
        pose = frames.get_pose_frame()
        if pose:
            # Print some of the pose data to the terminal
            data = pose.get_pose_data()
            print("Frame #{}".format(pose.frame_number))
            print("Position: {}".format(data.translation))
            print("Velocity: {}".format(data.velocity))
            print("Acceleration: {}\n".format(data.acceleration))
            drive(100,100)

except KeyboardInterrupt:
    left = str(0)
    right = str(0)
    ser.write(left)
    ser.write("L")
    ser.write(right)
    ser.write("R")
    pipe.stop()
