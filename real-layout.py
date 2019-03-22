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
import math
import csv
ser = serial.Serial('/dev/ttyACM0', 9600)

waypoint_file = 'waypoints_office.csv'
waypoint_num = 0
waypoint=[[0 for j in range(2)] for i in range(1000)]  # dimension an array up to 1,000 waypoints
x_offset = 0
y_offset = 0
x = 0
y = 0
cruise_speed = 40
old_x = 0
old_y = 0
steering_dir = 1  # +/- 1 for direction of robot motors
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

def drive(angle):
    left = cruise_speed - angle/5
    right = cruise_speed + angle/5
    left = str(left)
    right = str(right)
    print ("Left", left,"Right", right)
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

def get_heading():
    qw = data.rotation.w # Realsense IMU data quaternians
    qx = data.rotation.x
    qy = -1 * data.rotation.y
    qz = -1 * data.rotation.z
    yaw = math.atan2(2.0*(qy*qz + qw*qx), qw*qw - qx*qx - qy*qy + qz*qz)
    pitch = math.asin(-2.0*(qx*qz - qw*qy))
    roll = math.atan2(2.0*(qx*qy + qw*qz), qw*qw + qx*qx - qy*qy - qz*qz)
    yaw *= 180.0 / math.pi  # convert to degrees
    pitch *= 180.0 / math.pi
    roll *= 180.0 / math.pi
    if yaw < 0: yaw += 360.0  # Ensure yaw stays between 0 and 360
    if pitch < 0: pitch += 360.0  # Ensure yaw stays between 0 and 360
    if roll < 0: roll += 360.0  # Ensure yaw stays between 0 and 360
    print ("Yaw", round(yaw), "Pitch", round(pitch), "Roll", round(roll))
#    heading = math.atan2(2.0 * (QZ * QW + QX * QY), 2.0 * (QW * QW + QX * QX))
    heading = pitch
    print ("Heading", heading)
    if heading < 0: heading += 360.0  # Ensure yaw stays between 0 and 360
    return heading

def navigate(x,y,heading):
    global waypoint_num
    delta_x = waypoint[waypoint_num][0] - x  # calculate angle to target
    delta_y = waypoint[waypoint_num][1] - y
    range = math.sqrt(delta_y**2 + delta_x**2)
    desired_angle = math.degrees(math.atan2(delta_y,delta_x))  # all converted into degrees
    direction = dir(heading, desired_angle)
    delta_angle = direction * (desired_angle - heading)
    print ("steer angle", round(delta_angle,3))
    drive (delta_angle)
    print ("Range", round(range,3))
    if range < 20:
        waypoint_num = waypoint_num + 1
        if waypoint_num > 3:
            waypoint_num = 0   # start over from beginning of waypoints

try:
    while True:
        # Wait for the next set of frames from the camera
        frames = pipe.wait_for_frames()
        # Fetch pose frame
        pose = frames.get_pose_frame()
        if pose:
            # Print some of the pose data to the terminal
            data = pose.get_pose_data()
#            yaw = pose.QueryYaw()
            x = data.translation.x
            y = data.translation.z # don't ask me why, but in "VR space", y is z
            print("Current X", round(x,2),"Y", round(y,2))
            print("Target X", round(waypoint[waypoint_num][0],2),"Y", round(waypoint[waypoint_num][1],2))
            heading = get_heading()
            time.sleep(0.1) # don't flood the print buffer
#            print ("heading", round(heading,2))
#            navigate(x,y,heading)

except KeyboardInterrupt:
    left = str(0)  # set motors to zero
    right = str(0)
    ser.write(left)
    ser.write("L")
    ser.write(right)
    ser.write("R")
    pipe.stop()
