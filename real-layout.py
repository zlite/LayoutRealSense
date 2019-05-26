#!/usr/bin/python
# -*- coding: utf-8 -*-
## License: Apache 2.0. See LICENSE file in root directory.
## Copyright(c) 2019 Intel Corporation. All Rights Reserved.

#####################################################
# Sample of using RealSense T265 with Roboclaw motor controller
#####################################################

# First import the library
import pyrealsense2 as rs
import serial
import time
import math
import transformations as tf
import csv
import numpy as np
from roboclaw import Roboclaw

#Windows comport name
#rc = Roboclaw("COM3",115200)
#Linux comport name
rc = Roboclaw("/dev/ttyACM0",0x80)
waypoint_file = 'waypoints_office.csv'
waypoint_num = 0
waypoints = 0
waypoint=[[0 for j in range(2)] for i in range(1000)]  # dimension an array up to 1,000 waypoints
x_offset = 0
y_offset = 0
x = 0
y = 0
cruise_speed = 40
old_x = 0
old_y = 0
steering_dir = 1  # +/- 1 for direction of robot motors
testmode = False
# Declare RealSense pipeline, encapsulating the actual device and sensors
pipe = rs.pipeline()
H_aeroRef_T265Ref = np.array([[0,0,-1,0],[1,0,0,0],[0,-1,0,0],[0,0,0,1]])
H_T265body_aeroBody = np.linalg.inv(H_aeroRef_T265Ref)

# Build config object and request pose data
cfg = rs.config()
cfg.enable_stream(rs.stream.pose)

# Start streaming with requested config
pipe.start(cfg)


rc.Open()    # Start motor controller
address = 0x80
version = rc.ReadVersion(address)
tickdistanceL = 10 #  number of left encoder ticks per mm traveled
tickdistanceR = 10 #  number of right encoder ticks per mm traveled
if version[0]==False:
	print ("GETVERSION Failed")
else:
	print (repr(version[1]))


with open(waypoint_file) as csv_file:  # change to whatever waypoint file you want
    csv_reader = csv.reader(csv_file, delimiter=',')
    line_count = 0
    for row in csv_reader:
        waypoint[line_count][0] = float(row[0]) + x_offset # x data
        waypoint[line_count][1] = float(row[1]) + y_offset # y data
        line_count += 1
    print('Loaded', line_count, 'waypoints')
    waypoint_total = line_count

def displayspeed():
	enc1 = rc.ReadEncM1(address)
	enc2 = rc.ReadEncM2(address)
	speed1 = rc.ReadSpeedM1(address)
	speed2 = rc.ReadSpeedM2(address)
	print ("Encoder1:", enc1[1], "Encoder2:", enc2[1])
	print ("Speed1:", speed1[1], "Speed2:", speed2[1])


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
    H_T265Ref_T265body = tf.quaternion_matrix([data.rotation.w, data.rotation.x,data.rotation.y,data.rotation.z]) # in transformations, Quaternions w+ix+jy+kz are represented as [w, x, y, z]!
    # transform to aeronautic coordinates (body AND reference frame!)
    H_aeroRef_aeroBody = H_aeroRef_T265Ref.dot( H_T265Ref_T265body.dot( H_T265body_aeroBody ))
    rpy_rad = np.array( tf.euler_from_matrix(H_aeroRef_aeroBody, 'rxyz') )
    heading = rpy_rad[2]*180/math.pi
    if heading < 0: heading = 360+heading
#    print ("Heading", round(heading))
#     qw = data.rotation.w # Realsense IMU data quaternians
#     qx = data.rotation.x
#     qy = -1 * data.rotation.y
#     qz = -1 * data.rotation.z
#     yaw = math.atan2(2.0*(qy*qz + qw*qx), qw*qw - qx*qx - qy*qy + qz*qz)
#     pitch = math.asin(-2.0*(qx*qz - qw*qy))
#     roll = math.atan2(2.0*(qx*qy + qw*qz), qw*qw + qx*qx - qy*qy - qz*qz)
#     yaw *= 180.0 / math.pi  # convert to degrees
#     pitch *= 180.0 / math.pi
#     roll *= 180.0 / math.pi
#     if yaw < 0: yaw += 360.0  # Ensure yaw stays between 0 and 360
#     if pitch < 0: pitch += 360.0  # Ensure yaw stays between 0 and 360
#     if roll < 0: roll += 360.0  # Ensure yaw stays between 0 and 360
#     print ("Yaw", round(yaw), "Pitch", round(pitch), "Roll", round(roll))
# #    heading = math.atan2(2.0 * (QZ * QW + QX * QY), 2.0 * (QW * QW + QX * QX))
#     heading = pitch
#     print ("Heading", heading)
#     if heading < 0: heading += 360.0  # Ensure yaw stays between 0 and 360
    return heading

def drive(angle, speed):
	left_speed = int((100 * cruise_speed) + (angle * 10))
	right_speed = int((100 * cruise_speed) - (angle * 10))
	print ("Left: ", left_speed, "Right: ", right_speed)
	rc.SpeedM1(address,left_speed)
	rc.SpeedM2(address,right_speed)

def navigate(x,y,heading):
        global waypoint_num, waypoints
        delta_x = waypoint[waypoint_num][0] - x  # calculate angle to target
        delta_y = waypoint[waypoint_num][1] - y
        range = math.sqrt(delta_y**2 + delta_x**2)
        desired_angle = math.degrees(math.atan2(delta_y,delta_x))  # all converted into degrees
        #    direction = dir(heading, desired_angle)
        delta_angle = desired_angle - heading
        print ("Current heading", round(heading), "Waypoint angle", round(desired_angle), "Steer angle", round(delta_angle))
        drive (delta_angle, cruise_speed)
        print ("Range", round(range,3))
        if range < 0.1:
                waypoint_num = waypoint_num + 1
                if waypoint_num > 3:
                    waypoint_num = 0   # start over from beginning of waypoints



# main loop

try:
        while True:        
                if (testmode):
                        while True:
                                rc.ResetEncoders(address)
                                buffers = (0,0,0)
                                displayspeed()
                                time.sleep(2)
                                if (waypoints == 0): # straight
                                        rc.SpeedDistanceM1(address,2000,500*tickdistanceL,1)
                                        rc.SpeedDistanceM2(address,2000,500*tickdistanceR,1)
                                if (waypoints == 1): # turn
                                        rc.SpeedDistanceM1(address,-2000,200*tickdistanceL,1)
                                        rc.SpeedDistanceM2(address,2000,200*tickdistanceR,1)
                                if (waypoints == 2):
                                        rc.SpeedDistanceM1(address,-2000,500*tickdistanceL,1)
                                        rc.SpeedDistanceM2(address,2000,500*tickdistanceR,1)
                                if (waypoints == 3):
                                        rc.SpeedDistanceM1(address,2000,1000*tickdistanceL,1)
                                        rc.SpeedDistanceM2(address,2000,1000*tickdistanceR,1)
                                while(buffers[1]!=0x80 and buffers[2]!=0x80):   #Loop until distance command has completed
                                        print ("Buffers: ", buffers[1]," ", buffers[2])
                                        displayspeed()
                                        buffers = rc.ReadBuffers(address)
                                print ("Next waypoint")
                                if (waypoints < 1):
                                        waypoints = waypoints + 1
                                else:
                                        waypoints = 0
                if (not testmode):                        
                        frames = pipe.wait_for_frames()
                        pose = frames.get_pose_frame()
                        if pose:
                                #            yaw = pose.QueryYaw()
                                data = pose.get_pose_data()
                                x = data.translation.x
                                y = data.translation.z # don't ask me why, but in "VR space", y is z
                                print("Current X", round(x,2),"Y", round(y,2))
                                print("Waypoint #", waypoint_num, " Target X", round(waypoint[waypoint_num][0],2),"Y", round(waypoint[waypoint_num][1],2))
                                heading = get_heading()
                                time.sleep(0.2) # don't flood the print buffer
                                #            print ("heading", round(heading,2))
                                navigate(x,y,heading)

except KeyboardInterrupt:
        rc.ForwardM1(address,0)
        rc.ForwardM2(address,0)
        pipe.stop()
