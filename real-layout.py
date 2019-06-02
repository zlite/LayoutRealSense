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
from serial.tools.list_ports import comports
import time
import math
import transformations as tf
import csv
import numpy as np
from roboclaw import Roboclaw
from simple_pid import PID
from marvelmind import MarvelmindHedge
import sys
sys.path.append('../')

pid = PID(1, 0.1, 0.05, setpoint=1)
pid.sample_time = 0.1  # update every 0.1 seconds


tickdistanceL = 10 #  number of left encoder ticks per mm traveled
tickdistanceR = 10 #  number of right encoder ticks per mm traveled
waypoint_file = 'waypoints_office.csv'
waypoint_num = 0
waypoints = 0
waypoint=[[0 for j in range(2)] for i in range(1000)]  # dimension an array up to 1,000 waypoints
x_offset = 0
y_offset = 0
x = 0
y = 0
use_marvelmind = True
testmode = False
recordmode = False
hedgehog_id = 1
cruise_speed = 15
old_x = 0
old_y = 0
new_waypoint = False
steering_dir = -1  # +/- 1 for direction of robot motors
steering_gain = 10
# Declare RealSense pipeline, encapsulating the actual device and sensors
pipe = rs.pipeline()
H_aeroRef_T265Ref = np.array([[0,0,-1,0],[1,0,0,0],[0,-1,0,0],[0,0,0,1]])
H_T265body_aeroBody = np.linalg.inv(H_aeroRef_T265Ref)

# Build config object and request pose data
cfg = rs.config()
cfg.enable_stream(rs.stream.pose)

# Start streaming with requested config
pipe.start(cfg)


roboclaw_vid = 0x03EB   # VID of Roboclaw device in hex
found = False
for port in comports():
	if port.vid == roboclaw_vid:
		roboclawport = port.device
		found = True
	if found == True:
		print ("Roboclaw port:", roboclawport)
		rc = Roboclaw(roboclawport,0x80)

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

# initiate local positioning
if use_marvelmind:
    marvelmind_vid = 0x1155   # VID of Marvelmind devices in hex
    found = False
    for port in comports():
        if port.vid == marvelmind_vid:
            hedgeport = port.device
            found = True
    if found == True:
        print ("Marvelmind port:", hedgeport)
        hedge = MarvelmindHedge(tty = hedgeport,adr=hedgehog_id, debug=False) # create MarvelmindHedge thread
        hedge.start() # start thread
        time.sleep(1) # pause to let it settle
    else:
        print ("Marvelmind not found")

def get_position():
    global hedgehog_x
    global hedgehog_y
    position = hedge.position()
    hedgehog_x = position[1] + x_offset
    hedgehog_y = position[2] + y_offset

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

def rotate (desired_angle):
        print('Starting rotation')
        heading = 999  # initialize
        while (heading < desired_angle - 1) or (heading > desired_angle + 1): 
            frames = pipe.wait_for_frames()
            pose = frames.get_pose_frame()
            if pose:
                data = pose.get_pose_data()
                heading = get_heading(data)
                delta_angle = heading-desired_angle  # get the difference between the current and intended angle
                direction = dir(heading, desired_angle)
                print ("Current heading:", round(heading,3), "Desired angle:", round(desired_angle), "Direction: ", direction)
                rc.ResetEncoders(address)
                rc.SpeedDistanceM1(address,-1*direction*2500,1*tickdistanceL,1) # rotate a few degrees
                rc.SpeedDistanceM2(address,direction*2500,1*tickdistanceR,1)
                buffers = (0,0,0)
                while(buffers[1]!=0x80 and buffers[2]!=0x80):   #Loop until distance command has completed
 #                   displayspeed()
                    buffers = rc.ReadBuffers(address)


def get_heading(data):  # this is essentially magic ;-)
    H_T265Ref_T265body = tf.quaternion_matrix([data.rotation.w, data.rotation.x,data.rotation.y,data.rotation.z]) # in transformations, Quaternions w+ix+jy+kz are represented as [w, x, y, z]!
    # transform to aeronautic coordinates (body AND reference frame!)
    H_aeroRef_aeroBody = H_aeroRef_T265Ref.dot( H_T265Ref_T265body.dot( H_T265body_aeroBody ))
    rpy_rad = np.array( tf.euler_from_matrix(H_aeroRef_aeroBody, 'rxyz') )
    heading = rpy_rad[2]*180.00/math.pi
    return heading

def drive(speed, angle):
        constrain(angle, -20, 20)
        left_speed = int(cruise_speed*100 + angle*200)
        right_speed = int(cruise_speed*100 - angle*200)
        rc.SpeedM1(address,left_speed)
        rc.SpeedM2(address,right_speed)
        frames = pipe.wait_for_frames()
        pose = frames.get_pose_frame()
        if pose:
                data = pose.get_pose_data()
                x = data.translation.x
                y = -1 * data.translation.z # don't ask me why, but in "VR space", y is z and it's reversed
 #               print("Going straight: Current X", round(x,2),"Y", round(y,2))


# main loop

try:
        while True:
                if (recordmode):
                        temp = raw_input("Hit enter to continue")
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
                                buffers = (0,0,0)
                                while(buffers[1]!=0x80 and buffers[2]!=0x80):   #Loop until distance command has completed
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
                                y = -1.000 * data.translation.z # don't ask me why, but in "VR space", y is z and it's reversed
                                print("Current X", round(x,2),"Y", round(y,2))
                                print("Waypoint #", waypoint_num, " Target X", round(waypoint[waypoint_num][0],2),"Y", round(waypoint[waypoint_num][1],2))
                                heading = get_heading(data)
                                time.sleep(0.1) # don't flood the print buffer
                                #            print ("heading", round(heading,2))
                                delta_x = waypoint[waypoint_num][0] - x  # calculate distance to target
                                delta_y = waypoint[waypoint_num][1] - y
#                                print ("Delta X: ", round(delta_x,2), "Y: ", round(delta_y,2))
                                range = math.sqrt(delta_y**2 + delta_x**2)
                                desired_angle = 90.000-math.degrees(math.atan2(delta_y,delta_x))  # all converted into degrees     
                                if desired_angle > 180:
                                        desired_angle = -1 * (360.000-desired_angle) # turning counterclockwise is negative
                                turn_angle = abs(desired_angle-heading) # get difference between desired angle and current angle
                                raw_turn_angle = turn_angle   # this is just for debugging
                                if desired_angle < heading: # these next lines correct for all the edge cases and singularities 
                                        turn_angle = -1 * turn_angle
                                if turn_angle > 180:
                                        turn_angle = -360 + turn_angle
                                if turn_angle < -180:
                                        turn_angle = 360 + turn_angle
                                print ("Waypoint angle ", round(desired_angle), "Current heading ", round(heading), "Raw Turn Angle", round(raw_turn_angle,2), "Corrected Turn angle ", round(turn_angle,2), "Range ", round(range,2))
                                if (range > 0.1) and (not new_waypoint):
                                        drive (cruise_speed,turn_angle) # Steer towards waypoint
                                if new_waypoint:
                                        rotate(desired_angle)
                                        new_waypoint = False
                                        rc.ResetEncoders(address)
                                if range <= 0.05:
                                        print ("Hit waypoint")
                                        waypoint_num = waypoint_num + 1
                                        if waypoint_num > 3:
                                            waypoint_num = 0   # start over from beginning of waypoints
                                        new_waypoint = True
                                            

except KeyboardInterrupt:
        rc.ForwardM1(address,0)  # kill motors
        rc.ForwardM2(address,0)
        pipe.stop()
