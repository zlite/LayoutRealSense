#!/usr/bin/python
# -*- coding: utf-8 -*-
## License: Apache 2.0.

import pyrealsense2 as rs
import serial
from serial.tools.list_ports import comports
import time
import math
import transformations as tf
import csv
import numpy as np
from roboclaw_3 import Roboclaw
from marvelmind import MarvelmindHedge
import sys
sys.path.append('../')

#from simple_pid import PID
#pid = PID(1, 0.1, 0.05, setpoint=1)    # setting the P, I and D terms
#pid.sample_time = 0.1  # update every 0.1 seconds


tickdistanceL = 10 #  number of left encoder ticks per mm traveled
tickdistanceR = 10 #  number of right encoder ticks per mm traveled
#waypoint_file = 'waypoints_office.csv'
waypoint_file = 'waypoints_wework.csv'
#waypoint_file = 'waypoints-wework.csv'
record_file = 'waypoints_recorded.csv'
datalog_file = 'datalog.csv'
waypoint_num = 0
waypoints = 0
waypoint=[[0 for j in range(2)] for i in range(1000)]  # dimension an array up to 1,000 waypoints
x = 0
y = 0
hedgehog_x = 0
hedgehog_y = 0
use_marvelmind = True
testmode = False
recordmode = False
datalog = True
hit_radius = 0.1
hedgehog_id = 6
cruise_speed = 45
steering_nudge = 200 # speed compensation for left/right imbalance when going straight in calibration
old_x = 0
old_y = 0
last_calibration = 0
if use_marvelmind:
	new_waypoint = True
else:
	new_waypoint = False        
steering_dir = -1  # +/- 1 for direction of robot motors
steering_gain = 250
turn = True  # start by turning to first waypoint

# Declare RealSense pipeline, encapsulating the actual device and sensors
pipe = rs.pipeline()
H_aeroRef_T265Ref = np.array([[0,0,-1,0],[1,0,0,0],[0,-1,0,0],[0,0,0,1]])
H_T265body_aeroBody = np.linalg.inv(H_aeroRef_T265Ref)

# Build config object and request pose data
cfg = rs.config()
cfg.enable_stream(rs.stream.pose)

# Start streaming with requested config
pipe.start(cfg)


roboclaw_vid = 0x03EB   # VID of Roboclaw motor driver in hex
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

# open waypoint file

if testmode:
        waypoint_file = 'waypoints_test.csv'

with open(waypoint_file) as csv_file:  # change to whatever waypoint file you want
        csv_reader = csv.reader(csv_file, delimiter=',')
        line_count = 0
        for row in csv_reader:
                waypoint[line_count][0] = float(row[0]) # x data
                waypoint[line_count][1] = float(row[1]) # y data
                print("Waypoint #",line_count, "X,Y", waypoint[line_count][0], waypoint[line_count][1])
                line_count += 1
        print('Loaded', line_count, 'waypoints')
        waypoint_total = line_count

if datalog:
        with open(datalog_file, 'w') as csvfile:  # overwrite original file
                recordwriter = csv.writer(csvfile, delimiter=',',quotechar='|', quoting=csv.QUOTE_MINIMAL)
                recordwriter.writerow(["Time", "Marvelmind X", "Marvelmind Y", "Fake Marvel X", "Fake Marvel Y","Realsense X", "Realsense Y"])


# initiate local positioning
if use_marvelmind:
        marvelmind_vid = 1155   # VID of Marvelmind device
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
                position = hedge.position()
        else:
                print ("Marvelmind not found")

# functions


def get_position():
        global hedgehog_x
        global hedgehog_y
        position = hedge.position()
        hedgehog_x = position[1]
        hedgehog_y = position[2]

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
                if (heading - desired_angle) >= math.pi:
                        direction = 1  # clockwise
                else:
                        direction = -1  # counterclockwise
        else:
                if abs(heading - desired_angle) >= math.pi:
                        direction = -1  # counterclockwise
                else:
                        direction = 1  # clockwise
        direction = steering_dir * direction # reverse if needed for robot
        return direction

def get_heading(data):  # this is essentially magic ;-)
    H_T265Ref_T265body = tf.quaternion_matrix([data.rotation.w, data.rotation.x,data.rotation.y,data.rotation.z]) # in transformations, Quaternions w+ix+jy+kz are represented as [w, x, y, z]!
    # transform to aeronautic coordinates (body AND reference frame!)
    H_aeroRef_aeroBody = H_aeroRef_T265Ref.dot( H_T265Ref_T265body.dot( H_T265body_aeroBody ))
    rpy_rad = np.array( tf.euler_from_matrix(H_aeroRef_aeroBody, 'rxyz') )
    heading = rpy_rad[2]
    if use_marvelmind and (not testmode):
        heading = heading - angle  # Right now rotation is 0, but you can change it to pi if you need to reverse the Realsense coordinate system to match Marvelmind        
        if heading < -math.pi:
                heading += 2*math.pi
        elif heading > math.pi:
                heading -=2*math.pi
        print("Realsense heading", round(heading,2), "Correction", round(angle,2), "Corrected heading:", round(heading-angle,2))
    return heading

def drive(speed, angle):
	left_speed = int(cruise_speed*100 + math.atan(angle)*steering_gain)    # we use atan(angle) as a smoothing function
	right_speed = int(cruise_speed*100 - math.atan(angle)*steering_gain)
	rc.SpeedM1(address,left_speed)
	rc.SpeedM2(address,right_speed)
	frames = pipe.wait_for_frames()
	pose = frames.get_pose_frame()
	if pose:
		data = pose.get_pose_data()
		x = data.translation.x
		y = -1 * data.translation.z # don't ask me why, but in "VR space", y is z and it's reversed
 #               print("Going straight: Current X", round(x,2),"Y", round(y,2))

def position_snapshot():
        global real_x, real_y, marvel_x, marvel_y
        ##	frames = pipe.wait_for_frames()
        ##	pose = frames.get_pose_frame()
        ##	if pose:
        ##		data = pose.get_pose_data()
        x = data.translation.x
        y = -1.000 * data.translation.z # don't ask me why, but in "VR space", y is z and it's reversed
        #                print("Realsense X", round(x,2),"Y", round(y,2))
        real_x = x
        real_y = y
        get_position()
        #        print ("Marvelmind position X: ", round(hedgehog_x,2), "Y: ", round(hedgehog_y,2))
        marvel_x = hedgehog_x
        marvel_y = hedgehog_y


def calibrate_realsense(start, finish):
        marvelmind_a = start[2:4]
        marvelmind_b = finish[2:4]
        marvelmind_diff = marvelmind_b - marvelmind_a
        realsense_a = start[0:2]
        realsense_b = finish[0:2]
        realsense_diff = realsense_b - realsense_a
        scale = mag(marvelmind_diff) / mag(realsense_diff)
        offset = realsense_a - marvelmind_a
        angle = angle_between(realsense_diff, marvelmind_diff)
        def rs_to_mm(rs):
#                return rotation_matrix(angle) @ (rs - realsense_a) + marvelmind_a
                return scale * rotation_matrix(angle) @ (rs - realsense_a) + marvelmind_a        
        print ("Initial calibration angle:", angle)
        return rs_to_mm, angle

def mag(v): return np.sqrt(v.dot(v))
def unit(v): return v / mag(v)
def angle_between(v1, v2): return np.arctan2(v1[0], v1[1]) - np.arctan2(v2[0], v2[1])
def rotation_matrix(theta):
    c = np.cos(theta)
    s = np.sin(theta)
    return np.array([[c, -s], [s, c]])

def save_datalog():
        with open(datalog_file, 'a') as csvfile:  # append to file
                recordwriter = csv.writer(csvfile, delimiter=',',quotechar='|', quoting=csv.QUOTE_MINIMAL)
                
                if use_marvelmind:
                        recordwriter.writerow([str(time.time()),round(hedgehog_x,2), round(hedgehog_y,2), round(marvel[0],2), round(marvel[1],2),round(real_x,2), round(real_y,2)])
                else:                        
                        recordwriter.writerow([str(time.time()),round(x,2), round(y,2)])

# main loop

if use_marvelmind: # first, calibrate Realsense by traveling forward for one meter
        print("Pausing to let readings settle")
        marvel=np.array([0,0])
        time.sleep(5)  # pause 10 seconds to let marvelmind readings settle
        frames = pipe.wait_for_frames()
        pose = frames.get_pose_frame()
        if pose:
                data = pose.get_pose_data()
                position_snapshot() # get current positions
                start = np.array([real_x, real_y, marvel_x, marvel_y])
        speed = 2000
        distance = 3000
        rc.SpeedDistanceM1M2(address,speed-steering_nudge,distance*tickdistanceL,speed+steering_nudge,distance*tickdistanceR,1)  # go forward 1m at speed 2000
        buffers = (0,0,0)
        while(buffers[1]!=0x80 and buffers[2]!=0x80):   #Loop until distance command has completed
                buffers = rc.ReadBuffers(address)
                frames = pipe.wait_for_frames()
                pose = frames.get_pose_frame()
                if pose:
                        data = pose.get_pose_data()
                        x = data.translation.x
                        y = -1.000 * data.translation.z # don't ask me why, but in "VR space", y is z and it's reversed
                        if use_marvelmind:
                                get_position()
                        save_datalog()
                time.sleep(0.1)
        print("Pausing to let readings settle")
        time.sleep(5)  # pause 10 seconds to let marvelmind readings settle                
        position_snapshot() # get current positions
        finish = np.array([real_x, real_y, marvel_x, marvel_y])
        transform, angle = calibrate_realsense(start, finish)  # save affine transformation matrix elements
        start = np.array([real_x, real_y, marvel_x, marvel_y])  # make the new start the old finish

try:
	while True:
		if recordmode: # record waypoints
			temp = raw_input("Hit l, r, f, s or w to go left, right, forward, stop or save waypoint")
			if (temp == "l"):
				print("Going left")
				drive(cruise_speed, -30)
			if (temp == "r"):
				print("Going right")
				drive(cruise_speed, 20)
			if (temp == "f"):
				print("Going forward")
				drive(cruise_speed, 0)
			if (temp == "s"):
				print("Stopping")
				rc.ForwardM1(address,0)  # kill motors
				rc.ForwardM2(address,0)
			if (temp == "w"):
				print("Saving waypoint #", waypoint_num)
				frames = pipe.wait_for_frames()
				pose = frames.get_pose_frame()
				if pose:
					data = pose.get_pose_data()
					x = data.translation.x
					y = -1.000 * data.translation.z # don't ask me why, but in "VR space", y is z and it's reversed
				if (waypoint_num == 0):
					with open(record_file, 'w') as csvfile:  # overwrite original file
						recordwriter = csv.writer(csvfile, delimiter=',',quotechar='|', quoting=csv.QUOTE_MINIMAL)
						recordwriter.writerow([round(x,2), round(y,2)])

				else:
					with open(record_file, 'a') as csvfile:  # append to file
						recordwriter = csv.writer(csvfile, delimiter=',',quotechar='|', quoting=csv.QUOTE_MINIMAL)
						recordwriter.writerow([round(x,2), round(y,2)])
				waypoint_num = waypoint_num + 1


		if not recordmode:     # This is regular waypoint mode
                        if not turn:
                                frames = pipe.wait_for_frames()
                                pose = frames.get_pose_frame()
                                if pose:
                                        #            yaw = pose.QueryYaw()
                                        data = pose.get_pose_data()
                                        if use_marvelmind and (not testmode):
                                                position_snapshot()  # this will return real and marvel x's and y's
                                                real = np.array([real_x, real_y])
                                                marvel = transform(real)
                                                x = marvel[0]  # replaces realsense coordinate with fake marvelmind coordinates
                                                y = marvel[1]
                                                print("Waypoint Number", waypoint_num, "Marvel X:", round(marvel_x,2), "Y", "Real X:", round(real_x,2), "Y", round(real_y, 2),round(marvel_y,2), "Fake X", round(x,2),"Y", round(y,2)," Target X", round(waypoint[waypoint_num][0],2),"Y", round(waypoint[waypoint_num][1],2))
                                        else:
                                                x = data.translation.x
                                                y = -1.000 * data.translation.z # don't ask me why, but in "VR space", y is z and it's reversed
                                                print("Waypoint Number", waypoint_num, "Rover X", round(x,2),"Y", round(y,2)," Target X", round(waypoint[waypoint_num][0],2),"Y", round(waypoint[waypoint_num][1],2))
                                        heading = get_heading(data)
        #				time.sleep(0.1) # don't flood the print buffer
                                        #            print ("heading", round(heading,2))
                                        delta_x = waypoint[waypoint_num][0] - x  # calculate distance to target
                                        delta_y = waypoint[waypoint_num][1] - y
        #                                print ("Delta X: ", round(delta_x,2), "Y: ", round(delta_y,2))
                                        range = math.sqrt(delta_y**2 + delta_x**2)
                                        if last_calibration == 0:
                                                last_calibration = range # just set this the first time, when it's 0
                                        if (range < (last_calibration - 3)) and (range > 1):  # recalibrate every 3 m
                                                print("Pausing to recalibrate")
                                                rc.ForwardM1(address,0)  # kill motors
                                                rc.ForwardM2(address,0)
                                                time.sleep(3) # pause to let readings settle
                                                position_snapshot() # get current positions
                                                finish = np.array([real_x, real_y, marvel_x, marvel_y])
                                                transform, angle = calibrate_realsense(start, finish)  # save affine transformation matrix elements
                                                start = finish
                                                last_calibration = range
                                        desired_angle = (math.pi/2)-math.atan2(delta_y,delta_x)  
                                        if desired_angle > math.pi:
                                                desired_angle = -1 * (2*math.pi-desired_angle) # turning counterclockwise is negative
                                        turn_angle = abs(desired_angle-heading) # get difference between desired angle and current angle
                                        raw_turn_angle = turn_angle   # this is just for debugging
                                        if desired_angle < heading: # these next lines correct for all the edge cases and singularities
                                                turn_angle = -1*turn_angle
                                        if turn_angle > math.pi:
                                                turn_angle = -2*math.pi + turn_angle
                                        if turn_angle < -math.pi:
                                                turn_angle = 2*math.pi + turn_angle
                                        if use_marvelmind:
                                            get_position()
                                        print ("Waypoint angle ", round(desired_angle,2), "Current heading ", round(heading,2), "Turn angle ", round(turn_angle,2), "Range ", round(range,2))
                                        if (range > hit_radius) and (not new_waypoint):
                                                drive (cruise_speed,turn_angle*(180/math.pi)) # Steer towards waypoint, in degrees
                                                save_datalog()
                                        if (range <= hit_radius) and (not new_waypoint):
                                                print ("Hit waypoint")
                                                waypoint_num = waypoint_num + 1
                                                if waypoint_num > 3:
                                                    waypoint_num = 0   # start over from beginning of waypoints
                                                new_waypoint = True
                                                last_calibration = 0
                                                turn = True
                        if turn:
                                print('Starting rotation')
                                heading = 999  # initialize
                                position_snapshot()  # this will return real and marvel x's and y's
                                real = np.array([real_x, real_y])
                                marvel = transform(real)
                                x = marvel[0]  # replaces realsense coordinate with fake marvelmind coordinates
                                y = marvel[1]
                                delta_x = waypoint[waypoint_num][0] - x  # calculate distance to target
                                delta_y = waypoint[waypoint_num][1] - y
                                desired_angle = (math.pi/2)-math.atan2(delta_y,delta_x)  
                                if desired_angle > math.pi:
                                        desired_angle = -1 * (2*math.pi-desired_angle) # turning counterclockwise is negative

                                while (heading < desired_angle - 0.02) or (heading > desired_angle + 0.02): # rotate until you're within a degree (0.02 radians) of target
                                        frames = pipe.wait_for_frames()
                                        pose = frames.get_pose_frame()
                                        if pose:
                                                data = pose.get_pose_data()
                                                heading = get_heading(data)
                                                delta_angle = heading-desired_angle  # get the difference between the current and intended angle
                                                direction = dir(heading, desired_angle)
                                                print ("Current heading:", round(heading,3), "Desired angle:", round(desired_angle,3), "Direction: ", direction)
                                                rc.ResetEncoders(address)
                                        if (heading < desired_angle - 0.25) or (heading > desired_angle + 0.25):  # go fast if you're more than .25 radians away
                                                speed = 2500  # go fast
                                        else:
                                                speed = 1500   # go slow
                                        rc.SpeedDistanceM1(address,-1*direction*speed,1*tickdistanceL,1) # rotate a few degrees
                                        rc.SpeedDistanceM2(address,direction*speed,1*tickdistanceR,1)
                                        buffers = (0,0,0)
                                        while(buffers[1]!=0x80 and buffers[2]!=0x80):   #Loop until distance command has completed
                         #                   displayspeed()
                                                buffers = rc.ReadBuffers(address)
                                new_waypoint = False
                                rc.ResetEncoders(address)
                                turn = False




except KeyboardInterrupt:
	rc.ForwardM1(address,0)  # kill motors
	rc.ForwardM2(address,0)
	pipe.stop()
	if (datalog or recordmode):
            csvfile.close() # close the file you've been writing
	if use_marvelmind:
	    hedge.stop()  # stop and close serial port
