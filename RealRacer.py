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
import sys
sys.path.append('../')

#from simple_pid import PID
#pid = PID(1, 0.1, 0.05, setpoint=1)    # setting the P, I and D terms
#pid.sample_time = 0.1  # update every 0.1 seconds

waypoint_file = 'waypoints_office.csv'
#waypoint_file = 'waypoints_wework.csv'
#waypoint_file = 'waypoints-wework.csv'
record_file = 'waypoints_recorded.csv'
datalog_file = 'datalog.csv'
waypoint_num = 0
waypoints = 0
waypoint=[[0 for j in range(2)] for i in range(1000)]  # dimension an array up to 1,000 waypoints
x = 0
y = 0
testmode = False
recordmode = False
datalog = True
hit_radius = 0.1
cruise_speed = 35
steering_nudge = 200 # speed compensation for left/right imbalance when going straight in calibration
old_x = 0
old_y = 0
last_calibration = 0
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
    return heading

def drive(speed, angle):
	left_speed = int(cruise_speed*100 + math.atan(angle)*steering_gain)    # we use atan(angle) as a smoothing function
	right_speed = int(cruise_speed*100 - math.atan(angle)*steering_gain)
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
                                        heading = get_heading(data)
        #				time.sleep(0.1) # don't flood the print buffer
                                        #            print ("heading", round(heading,2))
                                        delta_x = waypoint[waypoint_num][0] - x  # calculate distance to target
                                        delta_y = waypoint[waypoint_num][1] - y
        #                                print ("Delta X: ", round(delta_x,2), "Y: ", round(delta_y,2))
                                        range = math.sqrt(delta_y**2 + delta_x**2)
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




except KeyboardInterrupt:
	pipe.stop()
	if (datalog or recordmode):
            csvfile.close() # close the file you've been writing
