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


# these are the affine transformation variables
translation_x = 0
translation_y = 0
rotation = 0
scale_x = 0
scale_y = 0
real_x = 0
real_y = 0
marvel_x = 0
marvel_y = 0
translated = np.array([2.0,2.0]) # just dimension this for later use

tickdistanceL = 10 #  number of left encoder ticks per mm traveled
tickdistanceR = 10 #  number of right encoder ticks per mm traveled
waypoint_file = 'waypoints_test.csv'
#waypoint_file = 'waypoints_home.csv'
#waypoint_file = 'waypoints-wework.csv'
record_file = 'waypoints_recorded.csv'
datalog_file = 'datalog.csv'
waypoint_num = 0
waypoints = 0
waypoint=[[0 for j in range(2)] for i in range(1000)]  # dimension an array up to 1,000 waypoints
x = 0
y = 0
use_marvelmind = True
hedgehog_x = 0
hedgehog_y = 0
testmode = False
position_testmode = False
recordmode = False
datalog = True
hedgehog_id = 6
cruise_speed = 25
steering_nudge = 200 # speed compensation for left/right imbalance when going straight in calibration
old_x = 0
old_y = 0
if use_marvelmind:
	new_waypoint = True
else:
	new_waypoint = False        
steering_dir = -1  # +/- 1 for direction of robot motors
steering_gain = 250
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
                recordwriter.writerow(["Marvelmind X", "Marvelmind Y", "Fake Marvel X", "Fake Marvel Y","Realsense X", "Realsense Y"])


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

def rotate (desired_angle):
	print('Starting rotation')
	heading = 999  # initialize
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

def get_heading(data):  # this is essentially magic ;-)
    H_T265Ref_T265body = tf.quaternion_matrix([data.rotation.w, data.rotation.x,data.rotation.y,data.rotation.z]) # in transformations, Quaternions w+ix+jy+kz are represented as [w, x, y, z]!
    # transform to aeronautic coordinates (body AND reference frame!)
    H_aeroRef_aeroBody = H_aeroRef_T265Ref.dot( H_T265Ref_T265body.dot( H_T265body_aeroBody ))
    rpy_rad = np.array( tf.euler_from_matrix(H_aeroRef_aeroBody, 'rxyz') )
    heading = rpy_rad[2]
    if use_marvelmind:
	    heading = heading - rotation  # reflects the fact that the RealSense and Marvelmind coordinate systems have opposite directions
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
	frames = pipe.wait_for_frames()
	pose = frames.get_pose_frame()
	if pose:
		data = pose.get_pose_data()
		x = data.translation.x
		y = -1.000 * data.translation.z # don't ask me why, but in "VR space", y is z and it's reversed
#                print("Realsense X", round(x,2),"Y", round(y,2))
		real_x = x
		real_y = y
	get_position()
#        print ("Marvelmind position X: ", round(hedgehog_x,2), "Y: ", round(hedgehog_y,2))
	marvel_x = hedgehog_x
	marvel_y = hedgehog_y

def calibrate_realsense2(start, finish):
	global scale_x, scale_y, translation_x, translation_y, rotation
	real_x1 = start[0]  # just spelling it all out for clarity
	real_y1 = start[1]
	marvel_x1 = start[2]
	marvel_y1 = start[3]
	real_x2 = finish[0]
	real_y2 = finish[1]
	marvel_x2 = finish[2]
	marvel_y2 = finish[3]
	mdelta_x = marvel_x2 - marvel_x1
	mdelta_y = marvel_y2 - marvel_y1
	heading_marvel = math.atan2(mdelta_y,mdelta_x)  #  get marvelmind heading
	rdelta_x = real_x2 - real_x1
	rdelta_y = real_y2 - real_y1
	heading_real = math.atan2(rdelta_y,rdelta_x)  #  get realsense heading
	print("Marvelmind heading:", round(heading_marvel,2), "Realsense heading: ", round(heading_real,2))
	print("Marvelmind initial position:", round(marvel_x1,2), round(marvel_y1,2),"Realsense initial position ", round(real_x1,2), round(real_y1,2))
	vector_r = np.array([real_x2-real_x1,real_y2-real_y1]) # realsense travel vector
	vector_m = np.array([marvel_x2-marvel_x1, marvel_y2-marvel_y1]) # marvelmind travel vector
	length_r = math.sqrt((real_x2-real_x1)**2 + (real_y2-real_y1)**2)  # length of vector
	length_m = math.sqrt((marvel_x2-marvel_x1)**2 + (marvel_y2-marvel_y1)**2)
	translation_x = marvel_x1-real_x1  # difference between starting positions of the two sensors
	translation_y = marvel_y1-real_y1
	dot_product = np.dot(vector_r,vector_m)  # multiply the vector matrices
#       rotation = math.acos(dot_product/(length_r*length_m))    # formula is cos(angle) = (vector1*vector2)/(length1*length2), so we solve for angle
	rotation = heading_marvel-heading_real
# now rotate and translate and find the scaling facotrs for x and y
	real_x1 = real_x1 + translation_x
	real_y1 = real_y1 + translation_y
	real_x2 = real_x2 + translation_x
	real_y2 = real_y2 + translation_y
	real_x1 = real_x1*math.cos(rotation) - real_y1*math.sin(rotation)
	real_y1 = real_x1*math.sin(rotation) + real_y1*math.cos(rotation)
	real_x2 = real_x2*math.cos(rotation) - real_y2*math.sin(rotation)
	real_y2 = real_x2*math.sin(rotation) + real_y2*math.cos(rotation)
	rdelta_x = real_x2 - real_x1
	rdelta_y = real_y2 - real_y1
	scale_x = mdelta_x/rdelta_x  # find scaling factors for each axis
	scale_y = mdelta_y/rdelta_y
	print("Scale X:", round(scale_x,3), "Y:", round(scale_y,3),"Translation X ", round(translation_x,2), "Y", round(translation_y,2), "Rotation", round(rotation,2))

def calibrate_realsense(start, finish):
	marvelmind_a = start[2:4]
	marvelmind_b = finish[2:4]
	marvelmind_diff = marvelmind_b - marvelmind_a
	realsense_a = start[0:2]
	realsense_b = finish[0:2]
	realsense_diff = realsense_b - realsense_a
#        marvelmind_dist = mag(marvelmind_diff)
#        realsense_dist = mag(realsense_diff)
	offset = realsense_a - marvelmind_a
	angle = angle_between(marvelmind_diff, realsense_diff)
	def rs_to_mm(rs):
	    return (rs - realsense_a)@rotation_matrix(angle) + marvelmind_a
	return rs_to_mm


def mag(v): return np.sqrt(v.dot(v))
def unit(v): return v / mag(v)
def angle_between(v1, v2): return np.arccos(np.clip(np.dot(unit(v1), unit(v2)), -1.0, 1.0))
def rotation_matrix(theta):
    c = np.cos(theta)
    s = np.sin(theta)
    return np.array([[c, -s], [s, c]])

def affine_transformation(original):
	# first do scaling
	temp = np.array([2.0,2.0])
#        print("Original X", original[0], "Y", original[1])
#        print("Scale X", scale_x)
	scale1 = original[0] * scale_x
	scale2 = original[1] * scale_y
	# then do translation
	trans1 = scale1 + translation_x
	trans2 = scale2 + translation_y
	# then do rotation around origin(the equation of rotation is x′=x*cos(θ)−y*sin(θ) and y′=x*sin(θ)+y*cos(θ))
	rotate1 = trans1*math.cos(rotation) - trans2*math.sin(rotation)
	rotate2 = trans1*math.sin(rotation) + trans2*math.cos(rotation)
	temp[0] = rotate1
	temp[1] = rotate2
#        print ("Scaled", round(scale1, 2), round(scale2,2), "Translated", round(trans1,2), round(trans2,2), "Rotated:", round(rotate1,2), round(rotate2,2))       
	return temp

def save_datalog():
        with open(datalog_file, 'a') as csvfile:  # append to file
                recordwriter = csv.writer(csvfile, delimiter=',',quotechar='|', quoting=csv.QUOTE_MINIMAL)
                if use_marvelmind:
                        recordwriter.writerow([round(hedgehog_x,2), round(hedgehog_y,2), round(marvel[0],2), round(marvel[1],2),round(x,2), round(y,2)])
                else:                        
                        recordwriter.writerow([round(x,2), round(y,2)])


# main loop

if use_marvelmind: # first, calibrate Realsense by traveling forward for one meter
	print("Pausing to let readings settle")
	marvel=np.array([0,0])
	time.sleep(5)  # pause 10 seconds to let marvelmind readings settle
	position_snapshot() # get current positions
	start = np.array([real_x, real_y, marvel_x, marvel_y])
	speed = 2000
	distance = 1000
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
	transform = calibrate_realsense(start, finish)  # save affine transformation matrix elements




try:
	while True:
		if (recordmode): # record waypoints
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


		if testmode:   # in test mode just do a box
			while True:
				rc.ResetEncoders(address)
				buffers = (0,0,0)
#				displayspeed()
				time.sleep(2)
				if (waypoints == 0): # straight
					rc.SpeedDistanceM1(address,2000,1000*tickdistanceL,1)
					rc.SpeedDistanceM2(address,2000,1000*tickdistanceR,1)
				if (waypoints == 1): # turn
					rc.SpeedDistanceM1(address,-2000,200*tickdistanceL,1)
					rc.SpeedDistanceM2(address,2000,200*tickdistanceR,1)
				buffers = (0,0,0)
				while(buffers[1]!=0x80 and buffers[2]!=0x80):   #Loop until distance command has completed
#				    displayspeed()
                                        if position_testmode:  # this just prints relative positions while you move the rover around
                                                position_snapshot()  # this will return real and mavel x's and y's
                                                real = np.array([real_x, real_y])
                                                marvel = transform(real)
                                                save_datalog()
#                                                print("Fake Marvel", marvel)                                                  
                                        ##                        original = np.array([real_x,real_y])  # turn it into a vector
                                        ###                        print("Original", original)
                                        ##                        translated = affine_transformation(original)  # run all this through the affine transformation to change to Marvelmind coordindate frame
                                        ##                        real_x = translated[0] 
                                        ##                        real_y = translated[1]
                                                print("Marvelmind", round(marvel_x,2), round(marvel_y,2), "Fake Marvel", round(marvel[0],2), round(marvel[1],2), "Realsense", round(real_x,2), round(real_y,2))
                                                time.sleep(0.1) # slow down the stream
                                        buffers = rc.ReadBuffers(address)
				print ("Next waypoint")
				if (waypoints < 1):
					waypoints = waypoints + 1
				else:
					waypoints = 0



		if ((not testmode) and (not recordmode) and (not position_testmode)):     # This is regular waypoint mode
			frames = pipe.wait_for_frames()
			pose = frames.get_pose_frame()
			if pose:
				#            yaw = pose.QueryYaw()
				data = pose.get_pose_data()
				x = data.translation.x
				y = -1.000 * data.translation.z # don't ask me why, but in "VR space", y is z and it's reversed
				if use_marvelmind:
					original = np.array([x,y])  # turn it into a vector
					translated = np.array([x,y]) # just dimension this for later use
					translated = affine_transformation(original)  # run alll this through the affine transformation to change to Marvelmind coordindate frame
 #                                       print("Affine transformation. Original:", original, "Translated:", translated)
##					x = translated[0] 
##					y = translated[1]
					position_snapshot()  # this will return real and mavel x's and y's
					real = np.array([real_x, real_y])
					marvel = transform(real)
				print("Waypoint Number", waypoint_num, "Rover X", round(x,2),"Y", round(y,2)," Target X", round(waypoint[waypoint_num][0],2),"Y", round(waypoint[waypoint_num][1],2))
				heading = get_heading(data)
				time.sleep(0.1) # don't flood the print buffer
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
				if use_marvelmind:
				    get_position()
				    print ("Marvelmind position X: ", round(hedgehog_x,2), "Y: ", round(hedgehog_y,2))
				print ("Waypoint angle ", round(desired_angle,2), "Current heading ", round(heading,2), "Turn angle ", round(turn_angle,2), "Range ", round(range,2))
				if new_waypoint:
					rotate(desired_angle)
					new_waypoint = False
					rc.ResetEncoders(address)
				if (range > 0.1) and (not new_waypoint):
					drive (cruise_speed,turn_angle*(180/math.pi)) # Steer towards waypoint, in degrees
					save_datalog()
				if (range <= 0.1) and (not new_waypoint):
					print ("Hit waypoint")
					waypoint_num = waypoint_num + 1
					if waypoint_num > 3:
					    waypoint_num = 0   # start over from beginning of waypoints
					new_waypoint = True


except KeyboardInterrupt:
	rc.ForwardM1(address,0)  # kill motors
	rc.ForwardM2(address,0)
	pipe.stop()
	if (datalog or recordmode):
            csvfile.close() # close the file you've been writing
	if use_marvelmind:
	    hedge.stop()  # stop and close serial port
