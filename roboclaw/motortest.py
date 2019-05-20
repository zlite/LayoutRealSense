#***Before using this example the motor/controller combination must be
#***tuned and the settings saved to the Roboclaw using IonMotion.
#***The Min and Max Positions must be at least 0 and 50000

import time
from roboclaw import Roboclaw

#Windows comport name
#rc = Roboclaw("COM3",115200)
#Linux comport name
rc = Roboclaw("/dev/ttyACM0",0x80)
waypoints = 0

def displayspeed():
	enc1 = rc.ReadEncM1(address)
	enc2 = rc.ReadEncM2(address)
	speed1 = rc.ReadSpeedM1(address)
	speed2 = rc.ReadSpeedM2(address)
	print ("Encoder1:", enc1[1], "Encoder2:", enc2[1])
	print ("Speed1:", speed1[1], "Speed2:", speed2[1])

rc.Open()
address = 0x80
version = rc.ReadVersion(address)
tickdistanceL = 10 #  number of left encoder ticks per mm traveled
tickdistanceR = 10 #  number of right encoder ticks per mm traveled
if version[0]==False:
	print "GETVERSION Failed"
else:
	print repr(version[1])

try:
	while(True):
		rc.ResetEncoders(address)
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
	        buffers = (0,0,0)
        	while(buffers[1]!=0x80 and buffers[2]!=0x80):   #Loop until distance command has completed
                	print ("Buffers: ", buffers[1]," ", buffers[2])
	                displayspeed()
        	        buffers = rc.ReadBuffers(address)
		print ("Next waypoint")
		if (waypoints < 1):
			waypoints = waypoints + 1
		else: 
			waypoints = 0
except KeyboardInterrupt:
	rc.ForwardM1(address,0)
	rc.ForwardM2(address,0)
  
