#!/usr/bin/env python

## Simple myo demo that listens to std_msgs/UInt8 poses published 
## to the 'myo_gest' topic and drives turtlesim

import rospy, math
from std_msgs.msg import UInt8, String
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist, Vector3
from ros_myo.msg import EmgArray

if __name__ == '__main__':
	global movingState
	global zero
	global y
	global K
	global speed
	speed = 0
	K = 1
	movingState = 0
	zero = 0
	y = 0
	rospy.init_node('turtlesim_driver', anonymous=True)

	# Publish to the turtlesim movement topic
	tsPub = rospy.Publisher("turtle1/cmd_vel", Twist, queue_size=1)
	turtlesimPub = rospy.Publisher("directs", String, queue_size=10)

	# Use the calibrated Myo gestures to drive the turtle
	def drive(gest):
		global movingState
		global zero
		global speed
		if gest.data == 1: #FIST
			movingState -= 1
		elif gest.data == 4 or gest.data == 2: #FINGERS_SPREAD
			movingState += 1
		elif gest.data == 3 :
			zero = y

		if movingState > 0 :
			movingState = 1
			turtlesimPub.publish("go forward")
			speed = 1
#			tsPub.publish(Twist(Vector3(1.0, 0, 0), Vector3(0, 0, 0)))
		elif movingState < 0 :
			movingState = -1
			turtlesimPub.publish("go back")
			speed = -1
#			tsPub.publish(Twist(Vector3(-1.0, 0, 0), Vector3(0, 0, 0)))
		else :
			speed = 0
		print (speed)

	def turn(imuRead):
		global zero
		global y
		global K
		global speed
		y = imuRead.orientation.y
		if (imuRead.orientation.y>zero):
			tsPub.publish(Twist(Vector3(speed,0,0),Vector3(0,0,K*(imuRead.orientation.y-zero))))
		if (imuRead.orientation.y<zero):
			tsPub.publish(Twist(Vector3(speed,0,0),Vector3(0,0,-K*(zero-imuRead.orientation.y))))
		#print (y)

	def strength(emgArr1):
		emgArr=emgArr1.data
		# Define proportional control constant:
		K = 0.005
		# Get the average muscle activation of the left, right, and all sides
		aveRight=(emgArr[0]+emgArr[1]+emgArr[2]+emgArr[3])/4
		aveLeft=(emgArr[4]+emgArr[5]+emgArr[6]+emgArr[7])/4
		ave=(aveLeft+aveRight)/2
		# If all muscles activated, drive forward exponentially
		if ave > 500:
			tsPub.publish(Twist(Vector3(0.1*math.exp(K*ave),0,0),Vector3(0,0,0)))
		# If only left muscles activated, rotate proportionally
		elif aveLeft > (aveRight + 200):
			tsPub.publish(Twist(Vector3(0,0,0),Vector3(0,0,K*ave)))
		# If only right muscles activated, rotate proportionally
		elif aveRight > (aveLeft + 200):
			tsPub.publish(Twist(Vector3(0,0,0),Vector3(0,0,-K*ave)))
		# If not very activated, don't move (high-pass filter)
		else:
			tsPub.publish(Twist(Vector3(0,0,0),Vector3(0,0,0)))

	rospy.Subscriber("myo_imu", Imu, turn)
	rospy.Subscriber("myo_gest", UInt8, drive)

	# spin() simply keeps python from exiting until this node is stopped
	rospy.spin()
