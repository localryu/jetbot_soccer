#!/usr/bin/env python
import rospy
import time
import math

from Adafruit_MotorHAT import Adafruit_MotorHAT
from std_msgs.msg import String
from vision_msgs.msg import BoundingBox2D

Bbox_x = 640
Bbox_y = 360/2
Bbox_h = 0
Gbox_x = 640
Gbox_y = 360/2
Gbox_h = 0
go2ball = False
go2goal = False
finish = False
goal_cnt = 0

# sets motor speed between [-1.0, 1.0]
def set_speed(motor_ID, value):
	max_pwm = 115.0
	speed = int(min(max(abs(value * max_pwm), 0), max_pwm))

	if motor_ID == 1:
		motor = motor_left
	elif motor_ID == 2:
		motor = motor_right
	else:
		rospy.logerror('set_speed(%d, %f) -> invalid motor_ID=%d', motor_ID, value, motor_ID)
		return
	
	motor.setSpeed(speed)

	if value > 0:
		motor.run(Adafruit_MotorHAT.FORWARD)
	else:
		motor.run(Adafruit_MotorHAT.BACKWARD)


# stops all motors
def all_stop():
	motor_left.setSpeed(0)
	motor_right.setSpeed(0)

	motor_left.run(Adafruit_MotorHAT.RELEASE)
	motor_right.run(Adafruit_MotorHAT.RELEASE)

def motors(De, Sp):
	Degree = De/370 # positive : forward, negative : backward
	Speed = Sp # positive : right, negative : left 
	#print("Degree : %f  Speed : %f "% (Degree/370, Speed))
	Left_Gain = 0.79
	Right_Gain = 0.85
	
	if Degree <= 20 and Degree >= -20 :	
		set_speed(motor_left_ID,   Speed * -0.70)
		set_speed(motor_right_ID,   Speed * -1.0)
		#print("Goahead : %f, %f" % (Speed * -0.78, Speed * -1.0))
	elif Degree > 20 :
		set_speed(motor_left_ID,  (Degree/370)*-1.0)
		set_speed(motor_right_ID,  (Degree/370)*1.0) 
		#print("Left : %f, %f"%((Degree/370)*-1.0, (Degree/370)*1.0))

	else :
		set_speed(motor_left_ID,   (Degree/370)*1.0)
		set_speed(motor_right_ID, (Degree/370)*-1.0) 
		#print("Right : %f, %f" % ((Degree/370)*1.0, (Degree/370)*-1.0))

	


def ballCB(b_msg):
	global Bbox_x
	global Bbox_y
	global Bbox_h
	Bbox_h = b_msg.size_y
	if Bbox_h < 70:
		Bbox_x = b_msg.center.x
        	Bbox_y = b_msg.center.y
	else:
		Bbox_x = Bbox_x
		Bbox_y = Bbox_y


def goalpostCB(g_msg):
	global Gbox_x
	global Gbox_y
	global Gbox_h
	Gbox_x = g_msg.center.x
        Gbox_y = g_msg.center.y
	Gbox_h = g_msg.size_y

def play() :
	global Bbox_x
	global Bbox_y
	global Bbox_h
	global Gbox_x
	global Gbox_y
	global Gbox_h
	global go2ball
	global go2goal
	global finish
	global goal_cnt

	print('go2ball: %s go2goal : %s' %(go2ball, go2goal))
	#print('BALL : %d %d GOAL : %d %d' %(Bbox_x, Bbox_y, Gbox_x, Gbox_y))
	if go2ball == False:
		Degree = (Bbox_x - 270)
		#Speed = ((360 - Bbox_y)/360);
		Speed = 1;		
		motors(Degree, Speed)
		if Bbox_y > 300:
			#print('BALL DETECT')
			go2ball = True
	else:
                #print("GOALPOST_H : %d" %(Gbox_h))
		if go2goal == False:
			Degree = (Gbox_x - 270)
			Speed = 1;
			#Speed = ((360 - Bbox_y)/360);
			motors(Degree, Speed)
			if Gbox_h > 300:
				#print('GOAL DETECT')
				go2goal = True
		else:
			print('GOAL')
			goal_cnt = goal_cnt + 1
			motors(0, 0)
			rospy.sleep(3.0)
			if goal_cnt > 3:
				finish = True
			else : 
				go2ball = False
				go2goal = False



# initialization
if __name__ == '__main__':

	# setup ros node
	rospy.init_node('jetbot_soccer_')
	rospy.Subscriber('/ball/bounding_box', BoundingBox2D, ballCB)
	rospy.Subscriber('/goalpost/bounding_box', BoundingBox2D, goalpostCB)

	# setup motor controller
	motor_driver = Adafruit_MotorHAT(i2c_bus=1)

	motor_left_ID = 1
	motor_right_ID = 2

	motor_left = motor_driver.getMotor(motor_left_ID)
	motor_right = motor_driver.getMotor(motor_right_ID)

	# stop the motors as precaution
	all_stop()
	rospy.sleep(4.0)

	while 1:
		play()
		print("finish : %s"% (finish))
		if finish == True:
			break
	


