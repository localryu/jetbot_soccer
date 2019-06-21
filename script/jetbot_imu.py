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
flag = 0


def ballCB(b_msg):
	global Bbox_x
	global Bbox_y
	global Bbox_h
	global flag
	flag = 1
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

	print('go2ball: %s go2goal: %s' %(go2ball,go2goal))
	#print('BALL : %d %d GOAL : %d %d' %(Bbox_x, Bbox_y, Gbox_x, Gbox_y))
	if go2ball == False:
		Degree = -1*math.atan((270 - Bbox_x)/(360 - Bbox_y))
		#print('Degree : %f' %(Degree))
		#Speed = ((360 - Bbox_y)/360);
		Speed = 1.0;
		str_cmd = str(Degree) + ',' + str(Speed)
		cmd_pub.publish(str_cmd)
		if Bbox_y > 300:
			#print('BALL DETECT')
			go2ball = True
			if goal_cnt > 1:
				finish = True
	else:
                #print("GOALPOST_H : %d" %(Gbox_h))
		if go2goal == False:
			Degree = -1*math.atan((270 - Bbox_x)/(360 - Bbox_y))
			Speed = 1.0;
			str_cmd = str(Degree) + ',' + str(Speed)
			cmd_pub.publish(str_cmd)
			if Gbox_h > 300:
				#print('GOAL DETECT')
				go2goal = True
		else:
			print('GOAL')
			goal_cnt = goal_cnt + 1
			str_cmd = str(0.0) + ',' + str(0.0)
			cmd_pub.publish(str_cmd)
			rospy.sleep(3.0)
			if goal_cnt > 0:
				finish = True
				str_fin = str(1)
				miss_pub.publish(str_fin)
			else : 
				go2ball = False
				go2goal = False



# initialization
if __name__ == '__main__':

	# setup ros node
	rospy.init_node('jetbot_imu')
	rospy.Subscriber('/ball/bounding_box', BoundingBox2D, ballCB)
	rospy.Subscriber('/goalpost/bounding_box', BoundingBox2D, goalpostCB)
	cmd_pub = rospy.Publisher('jet_cmd', String, queue_size=1)
	miss_pub = rospy.Publisher('missin_fin', String, queue_size=1)

	# setup motor controller
	motor_driver = Adafruit_MotorHAT(i2c_bus=1)

	motor_left_ID = 1
	motor_right_ID = 2

	motor_left = motor_driver.getMotor(motor_left_ID)
	motor_right = motor_driver.getMotor(motor_right_ID)

	# stop the motors as precaution
	str_cmd = str(0.0) + ',' + str(0.0)
	cmd_pub.publish(str_cmd)
	rospy.sleep(4.0)

	while 1:
		if flag == 1:
			play()
			print("finish : %s"% (finish))
			flag = 0
			if finish == True:
				break
	


