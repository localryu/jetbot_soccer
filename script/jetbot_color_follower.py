#!/usr/bin/env python
import rospy
import time

from Adafruit_MotorHAT import Adafruit_MotorHAT
from std_msgs.msg import String
from vision_msgs.msg import BoundingBox2D

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

def detect_callback(msg):
        bbox_x = msg.center.x
        bbox_y = msg.center.y
        bbox_height = msg.size_y
        rospy.loginfo("x:%d, y:%d, height:%d", bbox_x, bbox_y, bbox_height)
        speed = (bbox_x - 640)/1000
        if bbox_x > 640/2 + 90:
		set_speed(motor_left_ID,  -0.55)
		set_speed(motor_right_ID,  0.55) 
        elif bbox_x < 640/2 - 90:
		set_speed(motor_left_ID,  0.55)
		set_speed(motor_right_ID,  -0.55) 
        elif bbox_height < 100:
		set_speed(motor_left_ID,  0.5)
		set_speed(motor_right_ID, 0.5) 
        else:
		set_speed(motor_left_ID,  0.0)
		set_speed(motor_right_ID, 0.0) 

# initialization
if __name__ == '__main__':

	# setup motor controller
        motor_driver = Adafruit_MotorHAT(i2c_bus=1)

	motor_left_ID = 1
	motor_right_ID = 2

	motor_left = motor_driver.getMotor(motor_left_ID)
	motor_right = motor_driver.getMotor(motor_right_ID)

	# stop the motors as precaution
	all_stop()

	# setup ros node
	rospy.init_node('jetbot_color_follwer')
	rospy.Subscriber('/bounding_box', BoundingBox2D, detect_callback)

	# start running
	rospy.spin()

	# stop motors before exiting
	all_stop()

