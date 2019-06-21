#!/usr/bin/env python
import cv2
import time
import os
from PIL import Image, ImageEnhance
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

i = 0
class CAM:

    def run(self):
        # Ros init
        rospy.init_node('filter', anonymous=True)
	r = rospy.Rate(5)
 	img = cv2.imread('/home/ryu/catkin_ws/src/jetbot_soccer/test1.jpg', cv2.IMREAD_COLOR)

        image_pub1 = rospy.Publisher("cam_pi",Image, queue_size=1)
        bridge = CvBridge()
	global i
	while i < 30:
		image_pub1.publish(bridge.cv2_to_imgmsg(img, "bgr8"))
		i+=1
		r.sleep()
        

################ MAIN ###################

cam = CAM()
cam.run()





