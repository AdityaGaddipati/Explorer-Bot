import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import cv2
import time

bridge = CvBridge()

rospy.init_node("IMAGE")

img1 = np.zeros((480,640,3),np.uint8)

frames = []
frames.append(np.zeros((480,640,3),np.uint8))

last = time.time()
i = 0


def image(msg):
	global img1,last,i
	img = bridge.imgmsg_to_cv2(msg, "bgr8")

	img1 = img
	if(time.time()-last>5):
		frames.append(img)
		i+=1
		last = time.time()

sub = rospy.Subscriber('/usb_cam/image_raw',Image,image)

while not rospy.is_shutdown():
	cv2.imshow('image',frames[i])
	print i	
	cv2.waitKey(1000)
	cv2.destroyAllWindows()
	

	
