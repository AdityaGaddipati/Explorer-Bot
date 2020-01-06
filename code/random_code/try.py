import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import cv2
import time

bridge = CvBridge()

rospy.init_node("IMAGE")

img1 = np.zeros((480,640,3),np.uint8)
#img1[200:400,0:200]=255
#img1[0:200,200:400]=255

def image(msg):
	global img1
	img = bridge.imgmsg_to_cv2(msg, "bgr8")

	img1 = img
	gray = cv2.cvtColor(img1,cv2.COLOR_BGR2GRAY)	
	ret , img1 = cv2.threshold(gray,127,255,0)
	contours,hierarchy = cv2.findContours(img1,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
	cv2.drawContours(img,contours,-1,(0,255,0),3)
	#img1 = img
	#print len(contours)
	'''
	for i in range(0,len(hierarchy[0])):
		if not (hierarchy[0][i][2]== -1):
			cv2.drawContours(img,contours,i,(0,255,0),3)
	'''	
	img1 = img	
	#h,w,c = img.shape
	#img2 = img[:,:,:]
	#img2[:,:,0] = 0
	#img2[:,:,1] = 0
	#img2[:,:,2] = 0
	#print img
	
sub = rospy.Subscriber('/usb_cam/image_raw',Image,image)

while not rospy.is_shutdown():
	cv2.imshow('image',img1)
	cv2.waitKey(500)
	cv2.destroyAllWindows()	

#rospy.spin()	
