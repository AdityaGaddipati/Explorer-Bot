import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import cv2
import time
import math
import aruco

bridge = CvBridge()

rospy.init_node("IMAGE")

img = np.zeros((480,640,3),np.uint8)
img1 = np.zeros((480,640,3),np.uint8)

K = np.array([[808.2116846767416, 0.0, 327.4704978292182],
              [0.0, 812.225244108047, 286.5597702395094],
              [0.0, 0.0, 1.0]])

D = np.array([[-0.1560602083373723, 0.498662130195463, -0.0008601943524612483, -0.003936603089513113, 0]])

obj_pts = np.array([[0, 0, 0],
                    [0, 4, 0],
                    [4, 4, 0],
                    [4, 0, 0]],np.float32)

img_pts = np.zeros((4,2),np.float32)

def process_img():
	global img,img1
	min_x=480
	min_y=640
	max_x=0
	max_y=0

	gray = cv2.cvtColor(img1,cv2.COLOR_BGR2GRAY)	
	ret , img1 = cv2.threshold(gray,127,255,0)
	contours,hierarchy = cv2.findContours(img1,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

	for i in range(0,len(hierarchy[0])):
		if not (hierarchy[0][i][2]== -1 or hierarchy[0][i][3]== -1 or hierarchy[0][hierarchy[0][i][2]][2]!=-1):			 
			#cv2.drawContours(img,contours,i,(0,255,0),3)
			M = cv2.moments(contours[i])
			cx = int(M['m10']/M['m00'])
			cy = int(M['m01']/M['m00'])

			
			for temp in range(0,len(contours[i])):
				for pt in contours[i][temp]:		
					if(pt[0]>max_x):
						max_x = pt[0]
					elif(pt[0]<min_x):
						min_x = pt[0]
					if(pt[1]>max_y):
						max_y = pt[1]
					elif(pt[1]<min_y):
						min_y = pt[1]
			
			#area = cv2.contourArea(contours[i])
			#print area
			
			if not ((abs((2*cx)-(max_x+min_x))>7) and (abs((2*cy)-(max_y+min_y))>7)):
				cv2.drawContours(img,contours,i,(0,255,0),2)
				
				
				#print min_x,min_y,max_x,max_y,cx,cy
			
				cv2.circle(img,(cx,cy),5,(255,0,0),-1)
				cv2.circle(img,(min_x,min_y),5,(255,0,0),-1)
				cv2.circle(img,(max_x,max_y),5,(255,0,0),-1)
				cv2.circle(img,(min_x,max_y),5,(255,0,0),-1)
				cv2.circle(img,(max_x,min_y),5,(255,0,0),-1)
				
				cv2.line(img,(min_x,min_y),(min_x,max_y),(255,0,0),1)
				cv2.line(img,(min_x,min_y),(max_x,min_y),(255,0,0),1)
				cv2.line(img,(max_x,max_y),(max_x,min_y),(255,0,0),1)
				cv2.line(img,(max_x,max_y),(min_x,max_y),(255,0,0),1)
				
				img_pts[0] = [min_x, min_y]
				img_pts[1] = [min_x, max_y]
				img_pts[2] = [max_x, max_y]
				img_pts[3] = [max_x, min_y]

				#ellipse = cv2.fitEllipse(contours[i])
				#cv2.ellipse(img,ellipse,(0,0,255),2)

				#(x,y),radius = cv2.minEnclosingCircle(contours[i])
				#cv2.circle(img,((int(x),int(y))),int(radius),(0,0,255),2)
			
				rtval, rvec, tvec = cv2.solvePnP(obj_pts, img_pts, K, D)
				#print tvec
				
				rot_mat = cv2.Rodrigues(rvec)[0]
				print math.asin(rot_mat[0][2])*(180/math.pi)
				#print (rot_mat)
				#for a in range(min_x,max_x):
				#	for b in range(min_y,max_y): 
				#		print gray[a][b]	

	img1 = img
	

def image(msg):
	global img1,img
	img = bridge.imgmsg_to_cv2(msg, "bgr8")
	img1 = img
	process_img()
	
sub = rospy.Subscriber('/usb_cam/image_raw',Image,image)

while not rospy.is_shutdown():
	cv2.imshow('image',img1)
	cv2.waitKey(500)
	cv2.destroyAllWindows()	

#rospy.spin()	
#

