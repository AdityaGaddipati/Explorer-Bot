import rospy
from sensor_msgs.msg import Image
import numpy as np
import cv2
import cv
from cv_bridge import CvBridge, CvBridgeError
import time

img = cv2.imread('frame0000.jpg')
img1 = img
gray = cv2.cvtColor(img1,cv2.COLOR_BGR2GRAY)	
ret , img1 = cv2.threshold(gray,127,255,0)
contours,hierarchy = cv2.findContours(img1,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
#cv2.drawContours(img,contours,53,(0,255,0),3)

'''
print len(contours)
for i in range(0,len(contours)):
	if len(contours[i])==172:
		print i
	#time.sleep(1)
print contours[53]
'''
for i in range(0,len(hierarchy[0])):
	if not (hierarchy[0][i][2]== -1):
		cv2.drawContours(img,contours,i,(0,255,0),3)
	#print (hierarchy[0][53][2])

cv2.imshow('image',img)
cv2.waitKey(0)
cv2.destroyAllWindows()	
