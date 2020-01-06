#!/usr/bin/env python


#import rospy and msg libraries
import rospy
import sys
import time
from std_msgs.msg import *

from keyboard_header import *

kb = KBHit()

rospy.init_node("Motor_Publisher")	#Initialisation of mot_pub

mot=rospy.Publisher('motor_control',UInt16, queue_size=10)

rate=rospy.Rate(100) #Set the rate for message reception


while not rospy.is_shutdown(): #iterate until rospy is not shutdonw
	inp_str=raw_input("enter input")
	inp=int(inp_str[-1])
	if(inp==7 or inp==9):
		while(True):
			mot.publish(UInt16(inp))
			display="data="+str(inp)+"\n"	
			rospy.loginfo(display)
			if kb.kbhit(): #If a key is pressed:
				break
			rate.sleep()
	else:
		mot.publish(UInt16(inp))
		display="data="+str(inp)+"\n"	
		rospy.loginfo(display)
			
	rate.sleep()
