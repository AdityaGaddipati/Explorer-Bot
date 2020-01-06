#!/usr/bin/env python

#import rospy and msg libraries
import rospy
from std_msgs.msg import *




def assign_num1(temp_num):       # callback function for Subscriber of topic 'number1'

	
def assign_num2(temp_num):	#call back function for Subscriber of topic 'number2'
	


rospy.init_node('NODE2')		#Initialisation of node2

 #publisher of the topic result_inter (intermediate result) which is gonna be used in node3 for result calculation  #data-type integer

rate=rospy.Rate(1)		# set rate of message reception

 	#Initialisation of a Subscriber for the topic number1 #data-type integer

	#Initialisation of a Subsciber for the topic number2  #data-type integer



while not rospy.is_shutdown():		#while rospy is not shutdown
	#calulate num1+num2 and store it in result_inter
	#result_inter is published over the topic result_inter

	rate.sleep()		#do nothing for specified delay in rate.

