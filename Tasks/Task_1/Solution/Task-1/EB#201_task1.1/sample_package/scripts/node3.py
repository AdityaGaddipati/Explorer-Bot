#!/usr/bin/env python

#Team Id:eYRC-EB#201
#Author List: Aditya Gaddipati
#Filename: node1.py
#Theme: Explorer Bot
#Functions: callback1(msg),callback2(msg),callback3(msg)
#Global Variables: number3,result_inter,op

import roslib;
roslib.load_manifest('sample_package')

#import rospy and msg libraries
import rospy
from std_msgs.msg import *

rospy.init_node('Node3')        #Initialisation of NODE3

number3 = Int32()               #object of type Int32
result_inter = Int32()          #used to store the intermediate result received from node2


#Function Name: callback1
#Logic: The data present on "number3" topic is recieved using
#       the "msg" arrgument of this callback function
def callback1(msg):
	number3.data = msg.data

#Function Name: callback2
#Logic: callback function for the subscriber of the topic result_inter.
def callback2(msg):
	result_inter.data = msg.data

#Function Name: callback3
#Logic: callback function for the subscriber of the topic operator.
#	Based on the message variable operation is performed
#       on result_inter and number3. Result is published.
def callback3(msg):
	if msg.data=='add':
		result = float(result_inter.data) + number3.data
	elif msg.data=='sub':
		result = float(result_inter.data) - number3.data
	elif msg.data=='mul':
		result = float(result_inter.data) * number3.data
	elif msg.data=='div':
		result = float(result_inter.data)/number3.data
	pub.publish(result)
	
#Initialisation of Subscriber for the topic number3 #data-type integer
sub1 = rospy.Subscriber('number3',Int32,callback1)

#Initialisation of Subscriber for the topic result_inter   #data-type integer
sub2 = rospy.Subscriber('result_inter',Int32,callback2)

#Initiatlisation of Subscriber for the topic operator #data-type string
sub3 = rospy.Subscriber('operator',String,callback3)

#Initialisation of publisher for the topic result(final result)  #data-type float
pub = rospy.Publisher('result',Float32)

rospy.spin() #prevents node from exiting. 
