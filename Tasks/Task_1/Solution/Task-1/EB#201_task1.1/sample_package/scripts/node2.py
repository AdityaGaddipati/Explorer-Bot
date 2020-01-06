#!/usr/bin/env python

#Team Id:eYRC-EB#201
#Author List: Aditya Gaddipati
#Filename: node1.py
#Theme: Explorer Bot
#Functions: callback1(msg),callback2(msg)
#Global Variables: number1,number2 

import roslib;
roslib.load_manifest('sample_package')

#import rospy and msg libraries
import rospy
from std_msgs.msg import *

rospy.init_node('Node2')        #Initialisation of NODE2

number1 = Int32()               #objects of type Int32
number2 = Int32()

#Function Name: callback1
#Logic: The data present on "number1" topic is recieved using the "msg" arrgument of this callback function
def callback1(msg):
	number1.data = msg.data

#Function Name: callback2
#Logic: call back function for Subscriber of topic 'number2'
#	result_inter is calculated and published only after recieving both numbers.
def callback2(msg):
	number2.data = msg.data
	result_inter = number1.data + number2.data
	pub.publish(result_inter)

#Subscriber for the number1 topic #data-type Int32
sub1 = rospy.Subscriber('number1',Int32,callback1)

#Subscriber for the number2 topic #data-type Int32
sub2 = rospy.Subscriber('number2',Int32,callback2)

#publisher of the topic result_inter  #data-type integer
pub = rospy.Publisher('result_inter',Int32)

rospy.spin()    #prevents node from exiting
