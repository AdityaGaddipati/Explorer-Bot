#!/usr/bin/env python

#Team Id:eYRC-EB#201
#Author List: Aditya Gaddipati
#Filename: node1.py
#Theme: Explorer Bot
#Functions: callback(msg)
#Global Variables: number1,number2,number3 

import roslib;
roslib.load_manifest('sample_package')

#import rospy and msg libraries
import rospy
from std_msgs.msg import *

number1 = 0
number2 = 1
number3 = 2
operator = ['add','sub','mul','div']

rospy.init_node('Node1')        #Initialisation of NODE1


#Function Name: callback
#Logic:  The data present on "result" topic is recieved using the "msg" arrgument of this callback function
#        Numbers are increamented only after the result is printed.
def callback(msg):
	global number1
	global number2
	global number3
	rospy.loginfo('('+str(number1)+'+'+str(number2)+') '+operator[number1%4]+' '+str(number3)+' = '+str(msg.data))
	number1 += 1
	number2 += 1
	number3 += 1


#publisher of the topic number1 #data-type integer
pub1 = rospy.Publisher('number1',Int32)

#publisher of the topic number2 #data-type integer
pub2 = rospy.Publisher('number2',Int32)

#publisher of the topic number3  #data-type integer
pub3 = rospy.Publisher('number3',Int32)

#publisher of the topic operator #data-type string
pub4 = rospy.Publisher('operator',String)

#Subscriber for the result topic #data-type float
sub = rospy.Subscriber('result',Float32,callback)

rate = rospy.Rate(4)
# 4 topics are published in 1sec with a delay of 0.25sec between each topic.

# On each topic 1 msg/sec is published	
while not rospy.is_shutdown():
	pub1.publish(number1)                   
	rate.sleep()
	pub2.publish(number2)
	rate.sleep()
	pub3.publish(number3)
	rate.sleep()
	pub4.publish(operator[number1%4])
	rate.sleep()
	
