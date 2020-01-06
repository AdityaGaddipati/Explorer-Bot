#!/usr/bin/env python

#Team Id:eYRC-EB#201
#Author List: Aditya Gaddipati
#Filename: task_3_aruco.py
#Theme: Explorer Bot
#Functions: callback(msg) 

#import rospy and msg libraries
import rospy
from std_msgs.msg import *
from visualization_msgs.msg import *

database = [[200,'Sandy'],[201,'Clay'],[202,'Silty sand'],[203,'Rain Water'],[204,'Ice'],
	    [205,'Snow water'],[206,'Sedimentary'],[207,'Metamorphic'],[208,'Igneous'],[209,'Oxides'],
	    [210,'Carbonates'],[211,'Phosphates'],[212,'Sulphides'],[213,'Native Element'],[214,'Silicates']]

#Initialisation of node 'ArUco'
rospy.init_node('ArUco')

#Function Name: callback
#Logic:	The data present on "Estimated_marker" topic is recieved using the "msg" arrgument 
#	which is of type Marker() of visualization_msgs. If marker ID is present in database 
#       then the corresponding substance name is printed. If ID is not present then it prints a message.
def callback(msg):
	if( msg.id <= 214 and msg.id>= 200):
		for i in database:
			if(i[0] == msg.id):
				rospy.loginfo("Marker ID " + str(msg.id) + " : " + i[1])
	
	else:
		rospy.loginfo("Marker ID " + str(msg.id) + " : No record found of this marker")

#Subscriber for the topic contining ID #msg-type Marker() 
sub = rospy.Subscriber('Estimated_marker',Marker,callback)

rospy.spin()	

