
#!/usr/bin/env python


#import rospy and msg libraries
import rospy
from std_msgs.msg import *

op=0


def operator_fn(msg):			# callback function for the subscriber of the topic operator
	global op			#based on the message variable op are given values of 0,1,2,3 for addition, subtraction, multiplication and division respectively
	if msg.data=="add":
		op=0
	elif msg.data=="sub":
		op=1
	elif msg.data=="mul":
		op=2
	elif msg.data=="div":
		op=3

def assign_inter_result(temp):	# callback function for the subscriber of the topic result_inter

	# intermediate result is stored in variable inter_result_int

def assign_num3(temp):		#callback function for the subscriber of the topic number3

	#number3 is stored in the variable num3

rospy.init_node("NODE3")	#Initialisation of NODE3

 #Initialisation of Subscriber for the topic result_inter   #data-type integer

#Initialisation of Subscriber for the topic number3 #data-type integer

#Initiatlisation of Subscriber for the topic operator #data-type string

 #Initialisation of publisher for the topic result(final result)  #data-type float

rate=rospy.Rate(1) #Set the rate for message reception

res=0
while not rospy.is_shutdown(): #iterate until rospy is not shutdonw
	if op==0:		#apply operations of addition, subtraction, multiplication and division between intermediate result and number3 depending upon op=0,1,2,3 respectively
		
		
	elif op==1:
		
		
	elif op==2:
		

	elif op==3:
		

	#pulishing the final result over the topic result (pub_res is the object name here)

	rate.sleep()		#do nothing for specified delay in rate.
	
