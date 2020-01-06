'''
Team ID:		eYRC-EB#201
Author List:		Aditya Gaddipati
Filename:		new_velocity.py
Theme:			Explorer Bot
'''

#!/usr/bin/env python
import rospy
import math
from std_msgs.msg import *
from geometry_msgs.msg import *

velocity = Vector3()

#initialize node
rospy.init_node('Velocity_cmd')


target_speed = 0		#this variable stores linear velocity received from move_base
target_turn = 0			#this variable stores angular velocity received from move_base
control_speed = 0		#this linear velocity is sent to robot
control_turn = 0		#this angular velocity is sent to robot

'''
Function Name: callback
Input: Twist message
Logic: This function is the callback for topic cmd_vel
Example Call: callback(msg)
'''
def callback(msg):
	global target_speed,target_turn
	target_speed = msg.linear.x
	target_turn = msg.angular.z
	#if move_base gives Vth>0.8 then it is reduced to 0.4 
	if(target_turn>0.8):
		target_turn=0.4
	
#subscriber for velocity commands from move_base on cmd_vel
sub = rospy.Subscriber('cmd_vel', Twist, callback)
#publisher for veloctiy sent to robot 
pub = rospy.Publisher('motor_vel', Vector3, queue_size=10)

rate = rospy.Rate(20)
while not rospy.is_shutdown():
	#the following code ensures that robot achives target linear speed in steps of 0.02
	# and target angular speed in steps of 0.1
	if target_speed > control_speed:
                control_speed = min( target_speed, control_speed + 0.02 )
        elif target_speed < control_speed:
                control_speed = max( target_speed, control_speed - 0.02 )
        else:
        	control_speed = target_speed

    	if target_turn > control_turn:
        	control_turn = min( target_turn, control_turn + 0.1 )
    	elif target_turn < control_turn:
        	control_turn = max( target_turn, control_turn - 0.1 )
    	else:
        	control_turn = target_turn

	#individual motor velocity is calculated and sent to robot
	velocity.y = control_speed + (control_turn/8)
	velocity.x = control_speed - (control_turn/8)
	pub.publish(velocity)
	rate.sleep()

