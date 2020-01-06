#!/usr/bin/env python
import rospy
import math
from std_msgs.msg import *
from geometry_msgs.msg import *

velocity = Vector3()

rospy.init_node('Velocity_cmd')

def callback(msg):
	velocity.y = msg.linear.x + (msg.angular.z/8)
	velocity.x = msg.linear.x - (msg.angular.z/8)
	pub.publish(velocity)

sub = rospy.Subscriber('cmd_vel', Twist, callback)
pub = rospy.Publisher('motor_vel', Vector3, queue_size=10)

rospy.spin()
