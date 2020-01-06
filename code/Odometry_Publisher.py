'''
Team ID:		eYRC-EB#201
Author List:		Aditya Gaddipati
Filename:		Odometry_Publisher.py
Theme:			Explorer Bot
'''


#!/usr/bin/env python
import rospy
import math
import tf
import tf_conversions
from std_msgs.msg import *
from geometry_msgs.msg import *
from nav_msgs.msg import *
from sensor_msgs.msg import JointState

#Global variables :
 
#variables for position calculation
dist_per_count = 0.000478		#distance per tick
wheel_axle_length = 0.22
odom = Odometry()
_previousLeftcount=0
_previousRightcount= 0
x =0
y=0
th=0
prev_th = 0.0
cur_th = 0.0
dth = 0.0
count = Vector3()
joint = JointState()

#initialize node
rospy.init_node('Odometry_Publisher')

# More global variables:
current_time = rospy.get_time()
last_time = rospy.get_time()

odom = Odometry()
odom.header.frame_id = "odom"
odom.child_frame_id = "base_link"

#variables for transform broadcastting
odom_trans = TransformStamped()		
br = tf.TransformBroadcaster()

#variables for receiving angle and offset
quat = [0,0,0,0]
offset = 0.0


'''
Function Name: process
Input: none
Logic: This function claculates the x and y position of base_link in odom frame. 
	And publishes the pose and twist information on odom topic.
Example Call: process()
'''
def process():
	global _previousLeftcount
	global _previousRightcount
	global x
	global y
	global th,last_time,current_time
	global itr,sum_vel,ave,dev,quat
	
	current_time = rospy.get_time()
	_currentLeftcount = count.y
	_currentRightcount = count.x
	
	#change in encoder ticks
	deltaLeft = _currentLeftcount - _previousLeftcount		
	deltaRight = _currentRightcount - _previousRightcount
	dt = current_time - last_time
		
	v_left = (deltaLeft*dist_per_count)/dt
	v_right = (deltaRight*dist_per_count)/dt
	
	Vx = (v_right + v_left)/2
	Vth = (v_right - v_left)/wheel_axle_length
	
	# x and y is with respect to the center of rotation
	x += Vx*math.cos(th)*dt				
	y += Vx*math.sin(th)*dt 
	
	#broadcaster for odom->base_link tf 
	br.sendTransform((x+(0.058*math.cos(th)),y+(0.058*math.sin(th)),0),(quat),rospy.Time.now(),"/base_link","/odom")
	
	odom.header.stamp = rospy.Time.now()

	#0.58*math.cos(th) and 0.058*math.sin(th) are added
	# to x and y to get the actual position of center of base_link  	
	odom.pose.pose.position.x = x + (0.058*math.cos(th))     
	odom.pose.pose.position.y = y + (0.058*math.sin(th))
	#filling the quaternion message	 
	odom.pose.pose.orientation.x = quat[0]			
	odom.pose.pose.orientation.y = quat[1]
	odom.pose.pose.orientation.z = quat[2]
	odom.pose.pose.orientation.w = quat[3]

	odom_pub.publish(odom)
	
	last_time = current_time
	_previousLeftcount = _currentLeftcount
	_previousRightcount = _currentRightcount
	

'''
Function Name: vel
Input: Vector3
Logic: This function is the callback for topic 'velocity'. It receives the current wheel velocity.
	And odom.twist message is calculated.
Example Call: vel(msg)
'''
def vel(msg):
	#linear velocity
	odom.twist.twist.linear.x = (msg.x+msg.y)/2	
	#angular velocity	
	odom.twist.twist.angular.z = (msg.y-msg.x)/wheel_axle_length	
	

'''
Function Name: jnt_st_pub
Logic: This function publishes the wheel joint state, which is used by robot_state_publisher 
	for publishing base_link->wheel tf.
Example Call: jnt_st_pub()
'''
def jnt_st_pub():
	joint.header.stamp = rospy.Time.now()
	joint.name = ['right_wheel_joint','left_wheel_joint','caster_front_joint']
	#encodder ticks are used to calculate how much the wheel joint has rotated
	joint.position = [count.x*(6.283185/420),count.y*(6.283185/420),-1.570796]
	pub.publish(joint)

'''
Function Name: enc_ticks
Input: Vector3
Logic: This function is the callback for topic 'encoder_ticks'.
	It receives the left and right wheel encoder ticks and sets the global variables.
Example Call: enc_ticks(msg)
'''
def enc_ticks(msg):
	count.x = msg.x
	count.y = msg.y
	#print count.x,count.y
	process()
	jnt_st_pub()

'''
Function Name: yaw
Input: Quaternion
Output:
Logic: This function is the callback for topic 'rotation'.
	It receives the current orientation of robot and computes the Quaternion to be published.
Example Call: yaw(msg_)
'''
def yaw(msg_):
	global offset,quat,th,error
	#Quaternion received from robot is converted into euler for correcting the offset
	euler = tf.transformations.euler_from_quaternion([msg_.x,msg_.y,msg_.z,msg_.w])
	#offset variable is used to store and subtract the initial offset value received from IMU
	if not (offset):
		offset = euler[2]

	#offset is subtracted from the angle received from IMU
	th = euler[2]-offset
	euler_ = (euler[0],euler[1],th)
	quat = tf.transformations.quaternion_from_euler(euler_[0],euler_[1],euler_[2])
	

#subscriber for encoder ticks
sub1 = rospy.Subscriber('encoder_ticks',Vector3,enc_ticks, queue_size=100)
#subscriber for encoder left and right wheel velocity
sub2 = rospy.Subscriber('velocity',Vector3,vel,queue_size=100)
#subscriber for angle from IMU
rot = rospy.Subscriber('rotation',Quaternion,yaw,queue_size=100)
#publisher for wheel joint_states
pub = rospy.Publisher('joint_states', JointState, queue_size=10)
#publisher for odom topic
odom_pub = rospy.Publisher('odom', Odometry, queue_size=10)

rospy.spin()	
