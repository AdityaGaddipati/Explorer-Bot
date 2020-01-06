'''
Team ID:		eYRC-EB#201
Author List:		Aditya Gaddipati
Filename:		map_odom_tf.py
Theme:			Explorer Bot
'''

#!/usr/bin/env python
import rospy
import tf
from tf import transformations as t
from std_msgs.msg import Empty
from std_msgs.msg import Float32
from geometry_msgs.msg import *
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
import math
from math import pi 
import time

SOI = [[208,6,0,0],[199,2,1,0],[212,3,3,pi/2],[205,0,6,pi/2],[219,6,6,pi/2]]
soi_visited = [[0,0,0,0]] 
aruco = []

database = [[200,'Sandy'],[201,'Clay'],[202,'Silty sand'],[203,'Rain Water'],[204,'Ice'],
	    [205,'Snow water'],[206,'Sedimentary'],[207,'Metamorphic'],[208,'Igneous'],[209,'Oxides'],
	    [210,'Carbonates'],[211,'Phosphates'],[212,'Sulphides'],[213,'Native Element'],[214,'Silicates']]

for obj in SOI:
	temp = []
	temp.append(obj[0])
	temp.append(0.3048*(0.5 + obj[1]))
	temp.append(0.3048*(0.5 + obj[2]))
	temp.append(obj[3])
	aruco.append(temp)
	
	if obj[3]==0:	
		aruco[-1][1] -= 0.05
	else:
		aruco[-1][2] -= 0.05

rospy.init_node('Map_Odom_tf')

listener = tf.TransformListener()

map_odom = Transform()
map_odom.translation.x = 0.1524 
map_odom.translation.y = 0.1524
br = tf.TransformBroadcaster()

translation = [0.1324,0.1524,0]
rotation = [0,0,0,1]

robot_x = 0.0
robot_y = 0.0
robot_yaw = 0.0
distance_z = 0.0
distance_y = 0.0
marker_yaw = 0.0
marker_x = 0.0
marker_y = 0.0
angle_error = 0.0
marker_angle = 0.0
a = 0.0
b = 0.0
front = 0.0
side = 0.0
flag=0

'''
Function Name: camera2base
Input: none
Output:
Logic:
Example Call: camera2base()
'''
def camera2base():
	global distance_z,distance_y,marker_yaw
		
	if (distance_z<0.5):
		distance_z -= 0.07+0.05
	elif (distance_z>=0.5 and distance_z<0.75):
		distance_z -= 0.07+0.065
	elif (distance_z>=0.75):
		distance_z -= 0.07+0.085
	
	marker_yaw -= 0.03
	
'''
Function Name: localize
Input: ID of the marker (integer)
Output:
Logic:
Example Call: localize(ID=int())
'''
def localize(ID=int()):
	global robot_x,robot_y,robot_yaw,distance_z,distance_y
	global marker_x,marker_y,marker_yaw,translation,angle_error,marker_angle
	global a,b,front,side
	
	if(marker_yaw>0):
		if(distance_y>0):
			a = abs(distance_y) + 0.033
			b = distance_z+(a/math.tan(abs(marker_yaw)))
			front = b*math.cos(abs(marker_yaw)) - a/math.sin(abs(marker_yaw))
		else:
			a = abs(distance_y) - 0.033
			b = distance_z-(a/math.tan(abs(marker_yaw)))
			front = b*math.cos(abs(marker_yaw)) + a/math.sin(abs(marker_yaw))
	else:
		if(distance_y>0):
			a = abs(distance_y) + 0.033
			b = distance_z-(a/math.tan(abs(marker_yaw)))
			b*math.cos(abs(marker_yaw)) + a/math.sin(abs(marker_yaw))
		else:
			a = abs(distance_y) - 0.033
			b = distance_z+(a/math.tan(abs(marker_yaw)))
			front = b*math.cos(abs(marker_yaw)) - a/math.sin(abs(marker_yaw))

	side = b*math.sin(abs(marker_yaw))	
	
	for obj in aruco:
		if(obj[0]==ID):
			if(obj[3]==0):

				marker_x = obj[1] - front
				if(marker_yaw>0):
					marker_y = obj[2] + side
				else:
					marker_y = obj[2] - side
				
			else:
				marker_y = obj[2] - front
				if(marker_yaw>0):
					marker_x = obj[1] - side
				else:
					marker_x = obj[1] + side
			
				
			marker_angle = obj[3]-marker_yaw
			angle_error = robot_yaw - marker_angle
			error.publish(angle_error)		
					
			break
	if(abs(marker_x - robot_x)<0.3 and abs(marker_y - robot_y)<0.3):
		translation[0] = marker_x - robot_x
		translation[1] = marker_y - robot_y
	

'''
Function Name: callback
Input: Marker ROS messsage
Output:
Logic: This function is the callback for topic
Example Call: callback(msg)
'''
count=0
def callback(msg):
	global distance_z,distance_y,marker_yaw,flag,count
	markerID = 'marker_'+str(msg.id)
	try:
		(trans,rot) = listener.lookupTransform( markerID, 'camera', rospy.Time(0))
		transform = t.concatenate_matrices(t.translation_matrix(trans), t.quaternion_matrix(rot))
		inv_tf = t.inverse_matrix(transform)

		distance_z = trans[2]
		distance_y = trans[1]
		marker_yaw = math.atan(inv_tf[0][2]/inv_tf[2][2])
		
		camera2base()
		if(distance_z<0.75):
			if(flag==1):
				if( msg.id <= 214 and msg.id>= 200):
					for i in database:
						if(i[0] == msg.id):
							substance = i[1]
							break
				else:
					substance = 'OBSTACLE'
				prev_site = (soi_visited[-1][1],soi_visited[-1][2])
				for obj in SOI:
					if(obj[0]==msg.id):
						serial_no = SOI.index(obj)
						current_site = (obj[1],obj[2])
						soi_visited.append(obj)
						break
				if(msg.id==219):
					msg.id = 189		
				print str(serial_no)+':'+str(prev_site)+':'+str(current_site)+':'+str(msg.id)+':'+substance
				count+=1
				if(count==5):
					print 'Mission Accomplished'		
				
				pub.publish()
				flag = 0
			localize(msg.id)

		
	except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
		pass	

'''
Function Name: current_pose
Input: Odometry message
Output:
Logic: This function is the callback for topic
Example Call: current_pose(msg)
'''
def current_pose(msg):
	global robot_x,robot_y,robot_yaw
	quat = [msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w]
	euler = tf.transformations.euler_from_quaternion(quat)
	robot_x = msg.pose.pose.position.x
	robot_y = msg.pose.pose.position.y
	robot_yaw = euler[2]
	
'''
Function Name: disp_output
Input: Empty ROS message
Output:
Logic: This function is the callback for topic
Example Call: disp_output(msg)
'''
def disp_output(msg):
	global flag
	flag = 1
	
sub = rospy.Subscriber('Estimated_marker',Marker,callback)
odom_sub = rospy.Subscriber('odom',Odometry,current_pose)
error = rospy.Publisher('Error',Float32,queue_size=10)
pub = rospy.Publisher('displayed',Empty,queue_size=10)
goal_sub = rospy.Subscriber('reached', Empty, disp_output)

rate = rospy.Rate(20)

while not rospy.is_shutdown():
	br.sendTransform(translation,rotation,rospy.Time.now(),"odom","map")
	rate.sleep()


