'''
Team ID:		eYRC-EB#201
Author List:		Aditya Gaddipati
Filename:		goal_point.py
Theme:			Explorer Bot
Functions:
Global Variables:
'''

#!/usr/bin/env python
import rospy
from std_msgs.msg import Empty
from geometry_msgs.msg import *
from nav_msgs.msg import Odometry
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.srv import GetPlan
import actionlib
from actionlib_msgs.msg import *
from tf.transformations import quaternion_from_euler
import math
from math import radians, pi
import tf
import time

SOI = [[208,6,0,0],[199,2,1,0],[212,3,3,pi/2],[205,0,6,pi/2],[219,6,6,pi/2]]
soi = SOI 
aruco = []
goal_pts = []
flag_=0
for obj in SOI:
	temp = []
	temp.append(obj[0])
	temp.append(0.3048*(0.5 + obj[1]))
	temp.append(0.3048*(0.5 + obj[2]))
	temp.append(obj[3])
	aruco.append(temp)
	goal_pts.append(temp)
	flag_=0
	for mrkr in SOI:
		if (obj[3]==0):
			flag_ = 0
			if((obj[1]-2 == mrkr[1] or obj[1]-2==-1) and obj[2]==mrkr[2]):
				#print obj[1],mrkr[1]
				flag_=1
				break
		elif(obj[3]==pi/2):	
			flag_ = 2
			if((obj[2]-2 == mrkr[2] or obj[2]-2==-1) and obj[1]==mrkr[1]):
				#print obj[2],mrkr				
				flag_=3
				break
	#print flag_
	if flag_==0:	
		goal_pts[-1][1] -= 0.4572
	elif flag_==1:
		goal_pts[-1][1] -= 0.3048
	elif flag_==2:
		goal_pts[-1][2] -= 0.4572
	elif flag_==3:
		goal_pts[-1][2] -= 0.3048

	if (obj[1]==0):
		goal_pts[-1][1] += 0.05 
	elif(obj[2]==0):
		goal_pts[-1][2] += 0.05
	elif(obj[1]==6 and obj[3]==pi/2):
		goal_pts[-1][1] -= 0.05
	elif(obj[2]==6 and obj[3]==0):
		goal_pts[-1][2] -= 0.05
 
print goal_pts

rospy.init_node('Goal_Point')

# stuff related to goal_point() function	
move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
move_base.wait_for_server(rospy.Duration(60))
goal_pt = MoveBaseGoal()
goal_pt.target_pose.header.frame_id = '/map' 

# stuff related to path_length() function
rospy.wait_for_service('/move_base/GlobalPlanner/make_plan')
client = rospy.ServiceProxy('/move_base/GlobalPlanner/make_plan', GetPlan)
start = PoseStamped()
start.header.frame_id = "/map"
goal = PoseStamped()
goal.header.frame_id = "/map"

# stuff related to robot_pose() function
listener = tf.TransformListener()
odom_pose = PoseStamped()
odom_pose.header.frame_id = "/odom"
robot_x = 0.0
robot_y = 0.0
robot_yaw = []
euler = []
flag=1	

'''
Function Name: path_length
Input: coordinates of a point on map 
Output:
Logic:
Example Call: path_length( 0.123,0.456 )
'''
def path_length( Goalx=float(),Goaly=float() ):
	global robot_x,robot_y
	start.pose.position.x = robot_x
	start.pose.position.y = robot_y

	goal.pose.position.x = Goalx
	goal.pose.position.y = Goaly
	
	path = client(start,goal,0.05)
	#print start
	length = 0
	x = path.plan.poses[0].pose.position.x
	y = path.plan.poses[0].pose.position.y

	for pt in path.plan.poses[1:]:
		length += math.sqrt(math.pow(x - pt.pose.position.x,2)+math.pow(y - pt.pose.position.y,2))
		x = pt.pose.position.x
		y = pt.pose.position.y
	
	return length

'''
Function Name: robot_pose
Input: Odomertry ROS message
Output:
Logic: This function is the callback for topic
Example Call: robot_pose(msg)
'''
def robot_pose(msg):
	global robot_x,robot_y,robot_yaw,euler
	odom_pose.pose = msg.pose.pose
	robot_yaw = [msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w]
	euler = tf.transformations.euler_from_quaternion(robot_yaw)
	#print euler[2]*(180/3.14)
	try:
		map_pose = listener.transformPose("/map",odom_pose)
		robot_x = map_pose.pose.position.x 		
		robot_y = map_pose.pose.position.y	
		#if(move_base.get_state()!=0 or move_base.get_state()!=1):	
		#goal_point()
		#print move_base.get_state()		
	except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
		pass	

'''
Function Name:
Input:
Output:
Logic: This function is the callback for topic
Example Call: reset(msg)
'''
def reset(msg):
	global flag
	flag=0

sub = rospy.Subscriber('odom',Odometry,robot_pose)
sub = rospy.Subscriber('displayed',Empty,reset)
pub = rospy.Publisher('reached', Empty, queue_size=10)

time.sleep(2)
'''
goal_pt.target_pose.header.stamp = rospy.Time.now() 
goal_pt.target_pose.pose.position.x = 0.3048
goal_pt.target_pose.pose.position.y = 0.3048
quat = quaternion_from_euler(0, 0, 0, axes='sxyz')	
goal_pt.target_pose.pose.orientation = Quaternion(*quat)
move_base.send_goal(goal_pt)
move_base.wait_for_result()
'''

while not rospy.is_shutdown():
	min_len = 100.0
	for pt in goal_pts:		
		length = path_length(pt[1],pt[2])
		if(length<min_len):
			min_len = length
			dst = pt
		#print length
	
	#time.sleep(10)
	'''
	a = robot_y-dst[2]
	b = robot_x-dst[1]

	if((abs(euler[2]))<(30*pi/180)):

		if(abs(a)<=0.3038):
			if(abs(b)>0.3048):
				if(b>0.3048):
					quat = quaternion_from_euler(0, 0, pi, axes='sxyz')	
					goal_pt.target_pose.pose.orientation = Quaternion(*quat)
					#face 180
		elif(a>0.3048):
			if(b<=0):
				quat = quaternion_from_euler(0, 0, 7*pi/4, axes='sxyz')	
				goal_pt.target_pose.pose.orientation = Quaternion(*quat)
				#face -45
			else:
				quat = quaternion_from_euler(0, 0, 5*pi/4, axes='sxyz')	
				goal_pt.target_pose.pose.orientation = Quaternion(*quat)
				#face -135
		else:
			if(b<=0):
				quat = quaternion_from_euler(0, 0, pi/4, axes='sxyz')	
				goal_pt.target_pose.pose.orientation = Quaternion(*quat)
				#face 45
			else:
				quat = quaternion_from_euler(0, 0, 3*pi/4, axes='sxyz')	
				goal_pt.target_pose.pose.orientation = Quaternion(*quat)
				#face 135
				
	elif(abs(euler[2])>(60*pi/180)):

		if(abs(b)<=0.3038):
			if(abs(a)>0.3048):
				if(a>0.3048):
					quat = quaternion_from_euler(0, 0, 3*pi/2, axes='sxyz')	
					goal_pt.target_pose.pose.orientation = Quaternion(*quat)
					#face -90
		elif(b>0.3048):
			if(a<=0):
				quat = quaternion_from_euler(0, 0, pi/4, axes='sxyz')	
				goal_pt.target_pose.pose.orientation = Quaternion(*quat)
				#face 45
			else:
				quat = quaternion_from_euler(0, 0, 7*pi/4, axes='sxyz')	
				goal_pt.target_pose.pose.orientation = Quaternion(*quat)
				#face -45
		else:
			if(a<=0):
				quat = quaternion_from_euler(0, 0, 3*pi/4, axes='sxyz')	
				goal_pt.target_pose.pose.orientation = Quaternion(*quat)
				#face 135
			else:
				quat = quaternion_from_euler(0, 0, 5*pi/4, axes='sxyz')	
				goal_pt.target_pose.pose.orientation = Quaternion(*quat)
				#face -135
	goal_pt.target_pose.pose.position.x = robot_x
	goal_pt.target_pose.pose.position.y = robot_y
	move_base.send_goal(goal_pt)
	move_base.wait_for_result()
	'''

	goal_pt.target_pose.header.stamp = rospy.Time.now() 
	goal_pt.target_pose.pose.position.x = dst[1]
	goal_pt.target_pose.pose.position.y = dst[2]
	quat = quaternion_from_euler(0, 0, dst[3], axes='sxyz')	
	goal_pt.target_pose.pose.orientation = Quaternion(*quat)
	move_base.send_goal(goal_pt)
	move_base.wait_for_result()
	
	state = move_base.get_state()
	
	#print state
	if state == GoalStatus.SUCCEEDED:
		time.sleep(0.5)
		if(abs((euler[2]-dst[3])*(180/pi))>10):
			goal_pt.target_pose.pose.position.x = robot_x
			goal_pt.target_pose.pose.position.y = robot_y
		
			angle = euler[2] + (dst[3]-euler[2])/2	
			quat = quaternion_from_euler(0, 0, angle, axes='sxyz')
			goal_pt.target_pose.pose.orientation = Quaternion(*quat)
			move_base.send_goal(goal_pt)
			move_base.wait_for_result()
		goal_pts.remove(dst)
	
	pub.publish()	
	while(flag):
		pass
	flag=1
	time.sleep(1)
	if(len(goal_pts)==0):
		print 'Mission Accomplished'
		while(1):
			pass

#rospy.spin()

