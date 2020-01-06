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

#This is the input SOI list. X facing corresponds to pi/2 and Y facing corresponds to 0. The first element will contain 
#the ID as and when detected.

#SOI = [[217,2,3,pi/2],[212,6,4,0],[209,5,2,pi/2],[205,4,6,pi/2],[220,1,1,0],[213,5,0,0],[200,1,5,0],[219,3,5,pi/2]]
#SOI = [[205,2,3,pi/2],[212,6,4,0],[208,5,2,pi/2],[203,4,6,pi/2],[220,1,1,0],[201,5,0,0],[199,1,5,0],[219,3,5,pi/2]]
SOI = [[0,2,3,pi/2],[0,6,4,0],[0,5,2,pi/2],[0,4,6,pi/2],[0,5,0,0],[0,1,5,0],[0,3,5,pi/2]]
#SOI = [[208,6,0,0],[199,2,1,0],[212,3,3,pi/2],[205,0,6,pi/2],[219,6,6,pi/2]]
#SOI = [[0,6,0,0],[0,2,1,0],[0,3,3,pi/2],[0,0,6,pi/2],[0,6,6,pi/2]]

aruco = []
goal_pts = []


# The code below will calculate goal point for each element in SOI list and store it in goal_pts list.
# The goal point is kept 1 feet away from aruco and facing it.
# For ArUco at difficult locations, goal point is shifted by +-2cm to avoid taking the robot into the inflated region.
# We told the robot to skip ArUco if it is present at (1,0), (0,1) or (1,1).
flag_=0
for obj in SOI:
	temp = []
	temp.append(obj[0])
	temp.append(0.3048*(0.5 + obj[1]))
	temp.append(0.3048*(0.5 + obj[2]))
	temp.append(obj[3])
	aruco.append(temp)

	if((obj[1]==1 and obj[2]==1) or (obj[1]==1 and obj[2]==0) or (obj[1]==0 and obj[2]==1)):
		continue	

	goal_pts.append(temp)
	flag_=0
	for mrkr in SOI:
		if (obj[3]==0):
			flag_ = 0
			if( (obj[1]-2 == mrkr[1] and obj[2]==mrkr[2]) or obj[1]-2==-1):
				flag_=1
				break
		elif(obj[3]==pi/2):	
			flag_ = 2
			if((obj[2]-2 == mrkr[2] or obj[2]-2==-1) and obj[1]==mrkr[1]):		
				flag_=3
				break

	if flag_==0:	
		goal_pts[-1][1] -= 0.3048
	elif flag_==1:
		goal_pts[-1][1] -= 0.3048
	elif flag_==2:
		goal_pts[-1][2] -= 0.3048
	elif flag_==3:
		goal_pts[-1][2] -= 0.3048
	
	if (obj[1]==0):
		goal_pts[-1][1] += 0.02 
	elif(obj[2]==0):
		goal_pts[-1][2] += 0.02
	elif(obj[1]==6 and obj[3]==pi/2):
		goal_pts[-1][1] -= 0.02
	elif(obj[2]==6 and obj[3]==0):
		goal_pts[-1][2] -= 0.02

	
#node initialization
rospy.init_node('Goal_Point')

# stuff related to goal_point() function	
move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
move_base.wait_for_server(rospy.Duration(60))
goal_pt = MoveBaseGoal()
goal_pt.target_pose.header.frame_id = '/map'
angle = 0.0

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

#
reach_time = 0.0
detect_time = 0.0
wait_time = 0.0

#
visible = 0

'''
Function Name: path_length
Input: coordinates of a point on map 
Output:
Logic: This will ask the move_base for a plan to a given point from the current location. That plan is stored as an array os points.
       The length of that path is claculated using distance between two ponints formula applied to the array of all points on the path.
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

#This function is just filling the goal_pt object.
def create_goal(x_=float(), y_=float(), th=float()):
	global goal_pt
	goal_pt.target_pose.header.frame_id = '/map' 
	goal_pt.target_pose.header.stamp = rospy.Time.now()
	goal_pt.target_pose.pose.position.x = x_
	goal_pt.target_pose.pose.position.y = y_
	quat = quaternion_from_euler(0, 0, th, axes='sxyz')
	goal_pt.target_pose.pose.orientation = Quaternion(*quat)




'''
Function Name: robot_pose
Input: Odomertry ROS message
Output:
Logic: This function is the callback for topic odom. The current pose of robot in map frame is updated.
Example Call: robot_pose(msg)
'''
def robot_pose(msg):
	global robot_x,robot_y,robot_yaw,euler

	odom_pose.pose = msg.pose.pose

	robot_yaw = [msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w]
	euler = tf.transformations.euler_from_quaternion(robot_yaw)
	
	try:
		map_pose = listener.transformPose("/map",odom_pose)
		robot_x = map_pose.pose.position.x 		
		robot_y = map_pose.pose.position.y	
				
	except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
		pass	

'''
Function Name:
Input:
Output:
Logic: This function is the callback for topic displayed. The flag is reset to indicate output has been displayed.
Example Call: reset(msg)
'''
def reset(msg):
	global flag,reach_time,detect_time,wait_time
	flag=0
	detect_time = rospy.get_time()
	wait_time = detect_time - reach_time

'''
Function Name:
Input:
Output:
Logic: This function is the callback for topic feedback. The 'visible' is set to indicate that ArUco is visible.
Example Call: reset(msg)
'''
def vision(msg):
	global visible
	visible = 1

sub = rospy.Subscriber('odom',Odometry,robot_pose)
sub_ = rospy.Subscriber('displayed',Empty,reset)
pub = rospy.Publisher('reached', Empty, queue_size=10)
sub_ = rospy.Subscriber('feedback',Empty,vision)

time.sleep(2)

# First a path is planned to each goal point in the goal_pts list. The point having shortest plan is sent as goal point to
# move_base. While the robot is moving, the camera is continuously sending feedback. The camera feedback is sent from the 
# map_odom_tf.py file. Feedback means (is the AruCo visible ?). Till nothing is visible, robot will try to reach the goal.
# But if it comes into vision then robot will cancel the goal and stop and send a signal to display the output. Th ouput is
# displayed by map_odom_tf.py file. It will send a signal that it has diaplayed the output. Then this node will remove the 
# goal point from the list and send a new goal.

# Basically here two nodes (this one and map_odom_tf) are working synchronously to achieve two things:
# 1. Getting the robot near the SOI till it is not visible.
# So getting the robot near the SOI by sending goals to move_base is this nodes function. Telling whether ArUco is visible or not
# is map_odom_tf node's dunction. 
# 2. Once visible the other node will communicate the same to this node and display the output wheen this node tells to do so.

while not rospy.is_shutdown():

	#This loop will find the nearest ArUco path length wise.
	min_len = 100.0
	for pt in goal_pts:		
		length = path_length(pt[1],pt[2])
		if(length<min_len):
			min_len = length
			dst = pt
		
	# Then send that goal
	create_goal(dst[1],dst[2],dst[3])
	move_base.send_goal(goal_pt)
	
	#The following loop ensures that robot stops only after ArUco is visible.
	#This is done using the feedback that the other node is sending.
	visible = 0
	while True:
		state = move_base.get_state()

		if(state == GoalStatus.SUCCEEDED):
			time.sleep(0.5)
			if(abs((euler[2]-dst[3])*(180/pi))>10):
				angle = euler[2] + (dst[3]-euler[2])/2	
				create_goal(robot_x,robot_y,angle)
				move_base.send_goal(goal_pt)
				move_base.wait_for_result()
				while not (visible):
					angle = euler[2] + 0.17
					create_goal(robot_x,robot_y,angle)
					if(state == GoalStatus.SUCCEEDED):
						move_base.send_goal(goal_pt)
					state = move_base.get_state()
		
				if not (state == GoalStatus.SUCCEEDED):
					move_base.cancel_goal()
			break

		if(visible):
			move_base.cancel_goal()
			break
	visible = 0			
	
	#Then it will tell the other node to display the output by sending an Empty ROS message and wait for acknowledgement.
	pub.publish()
	reach_time = rospy.get_time()	
	while(flag):
		pass	
	flag=1

	if(wait_time<3):
		goal_pts.remove(dst)

	if(len(goal_pts)==0):
		while(1):
			pass



