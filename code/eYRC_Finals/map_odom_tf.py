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

#This is the input SOI list. X facing corresponds to pi/2 and Y facing corresponds to 0. The first element will contain 
#the ID as and when detected.

#SOI = [[217,2,3,pi/2],[212,6,4,0],[209,5,2,pi/2],[205,4,6,pi/2],[220,1,1,0],[213,5,0,0],[200,1,5,0],[219,3,5,pi/2]]
#SOI = [[205,2,3,pi/2],[212,6,4,0],[208,5,2,pi/2],[203,4,6,pi/2],[220,1,1,0],[201,5,0,0],[199,1,5,0],[219,3,5,pi/2]]
SOI = [[0,2,3,pi/2],[0,6,4,0],[0,5,2,pi/2],[0,4,6,pi/2],[0,5,0,0],[0,1,5,0],[0,3,5,pi/2]]
#SOI = [[208,6,0,0],[199,2,1,0],[212,3,3,pi/2],[205,0,6,pi/2],[219,6,6,pi/2]]
#SOI = [[0,6,0,0],[0,2,1,0],[0,3,3,pi/2],[0,0,6,pi/2],[0,6,6,pi/2]]

#This list contains the SOI that are visited
soi_visited = [[0,0,0,0]] 

#This list will contain the actual coordinates of ArUco in the map.
aruco = []

#This list contains substance information corresponding to the ArUco ID
database = [[200,'Sandy'],[201,'Clay'],[202,'Silty sand'],[203,'Rain Water'],[204,'Ice'],
	    [205,'Snow water'],[206,'Sedimentary'],[207,'Metamorphic'],[208,'Igneous'],[209,'Oxides'],
	    [210,'Carbonates'],[211,'Phosphates'],[212,'Sulphides'],[213,'Native Element'],[214,'Silicates']]

#Here the actual coordinates of ArUco are calculated from the given coordinates in SOI list
#For exapmle (3,2) means centre of the cell at (3,2). 
#Therefore (3,2) = (3,2)*0.3048 + 0.1524
#The actual coordinates are stored in list aruco.
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

#node initialization
rospy.init_node('Map_Odom_tf')

#create a tf listner that will listen to camera->marker tf published by viewpoint_estimation node
listener = tf.TransformListener()

#These variables will contain translation and rotation of odom frame wrt map frame 
translation = [0.1324,0.1524,0]
rotation = [0,0,0,1]

#This tf broadcaster will send the map->odom tf 
br = tf.TransformBroadcaster()

#These variables will contain the current x,y and yaw of robot in odom frame
robot_x = 0.0
robot_y = 0.0
robot_yaw = 0.0
#These variables will contain the distance and angle of marker wrt camera
distance_z = 0.0	#Front distance from camera
distance_y = 0.0	#sideways distance from camera
marker_yaw = 0.0
#These variables will contain the x and y of robot calculated using ArUco 
marker_x = 0.0
marker_y = 0.0
#These variables are used while calculating x and y of robot wrt ArUco
a = 0.0
b = 0.0
front = 0.0
side = 0.0
#This variable is set to 1 whenever Empty signal is received on  topic 'reached'
#This topic is published by the node 'Goal_Point' from the file 'goal_pt.py'
flag=0
#This keeps track of how many ArUco have been visited
count=0
#These variables are used to calculate how much time after reaching the goal point the ArUco was detected
reach_time = 0.0
detect_time = 0.0
#These varriables are used to calculate the coordinates of ArUco detected since the ID is not given in SOI list
aruco_x = 0.0
aruco_y = 0.0
aruco_facing = 0.0
target = []
#This dicttionary will contain the visited ID
visited = {}
#This variable acts as a flag. Its use is described later.
localized = 0
#This will contain the ID if ArUco was detected even for a fraction of second.
Marker_ID = 0

'''
Function Name: camera2base
Input:	none
Output: none
Logic:	This function will convert the distances and angle obtained in the camera frame to base_link frame.
	Depending on distance of ArUco from camera, differnet correction values have to be applied to distance_z
	That is why we have not used camera->base_link tf for this conversion. 
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
	#print distance_z
	
'''
Function Name: localize
Input:	ID of the marker (integer)
Output:	none
Logic:	Here we calculate the coordinates of ArUco that is visible. Then we compare with the coordinates in SOI list.
	The closest match is concluded as the one which is currently vsible. All this is done because we are not given the ID 
	in SOI list. So it becomes difficult to get the actual coordinates of ArUco from SOI list because we don't know which 
	one it could be. This function will then calculate the x and y of robot in map frame using ArUco detected.
	This x and y will be subtracted from odom frame x and y. The resulting differnece will 
	be set as the odom->map frame translation. Thereby correcting the position of robot in Rviz.
Example Call: localize(200)
'''
def localize(ID=int()):
	global robot_x,robot_y,robot_yaw,distance_z,distance_y
	global marker_x,marker_y,marker_yaw,translation
	global a,b,front,side
	global aruco_x,aruco_y,aruco_facing,target,localized
	
	#There are four cases of where the robot is located wrt to ArUco.
	# i.e. (+-marker_angle , +-distance_y). This will result in 4 combinations. 
	# For each of these cases the formulas are little bit differnt.

	if(marker_yaw>0):
		#positive angle wrt to marker
		if(distance_y>0):
			#positive sideways distance wrt to marker
			a = abs(distance_y) + 0.033
			b = distance_z+(a/math.tan(abs(marker_yaw)))
			front = b*math.cos(abs(marker_yaw)) - a/math.sin(abs(marker_yaw))
		else:
			#negative sideways distance wrt to marker
			a = abs(distance_y) - 0.033
			b = distance_z-(a/math.tan(abs(marker_yaw)))
			front = b*math.cos(abs(marker_yaw)) + a/math.sin(abs(marker_yaw))
			#side = (distance_z*math.sin(abs(marker_yaw))) - (a*math.cos(abs(marker_yaw)))
	else:
		#negative angle wrt to marker
		if(distance_y>0):
			#positive sideways distance wrt to marker
			a = abs(distance_y) + 0.033
			b = distance_z-(a/math.tan(abs(marker_yaw)))
			front = b*math.cos(abs(marker_yaw)) + a/math.sin(abs(marker_yaw))
			#print a/math.tan(abs(marker_yaw))
		else:
			#negative sideways distance wrt to marker
			a = abs(distance_y) - 0.033
			b = distance_z+(a/math.tan(abs(marker_yaw)))
			front = b*math.cos(abs(marker_yaw)) - a/math.sin(abs(marker_yaw))
			#side = (distance_z*math.sin(abs(marker_yaw))) + (a*math.cos(abs(marker_yaw)))

	side = b*math.sin(abs(marker_yaw))
	
	# Below code will calculate the actual position of robot in map frame  
	
	aruco_facing = robot_yaw + marker_yaw
	if(abs(aruco_facing)<=(20*(pi/180))):
		aruco_facing = 0
	elif(abs(pi/2-abs(aruco_facing))<=(20*(pi/180))):
		aruco_facing = pi/2

	if(aruco_facing==0 or aruco_facing==pi/2):
		if(aruco_facing == 0):
			aruco_x = (translation[0]+robot_x) + front
			if (marker_yaw>0):
				aruco_y = (translation[1]+robot_y) - side
			else:
				aruco_y = (translation[1]+robot_y) + side
		elif(aruco_facing == pi/2):
			aruco_y = (translation[1]+robot_y) + front
			if (marker_yaw>0):
				aruco_x = (translation[0]+robot_x) + side
			else:
				aruco_x = (translation[0]+robot_x) - side
		closest_x = 100
		closest_y = 100
		target = []
		for soi in aruco:
			if(soi[3] == aruco_facing):
				if( abs(aruco_x - soi[1]) <= closest_x):
					closest_x = abs(aruco_x - soi[1])
					if(abs(aruco_y - soi[2]) <= closest_y):
						closest_y = abs(aruco_y - soi[2])
						target = soi
			else:
				continue
		
		#After finding which cell the detected ArUco belongs to, the ID will be added to that element in SOI list 
		SOI[aruco.index(target)][0] = ID

		# Robot x andd y is calculated here
		if(target[3]==0):

			marker_x = target[1] - front
			if(marker_yaw>0):
				marker_y = target[2] + side
			else:
				marker_y = target[2] - side
		
		else:
			marker_y = target[2] - front
			if(marker_yaw>0):
				marker_x = target[1] - side
			else:
				marker_x = target[1] + side
	
		#Here the differnce in RVIZ pose and actual pose is used to update the translation of map->odom tf
		translation[0] = marker_x - robot_x
		translation[1] = marker_y - robot_y
		
		# The variable is set to 1 to indicate that a new ID has been detected and successfully addded to the correct
		# element in the SOI list.
		if not (visited.has_key(ID)):
			localized = 1

'''
Function Name: display_output
Input:	ID of the marker (int)
Output:	none
Logic:	This function is responsible for printing the ouput string on the terminal. 
Example Call: display_output(200)
'''
def display_output(id_ = int()):
	global flag,count,detect_time,reach_time

	if(flag==1):
		detect_time = rospy.get_time()
		wait_time = detect_time - reach_time
		print wait_time
		if(wait_time<3):
			if( id_ <= 214 and id_>= 200):
				for i in database:
					if(i[0] == id_):
						substance = i[1]
						break
			else:
				substance = 'OBSTACLE'

			prev_site = (soi_visited[-1][1],soi_visited[-1][2])

			for obj in SOI:
				if(obj[0]==id_):
					serial_no = SOI.index(obj) + 1
					current_site = (obj[1],obj[2])
					soi_visited.append(obj)
					visited.update({id_:current_site})
					#break
			
					print str(serial_no)+':'+str(prev_site)+':'+str(current_site)+':'+str(id_)+':'+substance

					count+=1
					if(count==len(SOI)):
						print 'Mission Accomplished'		
					
					#The flag is reset after printing the output.		
					flag = 0
					#After printing it will send an Empty message to node 'Goal_Point' indicating that 
					#it can send the next goal point. 
					pub.publish()
					break
		else:
			for obj_ in soi_visited:
				if(id_==obj_[0]):					
					flag = 0
					pub.publish()
					break
		
'''
Function Name: callback
Input:	Marker ROS messsage
Output:	none
Logic:	This function is the callback for topic 'Estimated_marker'.
	It uses the ID to find the camera->marker tf. Using the tf it will calculate 
	distance and angle of marker in the camera frame.
Example Call: callback(msg)
'''
def callback(msg):
	global distance_z,distance_y,marker_yaw,localized,Marker_ID
	markerID = 'marker_'+str(msg.id)
	try:
		(trans,rot) = listener.lookupTransform( markerID, 'camera', rospy.Time(0))
		transform = t.concatenate_matrices(t.translation_matrix(trans), t.quaternion_matrix(rot))
		inv_tf = t.inverse_matrix(transform)

		distance_z = trans[2]
		distance_y = trans[1]
		marker_yaw = math.atan(inv_tf[0][2]/inv_tf[2][2])
		
		#convert readings to base_link
		camera2base()
		
		#If the detected marker is within 50 cm of the robot then it will do three things:
		# 1. Localize
		# 2. Tell the node goal_pt to cancel the goal because ArUco is visible from here.
		#    eyes.publish() sends an Empty ROS message to do this. 
		# 3. Finally it will display the output.
		# The step 2 is carried out only if the ArUco was not previously detected and the ID has been added to SOI list. 
		# This is done by the if statement checking the variable 'localized'. It is set to 1 in the localize() function
		# if ID has been successfully added to the SOI list.  
		if(distance_z<0.5):
			
			localize(msg.id)

			if (not visited.has_key(msg.id)) and localized:	
				eyes.publish()
				localized = 0
				Marker_ID = msg.id
				print Marker_ID

			display_output(msg.id)
					
		
	except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
		pass	

'''
Function Name: current_pose
Input: Odometry message
Output:	none
Logic:	This function is the callback for topic 'odom'.
	It will update odom frame position variables. 
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
Output:	none
Logic:	This function is the callback for topic 'reached'
	This is used like a signal to indicate that robot has reached the goal point.
	It is published by node 'Goal_Point'.
Example Call: disp_output(msg)
'''
def disp_output(msg):
	global flag,reach_time
	#flag is set to 1 to indicate robot has reached the goal point
	flag = 1
	reach_time = rospy.get_time()
	
	
#Subscribers and publishers for various topics	
sub = rospy.Subscriber('Estimated_marker',Marker,callback)
odom_sub = rospy.Subscriber('odom',Odometry,current_pose)
pub = rospy.Publisher('displayed',Empty,queue_size=10)
goal_sub = rospy.Subscriber('reached', Empty, disp_output)
eyes = rospy.Publisher('feedback',Empty,queue_size=10)

rate = rospy.Rate(20)

#this loop will publish map->odom tf at a rate of 20Hz
while not rospy.is_shutdown():
	br.sendTransform(translation,rotation,rospy.Time.now(),"odom","map")

	# This is just a precaution. The result is displayed if callback() function fails to do so.
	if flag==1 and (Marker_ID!=0):
		display_output(Marker_ID)
		Marker_ID = 0

	rate.sleep()


