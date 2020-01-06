
#!/usr/bin/env python


#import rospy and msg libraries
import rospy
from std_msgs.msg import *

operation_list=["add","sub","mul","div"]	#operation list

result = 0

#callback function for the result
def Print_result(temp):
        
	result=			#data from the subscribed messager should be stored in variable 'result'

	rospy.loginfo(display)


rospy.init_node('NODE1')		#Init node NODE1

#publisher of the topic number1 #data-type integer

#publisher of the topic number2 #data-type integer

#publisher of the topic number3  #data-type integer

#publisher of the topic operator #data-type string

#Subscriber for the result topic #data-type float

rate=rospy.Rate(1) #set rospy rate as 1msg/sec


while not rospy.is_shutdown(): #until rospy is not shutdown
		#publish num1 over the topic number1(pub_num1 is the object name here)
                #publish num2 over the topic number1(pub_num2 is the object name here)
		#publish num3 over the topic number1(pub_num3 is the object name here)
		# publish operation over the topic operator(pub_operator is the object name here)

		#initialize number1, number2, number3 to be 0,1 and 2 respectively and increment each number by 1 after each iteration
		

	rate.sleep()           #do nothing for specified delay in rate.



