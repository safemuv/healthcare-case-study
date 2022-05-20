#!/usr/bin/env python
import rospy
import actionlib
import os
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from math import radians, degrees
from actionlib_msgs.msg import *
#from geometry_msgs.msg import Point
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import *
from py4j.java_gateway import JavaGateway
from py4j.java_gateway import JavaGateway, CallbackServerParameters


class map_navigation():
	
	def __init__(self): 

		# declare the coordinates of interest 
		self.xh  = -1.95
		self.yh  = -0.44
		self.xp1 = 0.09
		self.yp1 = -2.24
		self.xp2 = 2.38
		self.yp2 = 0.36
		self.goalReached = False
        	rospy.init_node('map_navigation', anonymous=False)#name of the node
        	#self.obj=rospy.Subscriber("amcl_pose",PoseWithCovarianceStamped,self.callbackPose)#subscription to findRobPosition
		#self.obj=rospy.Subscriber("move_base/status",GoalStatusArray,self.callbackStatusGoal)
		self.salir=False
		self.gateway = JavaGateway()#define java gateway
		#self.intermediary = self.gateway.entry_point# get the java class instance
		self.ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)#make the movebaseaction global
		gateway = JavaGateway(callback_server_parameters=CallbackServerParameters(),python_server_entry_point=self)#starts the gateway that sends the python class so that java can implement its behaviour
		 
		
		while not self.salir:
			rospy.loginfo("0 --> Home")
			rospy.loginfo("1 --> Point 1")
			rospy.loginfo("2 --> Point 2")
			rospy.loginfo("3 --> nonstop")
			rospy.loginfo("4 --> cancelGoal")
			rospy.loginfo("5 --> Quit ")
			rospy.loginfo("Destination?!!!!")
			choice = input()
			
			if (choice == 0):
				self.goalReached = self.moveToGoal(self.xp1, self.yp1)
		
			elif (choice == 1):

				self.goalReached = self.moveToGoal(self.xp1, self.yp1)
			
			elif (choice == 2):
	
				self.goalReached = self.moveToGoal(self.xp2, self.yp2)

			elif (choice == 3):

				self.goalReached = self.moveToGoal(self.xp1, self.yp1)
				rospy.sleep(10)
				self.goalReached = self.moveToGoal(self.xp2, self.yp2)
				rospy.sleep(10)
				self.goalReached = self.moveToGoal(self.xh, self.yh)
			
			elif(choice== 4):
				self.ac.cancel_goal()
			
			elif(choice== 5):
				self.salir=True
				self.shutdown()

			#os.system('clear')


			
	
			
	def goalStatus(self):
		dataGoal = self.gateway.jvm.java.util.ArrayList()#java List definition
		robotName = rospy.get_namespace()#get namespace of the robot
		dataGoal.add(robotName)
		dataGoal.add(self.ac.get_state())
		return dataGoal
	
	def cancelGoal(self):			
		self.ac.cancel_goal()

	#def javaCommunication(self,)		
	def shutdown(self):
        # stop turtlebot
        	rospy.loginfo("Quit program")
        	#rospy.on_shutdown(map_navigation)
		os.system("^C")#pending how to terminate the process

	
	
	def moveToGoal(self,xGoal,yGoal):
		#define a client for to send goal requests to the move_base server through a SimpleActionClient
		#ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)
		#wait for the action server to come up
		while(not self.ac.wait_for_server(rospy.Duration.from_sec(5.0))):
			rospy.loginfo("Waiting for the move_base action server to come up")

		goal=self.definingGoal(xGoal,yGoal)
		rospy.loginfo("Sending goal location ...")
		self.ac.send_goal(goal)
		self.ac.wait_for_result(rospy.Duration(60))
		
		

		if(self.ac.get_state() ==  GoalStatus.SUCCEEDED):
			rospy.loginfo("You have reached the destination")	
			return True
	
		else:
			rospy.loginfo("The robot failed to reach the destination")
			return False

	def definingGoal(self,xGoal,yGoal):
		goal = MoveBaseGoal()
		#set up the frame parameters
		goal.target_pose.header.frame_id = "map"
		goal.target_pose.header.stamp = rospy.Time.now()
		# moving towards the goal*/
		goal.target_pose.pose.position =  Point(xGoal,yGoal,0)
		goal.target_pose.pose.orientation.x = 0.0
		goal.target_pose.pose.orientation.y = 0.0
		goal.target_pose.pose.orientation.z = 0.0
		goal.target_pose.pose.orientation.w = 1.0
		
		return goal

	def sayHello(self):
        	rospy.loginfo("invocado desde java")
		


if __name__ == '__main__':
    try:
        mapNav=map_navigation()
	
	
        rospy.spin()
    except rospy.ROSInterruptException:
	rospy.loginfo("map_navigation node terminated.")

"""Example Java implements an abstract class of python then you can CALL PYTHON METHODS FROM JAVA """
