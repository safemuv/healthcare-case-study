#!/usr/bin/env python
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from math import radians, degrees
from actionlib_msgs.msg import *
from geometry_msgs.msg import Point 


class map_navigation():
	
	def choose(self):

		choice='q'
		
		rospy.loginfo("|-------------------------------|")
		rospy.loginfo("|'0': Home ")
		rospy.loginfo("|'1': Point 1 ")
		rospy.loginfo("|'2': Point 2 ")
		rospy.loginfo("|'q': Quit ")
		rospy.loginfo("|-------------------------------|")
		rospy.loginfo("|Destination?")
		choice = input()
		return choice

	def __init__(self): 


		# declare the coordinates of interest 
		self.xh =  -1.95
		self.yh = -0.44
		self.xp1 = 0.09
		self.yp1 = -2.24
		self.xp2 = 2.38
		self.yp2 = 0.36


		
		self.goalReached = False
		# initiliaze
        	rospy.init_node('map_navigation', anonymous=False)
		choice = self.choose()
		
		if (choice == 0):

			self.goalReached = self.moveToGoal(4.73,-2.30)
		
		elif (choice == 1):

			self.goalReached = self.moveToGoal(-4.09, -1.12)

		elif (choice == 2):

			self.goalReached = self.moveToGoal(-4.15,-3.44)
		elif (choice == 3):

			self.goalReached = self.moveToGoal(4.56,-1.20)


		if (choice!='q'):

			if (self.goalReached):
				rospy.loginfo("Destination reached")
				#rospy.spin()

				#rospy.spin()

			else:
				rospy.loginfo("Error")
		
		while choice != 'q':
			choice = self.choose()
			if (choice == 0):

				self.goalReached = self.moveToGoal(4.73, -2.30)
		
			elif (choice == 1):

				self.goalReached = self.moveToGoal(-4.09,-1.12)
			
			elif (choice == 2):
	
				self.goalReached = self.moveToGoal(4.15,-3.44)
			elif (choice == 3):

				self.goalReached = self.moveToGoal(4.56,-1.20)


			if (choice!='q'):

				if (self.goalReached):
					rospy.loginfo("Destination reached")
					

				else:
					rospy.loginfo("Error")
					


	def shutdown(self):
        # stop turtlebot
        	rospy.loginfo("Quit program")
        	rospy.sleep()

	def moveToGoal(self,xGoal,yGoal):

		#define a client for to send goal requests to the move_base server through a SimpleActionClient
		ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)

		#wait for the action server to come up
		while(not ac.wait_for_server(rospy.Duration.from_sec(5.0))):
			rospy.loginfo("Waiting for the move_base action server to come up")
		

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

		rospy.loginfo("Sending goal location ...")
		ac.send_goal(goal)

		ac.wait_for_result(rospy.Duration(120))

		if(ac.get_state() ==  GoalStatus.SUCCEEDED):
			rospy.loginfo("You have reached the destination")	
			return True
	
		else:
			rospy.loginfo("The robot failed to reach the destination")
			return False

if __name__ == '__main__':
    try:
	
	rospy.loginfo("You have reached the destination")
        map_navigation()
        rospy.spin()
    except rospy.ROSInterruptException:
	rospy.loginfo("map_navigation node terminated.")
