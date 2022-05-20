#!/usr/bin/env python
import rospy
import actionlib
import os
import random
import math
import time
from actionlib_msgs.msg import *
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from py4j.java_gateway import JavaGateway
from py4j.java_gateway import JavaGateway, CallbackServerParameters

class ThreadMonitor():

	def __init__(self):
		
		rospy.init_node('goal_monitor', anonymous=False)#name of the node
		self.gateway = JavaGateway()#define java gateway
		self.intermediary = self.gateway.entry_point# get the java class instance
		self.ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)#make the movebaseaction global
		self.obj=rospy.Subscriber("move_base/status",GoalStatusArray,self.callbackStatusGoal)
		self.robotName = rospy.get_namespace()#get namespace of the robot
		#self.runThread()	
	
	def callbackStatusGoal(self,array):
		robotName = rospy.get_namespace()#get namespace of the robot
		dataGoal = self.gateway.jvm.java.util.ArrayList()#java List definition
		#print(array)
		if array.status_list:#if is not empty; because at the begining it is
			dataGoal.add(robotName)
			#dataGoal.add(self.ac.get_state())
			dataGoal.add(array.status_list[0].goal_id.id)#idGoal
			dataGoal.add(array.status_list[0].status)#statusGoal
			#self.intermediary.receiveInfoGoal(dataGoal)
			#rospy.loginfo(array.status_list[0].goal_id.id)
			rospy.loginfo(array.status_list[0].status)

	
		

if __name__ == '__main__':
    try:
        ThreadMonitor()
	
        rospy.spin()
    except rospy.ROSInterruptException:
    	rospy.loginfo("Finished")
