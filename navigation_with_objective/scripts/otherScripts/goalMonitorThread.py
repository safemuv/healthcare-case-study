#!/usr/bin/env python
import rospy
import actionlib
import os
import random
import math
import threading
import time
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from py4j.java_gateway import JavaGateway
from py4j.java_gateway import JavaGateway, CallbackServerParameters

class ThreadMonitor(threading.Thread):

	def __init__(self):
		threading.Thread.__init__(self)
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
		if array.status_list:#if is not empty; because at the begining it is
			dataGoal.add(robotName)
			#dataGoal.add(self.ac.get_state())
			dataGoal.add(array.status_list[0].goal_id.id)#idGoal
			dataGoal.add(array.status_list[0].status)#statusGoal
			#self.intermediary.receiveInfoGoal(dataGoal)
			rospy.loginfo(array.status_list[0].goal_id.id)
			rospy.loginfo(array.status_list[0].status)

	def run(self):
		while(True):
			#print(self.ac.get_state())
			print(self.ac.get_goal_status_text())			
			time.sleep(3)
	
		

if __name__ == '__main__':
    try:
        h=ThreadMonitor()
	h.start()
        rospy.spin()
    except rospy.ROSInterruptException:
    	rospy.loginfo("Finished")
