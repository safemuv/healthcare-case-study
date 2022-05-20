#!/usr/bin/env python
import rospy
import actionlib
import os
import random
import math
from threading import Thread
import time
from numpy.random import choice
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from math import radians, degrees
from actionlib_msgs.msg import *
#from geometry_msgs.msg import Point
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import *
from py4j.java_gateway import JavaGateway
from py4j.java_gateway import JavaGateway, CallbackServerParameters
from rosgraph_msgs.msg import Clock

class Map_navigation():
	
	def __init__(self): 
		
		self.goalReached = False
        	rospy.init_node('map_navigation', anonymous=False)#name of the node
        	self.obj=rospy.Subscriber("amcl_pose",PoseWithCovarianceStamped,self.callbackPose)#subscription to findRobPosition
		rospy.Subscriber("cmd_vel",Twist,self.callbackCmdVel)#subscription to cmd_vel
		#self.obj=rospy.Subscriber("move_base/status",GoalStatusArray,self.callbackStatusGoal)
		self.salir=False
		self.gateway = JavaGateway()#define java gateway
		self.intermediary = self.gateway.entry_point# get the java class instance
		self.ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)#make the movebaseaction global
		
		
		self.robotName = rospy.get_namespace()#get namespace of the robot
		self.success=[]#array used to save the succeeded goals
		self.failure=[] #array to save the failed goals
		self.b=0#band that indicates if the interval of failure has been reached
		self.isFailureInjected=False
		#self.triggerClockMonitor()
		self.t=Thread(target=self.triggerClockMonitor)#thread that handles the trigger
		self.t.start()
		self.r=0

		while not self.salir:
			rospy.loginfo("Press 0 to start round")
			choice = input()
			if (choice == 0):
				
				self.getRoom()
				
			
	

	"""Methods for handeling goals"""
	def getGoalStatus(self):
		l = self.gateway.jvm.java.util.ArrayList()#java List definition
		l.add(self.robotName)
		l.add(self.ac.get_goal_status_text())
		#dataGoal.add(self.ac.get_state())
		print(self.ac.get_goal_status_text())
		
	
	"""End Methods for handeling goals"""
	
	def taskPerformerRed(self):
		#Task 1
		rospy.loginfo("Performing task t1")
		rospy.sleep(5.)
		rospy.loginfo("t1 done")	 	 
		#Task 2
		chosen=choice([3.,8.],p=[0.6,0.4])#randomly selects 3 or 8 depending on p
		rospy.loginfo(chosen)#the selected time(probabilistically)
		rospy.loginfo("Performing task t2")
		rospy.sleep(chosen)#waits the selected amount of seconds
		rospy.loginfo("t2 done")
	
	def taskPerformerGreen(self):
		#Task 1
		rospy.loginfo("Performing task t1")
		rospy.sleep(5.)
		rospy.loginfo("t1 done")	 	 
		#Task 2 required with some  probability
		chosen=choice([0,1],p=[0.5,0.5])#0 no required;1 required
		print(chosen)
		rospy.loginfo(chosen)
		if chosen==0:
			rospy.loginfo("corregir")
		elif chosen==1:
			rospy.loginfo("Performing task t2")
			rospy.sleep(5.)#waits fixed time seconds
			rospy.loginfo("t2 done")

	#receives vx and vy to obtain v^2=vx^2+vy^2
	def speedCalculator(self,vx,vy):
		#vx=0.115721046925
		#vy=7.36029760446e-05
		v=math.sqrt((vx**2)+(vy**2))
		return v
		
	

	#Random route selector in Python; I have the same in JAVA see method getRoom()	
	def routeSelector(self):
		pos=random.randint(2,34)#generates random between 2 and 33 each one represents position in coordinates
		rospy.loginfo(pos)#print the selected room
		self.moveToGoal(self.coordinates[pos]['x'],self.coordinates[pos]['y'])
	
	"""getRooms gets the room coordinates from Java"""
	def getRoom(self):
		coordinates = self.gateway.jvm.java.util.ArrayList()#Defining type "list"				
		coordinates=self.intermediary.getCoordinates(self.robotName)
		"""get coordinates from java, sends name of robot call coordinates is a string list of the form [['x,y'],['x2,y2']...]; 		therefore needs to be transformed into [[x,y],[x2,y2]] """
		rc=self.transformCoordinates(coordinates);#makes coordinates ready to be used as described above
		#len(rc)
		print(rc)
		self.sendGoals(rc)#sends the set goals to be executed	
	

	"""Function that sends goal by goal to moveBase(self.moveToGoal) and saves succeeded and failed goals, also stop sending goals when failure is injected"""	
	def sendGoals(self,rc):
		for i in range(0,len(rc)):
			if(self.isFailureInjected):
				last=i
				print("injectada la falla")
				for j in range(last,len(rc)):
					self.failure.append(str(rc[j][0])+","+str(rc[j][1]))
				break
			
			else:
				flag=self.moveToGoal(rc[i][0],rc[i][1])
				if flag==True:
					self.success.append(str(rc[i][0])+","+str(rc[i][1]))
				elif flag==False:
					self.failure.append(str(rc[i][0])+","+str(rc[i][1]))
		print(self.success)
		print(self.failure)
			
		

	def saveRemainingGoals(self,rc,last):
		for j in range(last,len(rc)):
			self.failure.append(str(rc[i][0])+","+str(rc[i][1]))
		print(self.success)
		print(self.failure)
			
	

	"""this function is handled by a thread otherwise the main thread is affected for the sleep below, the for loop is iterating through a list of time intervals, the firts time sends the interval to the function that gets info from the clock and sends start and end of the interval, the function sleeps during an amount of time (the end of the interval-current_time) this time is required so that callbackClock can verify the beginig and the end of the interval and during this time the failure is triggered. After that the next interval is sended. The process is repeated as many times as values in the array"""	
	def triggerClockMonitor(self):
		inicio=1860
		fin=inicio+5
		lista=[[inicio,fin],[inicio+60,fin+60]]
		for i in range(0,len(lista)):
			rospy.Subscriber("/clock",Clock,self.callbackClock,(lista[i][0],lista[i][1])) #subscription to clock 
			now = rospy.get_rostime()
			e=lista[i][1]-now.secs
			#print(e)
			print("esperando...")			
			rospy.sleep(e)
			"""after the sleep time has finished then the failure is over and you have to resume the navigation; if ypu do not create another thread to execute stopFailure the next failure interval is not scheduled"""
			threadStop=Thread(target=self.stopFailure)
			threadStop.start()
			
			"""print("los dos siguinetes son iguales")
			now = rospy.get_rostime()			
			print(now.secs)
			print(lista[i][1])"""

		
	"""This fucntion gets the info from clock in secs; activates and deactivates a flag when the interval is meet"""
	def callbackClock(self,data,args):
			if data.clock.secs==args[0]:
				#self.b=1
				threadFailure=Thread(target=self.triggerFailure)
				threadFailure.start()
				#rospy.sleep(1.)#the sleep is because this method is executed per millisecond!
			
			
			
		
	
	"""This function is executed when the interval of failure is reached, sets a flag so that getRooms identifies when to save the counter and stop sending goals to moveBase; finally cancel all goals and the robot stays still. NOTE goals have to be sent to the moveBase for injecting the failure otherwise will not have any effect"""	
	def triggerFailure(self):
			self.isFailureInjected=True
			self.ac.cancel_all_goals()
			

	def stopFailure(self):
		self.isFailureInjected=False
		tempFailure=self.failure #failure is copied into a temporal array
		self.failure=[] #failure is emptied
		print("aqui se reanudan los goals")
		"""goals are resumed"""	
		rc2=self.transformCoordinates(tempFailure);#makes coordinates ready to be used
		self.sendGoals(rc2)	
	
	
	""" transformCoordinates converts string list into float list and separates the coordinates e.g. [['x,y'],['x2,y2']...] into [[x,y],[x2,y2]]"""
	def transformCoordinates(self,coordinates):
		newCor=[]
		for i in range(0,len(coordinates)):
			intCoord=[float(x) for x in coordinates[i].split(",")]#converts the string list inot int list
			newCor.append(intCoord)		
		return newCor
	

	#vx and vy are obtained from topic /cmd_vel to obtain absolute speed
	def callbackCmdVel(self,array):
		#rospy.loginfo(array.linear.x)
		#rospy.loginfo(array.linear.y)
		#robotName = rospy.get_namespace()#get namespace of the robot
		arraySpeed = self.gateway.jvm.java.util.ArrayList()#java List definition
		arraySpeed.add(self.robotName)#nameOfRobot is added so that can be identified
		arraySpeed.add(self.speedCalculator(array.linear.x,array.linear.y))#send velocity components to speedCalculator to get the magnitude of total velocity; then add the v into array
		self.intermediary.receiveSpeed(arraySpeed)#invoques receiveCoordinates from the javaClass
		
	
	#Gets position of the robot and sends info to JAVA
	def callbackPose(self,array):
		#robotName = rospy.get_namespace()#get namespace of the robot
		l = self.gateway.jvm.java.util.ArrayList()#java List definition
		l.append(self.robotName)
		l.append(array.pose.pose.position.x)#either append or add are valid
		l.add(array.pose.pose.position.y)
		l.add(array.pose.pose.position.z)
		self.intermediary.receiveCoordinates(l)#invoques receiveCoordinates from the javaClass
		
	
	def moveToGoal(self,xGoal,yGoal):
		#define a client for to send goal requests to the move_base server through a SimpleActionClient
		#ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)
		#wait for the action server to come up
		while(not self.ac.wait_for_server(rospy.Duration.from_sec(5.0))):
			rospy.loginfo("Waiting for the move_base action server to come up")

		goal=self.definingGoal(xGoal,yGoal)
		rospy.loginfo("Sending goal location ...")
		self.ac.send_goal(goal)
		self.ac.wait_for_result(rospy.Duration(120))
		
		

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
		goal.target_pose.pose.orientation.z = 0.77
		goal.target_pose.pose.orientation.w = 0.63 #1.0
		
		return goal



if __name__ == '__main__':
    try:
	#rospy.loginfo("You have reached the destination")     
	Map_navigation()
        rospy.spin()
    except rospy.ROSInterruptException:
	pass
	#rospy.loginfo("map_navigation node terminated.")

