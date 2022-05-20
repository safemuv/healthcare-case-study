#!/usr/bin/env python
import rospy
import actionlib
import os
import random
from numpy.random import choice
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
		self.coordinates=[]
		self.home1={'x':4.73,'y':-2.30}
		self.coordinates.append(self.home1)

		self.home2={'x':4.74,'y':-2.68}  
		self.coordinates.append(self.home2)

		self.r1={'x':4.61,'y':-3.80}  
		self.coordinates.append(self.r1)

		self.r2={'x':3.51,'y':-3.73}  
		self.coordinates.append(self.r2)

		self.r3={'x':2.66,'y':-3.69}  
		self.coordinates.append(self.r3)

		self.r4={'x':1.80,'y':-3.68}  
		self.coordinates.append(self.r4)	

		self.r5={'x':0.92,'y':-3.81}  
		self.coordinates.append(self.r5)	
	
		self.r6={'x':0.11,'y':-3.66}  
		self.coordinates.append(self.r6)	

		self.r7={'x':-0.70,'y':-3.68}  
		self.coordinates.append(self.r7)	

		self.r8={'x':-1.55,'y':-3.67}  
		self.coordinates.append(self.r8)	

		self.r9={'x':-2.40,'y':-3.55}  
		self.coordinates.append(self.r9)	

		self.r10={'x':-3.27,'y':-3.54}  
		self.coordinates.append(self.r10)	

		self.r11={'x':-4.15,'y':-3.44}  
		self.coordinates.append(self.r11)	

		self.r12={'x':-4.98,'y':-3.52}  
		self.coordinates.append(self.r12)	

		self.r13={'x':-5.83,'y':-3.57}  
		self.coordinates.append(self.r13)	

		self.r14={'x':-6.77,'y':-3.52}  
		self.coordinates.append(self.r14)	

		self.r15={'x':-7.57,'y':-3.45}  
		self.coordinates.append(self.r15)	

		self.r16={'x':-8.50,'y':-3.47}  
		self.coordinates.append(self.r16)	

		self.r17={'x':4.56,'y':-1.20}  
		self.coordinates.append(self.r17)	

		self.r18={'x':3.52,'y':-1.21}  
		self.coordinates.append(self.r18)	

		self.r19={'x':2.66,'y':-1.26}  
		self.coordinates.append(self.r19)	

		self.r20={'x':1.76,'y':-1.24}  
		self.coordinates.append(self.r20)	

		self.r21={'x':0.88,'y':-1.26}  
		self.coordinates.append(self.r21)	

		self.r22={'x':0.019,'y':-1.20}  
		self.coordinates.append(self.r22)	

		self.r23={'x':-0.71,'y':-1.21}  
		self.coordinates.append(self.r23)	

		self.r24={'x':-1.55,'y':-1.23}  
		self.coordinates.append(self.r24)	

		self.r25={'x':-2.40,'y':-1.07}  
		self.coordinates.append(self.r25)	

		self.r26={'x':-3.28,'y':-1.044}  
		self.coordinates.append(self.r26)	

		self.r27={'x':-4.09,'y':-1.12}  
		self.coordinates.append(self.r27)	

		self.r28={'x':-4.99,'y':-1.08}  
		self.coordinates.append(self.r28)	

		self.r29={'x':-5.79,'y':-1.10}  
		self.coordinates.append(self.r29)	

		self.r30={'x':-6.65,'y':-1}  
		self.coordinates.append(self.r30)	

		self.r31={'x':-7.53,'y':-1.05}  
		self.coordinates.append(self.r31)	

		self.r32={'x':-8.38,'y':-1}  
		self.coordinates.append(self.r32)	
			

		
		self.goalReached = False
        	rospy.init_node('map_navigation', anonymous=False)#name of the node
        	#self.obj=rospy.Subscriber("amcl_pose",PoseWithCovarianceStamped,self.callbackPose)#subscription to findRobPosition
		#self.obj=rospy.Subscriber("move_base/status",GoalStatusArray,self.callbackStatusGoal)
		self.salir=False
		#self.gateway = JavaGateway()#define java gateway
		#self.intermediary = self.gateway.entry_point# get the java class instance
		self.ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)#make the movebaseaction global
		 
		
		while not self.salir:
			rospy.loginfo("0  --> Home R1")
			rospy.loginfo("1  --> Room 1")
			rospy.loginfo("2  --> Room 2")
			rospy.loginfo("3  --> Room 3")
			rospy.loginfo("4  --> Room 4")
			rospy.loginfo("5  --> Room 5")
			rospy.loginfo("6  --> Room 6")
			rospy.loginfo("7  --> Room 7")
			rospy.loginfo("8  --> Room 8")
			rospy.loginfo("9  --> Room 9")
			rospy.loginfo("10 --> Room 10")
			rospy.loginfo("11 --> Room 11")
			rospy.loginfo("12 --> Room 12")
			rospy.loginfo("13 --> Room 13")
			rospy.loginfo("14 --> Room 14")
			rospy.loginfo("15 --> Room 15")
			rospy.loginfo("16 --> Room 16")
			rospy.loginfo("Destination?!!!!")
			choice = input()
			
			if (choice == 0):
				#self.goalReached = self.moveToGoal(self.coordinates[0]['x'],self.coordinates[0]['y'])
				self.routeSelector()#choose a room
				#self.taskPerformerRed()#performs tasks in the room
				self.taskPerformerGreen()#performs tasks in the room
				self.routeSelector()#choose another room
			elif (choice == 1):
				self.goalReached = self.moveToGoal(self.coordinates[2]['x'],self.coordinates[2]['y'])
			elif (choice == 2):
				self.goalReached = self.moveToGoal(self.coordinates[3]['x'],self.coordinates[3]['y'])
			elif (choice == 3):
				self.goalReached = self.moveToGoal(self.coordinates[4]['x'],self.coordinates[4]['y'])
			elif (choice == 4):
				self.goalReached = self.moveToGoal(self.coordinates[5]['x'],self.coordinates[5]['y'])
			elif (choice == 5):
				self.goalReached = self.moveToGoal(self.coordinates[6]['x'],self.coordinates[6]['y'])
			elif (choice == 6):
				self.goalReached = self.moveToGoal(self.coordinates[7]['x'],self.coordinates[7]['y'])
			elif (choice == 7):
				self.goalReached = self.moveToGoal(self.coordinates[8]['x'],self.coordinates[8]['y'])
			elif (choice == 8):
				self.goalReached = self.moveToGoal(self.coordinates[9]['x'],self.coordinates[9]['y'])
			elif (choice == 9):
				self.goalReached = self.moveToGoal(self.coordinates[10]['x'],self.coordinates[10]['y'])
			elif (choice == 10):
				self.goalReached = self.moveToGoal(self.coordinates[11]['x'],self.coordinates[11]['y'])
			elif (choice == 11):
				self.goalReached = self.moveToGoal(self.coordinates[12]['x'],self.coordinates[12]['y'])
			elif (choice == 12):
				self.goalReached = self.moveToGoal(self.coordinates[13]['x'],self.coordinates[13]['y'])
			elif (choice == 13):
				self.goalReached = self.moveToGoal(self.coordinates[14]['x'],self.coordinates[14]['y'])
			elif (choice == 14):
				self.goalReached = self.moveToGoal(self.coordinates[15]['x'],self.coordinates[15]['y'])
			elif (choice == 15):
				self.goalReached = self.moveToGoal(self.coordinates[16]['x'],self.coordinates[16]['y'])
			elif (choice == 16):
				self.goalReached = self.moveToGoal(self.coordinates[17]['x'],self.coordinates[17]['y'])	
			elif(choice== 17 ):
				self.goalReached = self.moveToGoal(self.coordinates[18]['x'],self.coordinates[18]['y'])
				rospy.sleep(5)
				self.goalReached = self.moveToGoal(self.coordinates[20]['x'],self.coordinates[20]['y'])
				rospy.sleep(5)
				self.goalReached = self.moveToGoal(self.coordinates[2]['x'],self.coordinates[2]['y'])
			elif(choice== 18 ):
				self.goalReached = self.moveToGoal(self.coordinates[8]['x'],self.coordinates[8]['y'])
				rospy.sleep(5.)
				self.goalReached = self.moveToGoal(self.coordinates[18]['x'],self.coordinates[18]['y'])
				rospy.sleep(5.)
				self.goalReached = self.moveToGoal(self.coordinates[1]['x'],self.coordinates[1]['y'])
			
			elif(choice== 5):
				self.salir=True
				self.shutdown()


	
	def taskPerformerRed(self):
		#Task 1
		rospy.loginfo("Performing task t1")
		rospy.sleep(5.)
		rospy.loginfo("t1 done")	 	 
		#Task 2
		chosen=choice([3.,8.],p=[0.6,0.4])
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
			break
		elif chosen==1:
			rospy.loginfo("Performing task t2")
			rospy.sleep(5.)#waits fixed time seconds
			rospy.loginfo("t2 done")


	def routeSelector(self):
		pos=random.randint(2,34)#generates random between 2 and 33 each one represents position in coordinates
		rospy.loginfo(pos)#print the selected room
		self.moveToGoal(self.coordinates[pos]['x'],self.coordinates[pos]['y'])

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
	rospy.loginfo("You have reached the destination")
        map_navigation()
        rospy.spin()
    except rospy.ROSInterruptException:
	rospy.loginfo("map_navigation node terminated.")

