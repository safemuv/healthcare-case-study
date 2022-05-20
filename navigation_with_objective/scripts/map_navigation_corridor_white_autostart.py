#!/usr/bin/python2.7
import rospy
import actionlib
import os
import random
import math
import copy
from threading import Thread
import time
import socket
from collections import deque
from numpy.random import choice
import numpy as np
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionResult
from math import radians, degrees
from actionlib_msgs.msg import *
#from geometry_msgs.msg import Pointlll
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import *
#from py4j.java_gateway import JavaGateway
#from py4j.java_gateway import JavaGateway, CallbackServerParameters
from rosgraph_msgs.msg import Clock

class Map_navigation():
    
    def __init__(self):

        rospy.init_node('map_navigation', anonymous=False)#name of the node
        self.salir=False
        #self.gateway = JavaGateway()#define java gateway
        #self.intermediary = self.gateway.entry_point# get the java class instance
        self.nominalPiRetry=0.0 #the nominal Value will never change, after an interval piRetry will take this value again
        self.piRetry=0 #the current value of piRetry (will be changing over time according to the intervals); controls the probability of retrying a task
        self.robotName = rospy.get_namespace()#get namespace of the robot
        self.success=deque([]) #deque used to save the succeeded index of room
        self.failure=deque([]) #deque to save the failed index of room
        self.isFailureInjected=False
        self.modeFailureInjection=0 #this flag activates and deactivates the mode failure injection (0--> off; 1-->on)
        self.currentRoom=""#will save the index(noOfRoom), coordinates and type of room that the robot wants to serve
        self.statusCurrentRoom=False #will save when the robot is done with its tasks in self.currentRoom
        self.furthestAllocatedRoom=[]#the coordinates of the furthest allocated room
        self.currentDistance=0#distance between current location and furthest room
        self.furthestToHome=0#distance between furthest allocated room and home
        self.totalDistance=0#self.currentDistance+self.furthestToHome
        self.homeCor=[]#home cordinate will be distinct for each robot
        self.allTheRooms=[]#containst the coordinates of all rooms
        self.batteryEmpty = False # is the battery dead
        self.mySetOfRooms=deque([])
        
        self.newRooms = False

        if(self.robotName=="/tb3_0/"):
            self.homeCor.append(3.46)
            self.homeCor.append(-3.60)
        elif(self.robotName=="/tb3_1/"):
            self.homeCor.append(3.45)
            self.homeCor.append(-3.24)
        elif(self.robotName=="/tb3_2/"):
            self.homeCor.append(-5.59)
            self.homeCor.append(-3.23)

        self.furthestAllocatedRoom.append(0)#initialise
        self.furthestAllocatedRoom.append(0)#initialise

        #l1,l2,l3 are the means(not sure) that will be used to calculate the time that the robot will spend doing t1-t3 respectively
        #ask if lambdas will be differenty per type of room may be for type 1 lambda has a value and for Type 2 lambda has a different value
        self.l1=0
        self.l2=0
        self.l3=0#this lambda is not longer used because for t3 we added l3Full and l3Basic

        self.p2Norm=0.8 #for room type 1 t2 has an extended and a simple version, p2Norm is the probability of executing the normal version, 1-p2Norm would be the extended
        #read p2orm from the file! now is hard-c
        self.l2Extended=10 #lambdaDosRoom1 extended
        self.l2Normal=5 #lambdaDosRoom1 normal


        self.l3Full=0
        self.l3Basic=0
        self.p2Req=0
        self.p3Poss=0
        self.p3Full=0
        #p2Retry is piRetry ask if you require one variable per type of room

        #self.parameters = self.gateway.jvm.java.util.ArrayList()#java List that will contain all the required by DECIDE
        self.parameters=[]
        self.parameters.append(self.robotName)#robotName
        self.parameters.append(0)#position
        self.parameters.append(0)#speed
        self.parameters.append(0)#isTrapped
        self.parameters.append(0)#piRetry
        self.parameters.append(0)#name and status of last room visited
        self.parameters.append(0)#distance d(currentLocation,furthestAllocatedRoom)+d(home,FurthestAllocatedRoom)
        
        self.readRoomsFile()#read rooms from file and store them, will be used later, contains all rooms and types of room

        rospy.Subscriber("amcl_pose",PoseWithCovarianceStamped,self.callbackPose)#subscription to findRobPosition
        rospy.Subscriber("cmd_vel",Twist,self.callbackCmdVel)#subscription to cmd_vel
        rospy.Subscriber("move_base/result",MoveBaseActionResult,self.getGoalResult)#subscription to status of the goal(identify when robot is stuck)

        self.roomCompletedPub = rospy.Publisher('roomCompleted', std_msgs.msg.Int32, queue_size=10)        

        # JRH: subscribe to find work
        rospy.Subscriber("rooms", std_msgs.msg.String, self.work_assignment_callback)
        rospy.Subscriber("battery_empty_stop", std_msgs.msg.String, self.battery_empty_callback)

        #rospy.Subscriber("move_base/status",GoalStatusArray,self.callbackStatusGoal)#subscription to status of the goal(identify when robot is stuck)
        #self.obj=rospy.Subscriber("move_base/status",GoalStatusArray,self.callbackStatusGoal)
        self.ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)#make the movebaseaction global
        """Checks modeFailureInjection, if 1--> on; 0-->off, will not execute the failure """
        
        #self.socketManagerRunTh=Thread(target=self.run)#thread that executes run
        #self.socketManagerRunTh.start()

        if(self.modeFailureInjection==1):
            self.t=Thread(target=self.triggerClockMonitor)#thread that handles the trigger of failure injection
            self.t.start()
        else:
            pass
    
        
        self.fm=Thread(target=self.failureMonitor)#thread that handles the failureMonitor
        self.fm.start()

        self.piT=Thread(target=self.piRetryMonitor)#thread that handles the failureMonitor
        self.piT.start()

        #self.sendInfo=Thread(target=self.sendInfoToJava)#thread that handles the sendInfoToJava
        #self.sendInfo.start()
        #change thisthread so that not use self.intermediary has to send to the socket

        self.tRead=Thread(target=self.readConfValues)#thread that handles the function that reads the conf values
        self.tRead.start()

        self.currentRoomMonitorTh=Thread(target=self.currentRoomMonitor)#thread that handles the current room status
        self.currentRoomMonitorTh.start()

        self.distanceMonitorTh=Thread(target=self.totalDistanceMonitor)#thread that publishes the distance
        self.distanceMonitorTh.start()

        #self.socketManagerTh=Thread(target=self.client_program2)#thread that publishes the distance
        #//self.socketManagerTh.start()
        self.socketManagerTh=Thread(target=self.roomsEmulator)#thread that publishes the distance
        self.socketManagerTh.start()
        self.run()
    
    def work_assignment_callback(self,msg):
        print("work_assignment_callback on " + self.robotName + " : " + msg.data)
        workStrings = msg.data.split(",")
        d=[]

        if (not self.batteryEmpty):
            for r in workStrings:
                d.append(int(r))

            d=deque(d)
            self.mySetOfRooms=d
            self.ac.cancel_all_goals()
            self.newRooms = True
            rc=self.indexToRooms(self.mySetOfRooms)
            print("My new rooms: ", self.mySetOfRooms)
            setDistance=[]#distances from home to each of the assigned rooms
            for i in range(0, len(rc)):
                #calculates distances from home robot to each one of the assigned rooms, distances are stored in setDistance
                setDistance.append(self.calculateDistanceBetweenPoints(self.homeCor[0],self.homeCor[1],rc[i][0],rc[i][1]))
                ##print(setDistance)
            m=self.findLargestDistance(setDistance)#return the index of the largest distance, coordinates of the room are same index in rc
            self.furthestAllocatedRoom[0]=(rc[m][0])#xCoordinate is added
            self.furthestAllocatedRoom[1]=(rc[m][1])#yCoordinate is added

    def battery_empty_callback(self,msg):
        print("battery_empty_callback on " + self.robotName + " : " , msg.data)
        stop = msg.data
        
        if (stop == "STOP"):
            print("Stopping vehicle " + self.robotName)
            currentPos = self.parameters[0]
            self.mySetOfRooms=[]
            self.batteryEmpty = True
            self.ac.cancel_goal()
            self.ac.cancel_all_goals()

            self.newRooms = True
            rc=self.indexToRooms(self.mySetOfRooms)
            print("My new rooms: ", self.mySetOfRooms)
            setDistance=[]#distances from home to each of the assigned rooms
            for i in range(0, len(rc)):
                #calculates distances from home robot to each one of the assigned rooms, distances are stored in setDistance
                setDistance.append(self.calculateDistanceBetweenPoints(self.homeCor[0],self.homeCor[1],rc[i][0],rc[i][1]))
                ##print(setDistance)
            # JRH: taking out as we don't need this when rooms are empty
            #m=self.findLargestDistance(setDistance)#return the index of the largest distance, coordinates of the room are same index in rc
            #self.furthestAllocatedRoom[0]=(rc[m][0])#xCoordinate is added
            #self.furthestAllocatedRoom[1]=(rc[m][1])#yCoordinate is added
            print("Setting goal to current pos: ", currentPos)
            self.moveToGoal(currentPos[0], currentPos[1])

    def roomsEmulator(self):
        d = []
        if(self.robotName=="/tb3_0/"):
            d.append(1)
            d.append(2)
            d.append(6)
        elif(self.robotName=="/tb3_1/"):
            d.append(3)
            d.append(5)
            d.append(7)
        elif(self.robotName=="/tb3_2/"):
            d.append(4)
            d.append(8)
            d.append(9)

        ##print "Received from controller: ", d
        if (d[0] == "p3"):
            self.p3Full = float(d[1])
        else:
            d=deque(d)
            self.mySetOfRooms=d
            self.ac.cancel_all_goals()
            self.newRooms = True
            rc=self.indexToRooms(self.mySetOfRooms)
            ##print("My new rooms: ", self.mySetOfRooms)
            setDistance=[]#distances from home to each of the assigned rooms
            for i in range(0, len(rc)):
                    #calculates distances from home robot to each one of the assigned rooms, distances are stored in setDistance
                setDistance.append(self.calculateDistanceBetweenPoints(self.homeCor[0],self.homeCor[1],rc[i][0],rc[i][1]))
            ##print(setDistance)
            m=self.findLargestDistance(setDistance)#return the index of the largest distance, coordinates of the room are same index in rc
            self.furthestAllocatedRoom[0]=(rc[m][0])#xCoordinate is added
            self.furthestAllocatedRoom[1]=(rc[m][1])#yCoordinate is added
                ##print(rc)		

    # This method gets the result of the current goal, publishes information only when the goal has been finished (succeeded or not)"""
    def getGoalResult(self,array):
        #info about the idGoal is also available
        #3 means succeeded,4 aborted(stuck), 2 preempted (when canceled due to failure injection)
        return array.status.status
    

    """checks the isFailureInjected parameter each second and updates the info in the list that will be send to java"""
    def failureMonitor(self):
        while(True):
            #state = self.gateway.jvm.java.util.ArrayList()#java List definition
            state=[]
            state.append(self.isFailureInjected)
            self.parameters[3]=state
            #self.parameters.set(3,state)
            rospy.sleep(1.)

    def piRetryMonitor(self):
        while(True):
            #listPiRetry = self.gateway.jvm.java.util.ArrayList()#java List definition
            listPiRetry=[]
            listPiRetry.append(self.piRetry)
            ##print(self.piRetry)
            self.parameters[4]=listPiRetry
            #self.parameters.set(4,listPiRetry)
            rospy.sleep(1.)

    """monitors the currnet room and when is done"""	
    def currentRoomMonitor(self):
        while(True):
            ##print(self.currentRoom)
            #roomState = self.gateway.jvm.java.util.ArrayList()#java List definition
            roomState=[]
            roomState.append(self.currentRoom)
            roomState.append(self.statusCurrentRoom)
            self.parameters[5]=roomState
            #self.parameters.set(5,roomState)
            ##print(roomState)
            rospy.sleep(1.)

    def totalDistanceMonitor(self):
        while(True):
            self.furthestToHome=self.calculateDistanceBetweenPoints(self.homeCor[0],self.homeCor[1],self.furthestAllocatedRoom[0],self.furthestAllocatedRoom[1])
            self.totalDistance=self.currentDistance+self.furthestToHome
            self.parameters[6]=self.totalDistance
            #self.parameters.set(6,self.totalDistance)
            rospy.sleep(1)

    def sendInfoToJava(self):
        while(True):
            #print(self.parameters)
            #self.intermediary.receiveRobotData(self.parameters)#invoques receive info from the javaClass
            rospy.sleep(1.)
    
    
    #receives vx and vy to obtain v^2=vx^2+vy^2
    def speedCalculator(self,vx,vy):
        #vx=0.115721046925
        #vy=7.36029760446e-05
        v=math.sqrt((vx**2)+(vy**2))
        return v
    """chooses the rooms coordinates from the bank of coordinates given the index or no of room to ve served"""
    def roomsToServe(self,indexRoom):
        coordinates=[]	
        for i in range(0,len(indexRoom)):
            coordinates.append(self.allTheRooms[indexRoom[i]])
        return coordinates

    def indexToRooms(self,indexRoom):
        coordinates=[]	
        for e in indexRoom:
            coordinates.append(self.allTheRooms[int(e)])
            #coordinates.append(self.allTheRooms[indexRoom[int(e)]])
        ###print(coordinates)
        return coordinates	


    def client_program2(self):
        #host = socket.gethostname()  # as both code is running on same pc
        host = "127.0.0.1"#socket.gethostname()  # as both code is running on same pc
        #host  = "2001:630:61:4c00::6f"
        if self.robotName == "/tb3_0/":
            port = 8880  # socket server port number
        elif self.robotName == "/tb3_1/":
            port = 8881	
        else:
            port = 8882
        client_socket = socket.socket()  # instantiate
        client_socket.connect((host, port))  # connect to the server

        message = "Hello\n"#input(" -> ")  # take input
        while message.lower().strip() != 'bye':
                #client_socket.send(message.encode())  # send message
                ###print(str(self.parameters))
                #a=str(self.parameters)+"\n"

#			flattenParams =  ",".join(map(str, self.parameters))#map(','.join, self.parameters)
            flattenParams = self.flattenList(self.parameters)
                ##print("Sending to controller: ", flattenParams)
            a=str(flattenParams) + "\n"

            a=a.encode()
            client_socket.send(a)
            client_socket.settimeout(None)
            d=client_socket.recv(1024).decode()  # receive response
            d=str(d)#first to string
            d=d.split(',')#then split, otherwhise ',' are considered character
            ##print("Received from controller: ", d)
            if (d[0] == "p3"):
                self.p3Full = float(d[1])
            else:
                d=deque(d)
                self.mySetOfRooms=d
                self.ac.cancel_all_goals()
                self.newRooms = True
                rc=self.indexToRooms(self.mySetOfRooms)
                ##print("My new rooms: ", self.mySetOfRooms)
                setDistance=[]#distances from home to each of the assigned rooms
                for i in range(0, len(rc)):
                    #calculates distances from home robot to each one of the assigned rooms, distances are stored in setDistance
                    setDistance.append(self.calculateDistanceBetweenPoints(self.homeCor[0],self.homeCor[1],rc[i][0],rc[i][1]))
                    ##print(setDistance)
                m=self.findLargestDistance(setDistance)#return the index of the largest distance, coordinates of the room are same index in rc
                self.furthestAllocatedRoom[0]=(rc[m][0])#xCoordinate is added
                self.furthestAllocatedRoom[1]=(rc[m][1])#yCoordinate is added
                ##print(rc)				
        
        client_socket.close()  # close the connection	


    def run(self):
        home = True
        while True:
            #while I have rooms to serve, do serve them
            while self.mySetOfRooms:
                if(not self.isFailureInjected):#if failure is not injected then do visit rooms	
                    home = False
                    index=int(self.mySetOfRooms.popleft())
                    currentGoal=self.allTheRooms[index]
                    flag=self.moveToGoal(currentGoal[0],currentGoal[1])
                    self.newRooms = False
                    if flag==True:
                        #if the goal was reached is added to succeedeed goals and before sending the next goal tasks will be executed
                        #check if is correct how the robot reports when is done with a room
                        if(currentGoal[2]==1):#type of room is 1 or 2
                            roomExecuted = self.executeTasksRoom1()
                            ###print("Room " + str(index) + " done")
                            #self.currentRoom=str(index)+","+str(currentGoal[0])+","+str(currentGoal[1])+","+str(currentGoal[2])#the coordinates and index of the current room are stored
                            #self.statusCurrentRoom=True #room was finished 
                        elif(currentGoal[2]==2):
                            roomExecuted = self.executeTasksRoom2()
                            ###print("Room " + str(index) + " done")
                            ###print("Do room 2 here")
                            #self.currentRoom=str(index)+","+str(currentGoal[0])+","+str(currentGoal[1])+","+str(currentGoal[2])#the coordinates and index of the current room are stored
                            #self.statusCurrentRoom=True #room whas finished

                        #self.success.append(index)#succeeded index of served rooms are added
                        if (roomExecuted == True):
                            ##print("Room " + str(index) + " done")
                            self.currentRoom=str(index)+","+str(currentGoal[0])+","+str(currentGoal[1])+","+str(currentGoal[2])#the coordinates and index of the current
                            self.statusCurrentRoom=True #room was finished
                            self.success.append(index)#succeeded index of served rooms are added
                            ### JRH: publish TO /roomsCompleted here
                            self.roomCompletedPub.publish(std_msgs.msg.Int32(index))

                    elif flag==False:
                        self.currentRoom=str(index)+","+str(currentGoal[0])+","+str(currentGoal[1])+","+str(currentGoal[2])#the coordinates and index of the current room are stored
                        self.statusCurrentRoom=False #means that the robot failed the current room (only when is not reached doesn't takes into account when smth goes wrong with tasks) 
                        if(self.isFailureInjected):#if failure then retry current failed room
                            self.mySetOfRooms.appendleft(index)
  
                        self.failure.append(index)

            #If I do not have any more rooms to serve, go to my home location
            if (not home): 
                flag=self.moveToGoal(self.homeCor[0], self.homeCor[1])
                home = True



    def readRoomsFile(self):
        with open('/home/jharbin/configurationFiles/simulatedRoomCoordinates.txt', 'r') as filehandle:
            fileContent= [current_place.rstrip() for current_place in filehandle.readlines()]#read the file
        self.allTheRooms=self.transformCoordinates(fileContent)

            
    def executeTasksRoom1(self):
        ##print("Entering room type 1")
        self.taskOnePerformer()#always executes t1
        #self.taskTwoPerformer()#ask if t2 will vary, for sure will be retried
        t2Version=self.retryVerifier(self.p2Norm)
        if(t2Version==1):#means perform the normal version of the task
            ##print("Task 2 (normal version)")
            thisLambda=self.l2Normal
        elif(t2Version==0):#means perform the extended version, use normal lambda for calculating the time
            ##print("Task 2 (extended version)")
            thisLambda=self.l2Extended
        if (self.newRooms == True):
            return (False)
        self.taskTwoPerformer(thisLambda)#t2 will be executed always once at least
        while(self.retryVerifier(self.piRetry)==1):#after one execution the task might be retried
            if (self.newRooms == True):
                return (False)		
            self.taskTwoPerformer(thisLambda)#send this lambda
        return (True)


    """thie method executes tasks in room type two"""
    def executeTasksRoom2(self):
        ##print("Entering room type 2")
        self.taskOnePerformer()#always executes t1
        isReq=self.retryVerifier(self.p2Req) #check if t2 is required
        if(isReq==1):#will be req then perform task 2
            ##print("Task 2 is required")
            if (self.newRooms == True):
                return (False)
            self.taskTwoPerformer(self.l2)#t2 will be executed at least once
            while(self.retryVerifier(self.piRetry)==1):#retry with some probability
                if (self.newRooms == True):
                    return (False)
                ###print("entro a retry T2")
                self.taskTwoPerformer(self.l2)#Lambda two will be executed in room3
        isPos=self.retryVerifier(self.p3Poss)
        if(isPos==1):#means that is possible to perform t3
            isFull=self.retryVerifier(self.p3Full)#asks whether will be full or basic
            if (self.newRooms == True):
                return (False)
            if(isFull==1):#perform t3 in full mode: send lambdaFull self.l3Full
                ##print("Task 3 (full version)")
                self.taskThreePerformer(self.l3Full)
            else:#execute the basic mode
                ##print("Task 3 (basic version)")
                self.taskThreePerformer(self.l3Basic)
        return (True)

    def taskOnePerformer(self):
        t=self.timePerTaskCalulator(self.l1)#calculates the time that the robot will spend performing the task, you pass the mean
        ##print("Performing task 1 for", t, "seconds")
        ###print("time to sleep")
        ##print(t)
        rospy.sleep(t)

    def taskTwoPerformer(self,l):
        t=self.timePerTaskCalulator(l)#calculates the time that the robot will spend performing the task, you pass the mean
        #print("Performing task 2 for", t, "seconds")
        ##print("time to sleep")
        ##print(t)
        rospy.sleep(t)
    """performs task3 and will spend some time depending on the received mean"""
    def taskThreePerformer(self,l):
        ##print("Performing task 3")
        t=self.timePerTaskCalulator(l)#calculates the time that the robot will spend performing the task, you pass the mean
        #print("Performing task 3 for", t, "seconds")
        ##print("time to sleep")
        ##print(t)
        rospy.sleep(t)

    """this function compares the passed value with the randomly generated to check if a task will be req or retried"""
    def retryVerifier(self, toCompare):
        #chosen=choice([0,1],p=[self.piRetry,1-self.piRetry])#0 -> retry t2;1 done
        r=0
        #val=np.random.normal(0,1,1)#random number between 0 and 1
        val=random.uniform(0, 1)
        #print("random value", val,  " vs ", toCompare) 
        ##print(val)
        ##print("to compare value")
        ##print(toCompare)
        if (toCompare>val):
            r=1#means retry or req
            ##print("will retry or is req")
        else:
            r=0
            ##print("won't retry or not req")
        ##print("1 retry,  0 no retry")
        ##print(r)
        #r=0
        return r #0 -> retry t2;1 -> t2 done
    """returns the time based on the mean, now I am using seame mean(lambda) for the 3 tasks may be differtent for each full or basic ask!!"""
    def timePerTaskCalulator(self,mean):
        #link with info of the function-> https://www.sharpsightlabs.com/blog/numpy-random-normal/
        val=np.random.normal(loc=mean,scale=0.5,size=None)#loc=mean;scale is the standardDeviation
        return round(val)

        """This function changes the value of piRetry, this variable has a nominal value and given some intervals, this value will be changed for as long as the interval dictates"""
    def readConfValues(self):
        """piRetryConf.txt has 4 lines, the first line is nominal value of piRetry, the second line has the intrevals and
        the value that piRetry will take during that interval. The third line contains l1,l2,l3 in that order. Finally
        line 4 has l3Full,l3Basic,p2Req,p3Poss and p3Full (same order as listed)"""
        fileContent= []
        with open('/home/jharbin/configurationFiles/piRetryConf.txt', 'r') as filehandle:
            fileContent= [current_place.rstrip() for current_place in filehandle.readlines()]#read the file
        self.nominalPiRetry=float(fileContent[0])#nominal value of piRetry
        means=fileContent[2].split(",")#get means and in the next 3 lines are asigned
        otherConfValues=fileContent[3].split(",")#get l3Full,l3Basic,p2Req,p3Poss and p3Full
        ##print(otherConfValues)
        self.l1=int(means[0])
        self.l2=int(means[1])
        self.l3=int(means[2])

        self.l3Full=float(otherConfValues[0])
        self.l3Basic=float(otherConfValues[1])
        self.p2Req=float(otherConfValues[2])
        self.p3Poss=float(otherConfValues[3])
        self.p3Full=float(otherConfValues[4])
        
        self.piRetry=self.nominalPiRetry#at the begining piRetry
        piIntervals=self.transformPyRetryInrevals(fileContent[1])
        for i in range(0,len(piIntervals)): 
            rospy.Subscriber("/clock",Clock,self.callbackClockDos,(piIntervals[i][0],piIntervals[i][1],piIntervals[i][2])) #subscription to clockDos 
            now = rospy.get_rostime()#current time
            ttw=piIntervals[i][1]-now.secs#timeToWait is end of the interval-currentTime
            rospy.sleep(ttw)#wait the time that the interval requires
            #the thread that will change pyRetry to the nominal value again after the time interval has finished
            tPyNominal=Thread(target=self.changePiRetryToNominal)
            tPyNominal.start()

    """receives the array ['a:b:c,d:e:f,...x:y:z'] and returns [[a,b,c],[d,e,f],...[x,y,z]]"""	
    def transformPyRetryInrevals(self,intervals):
        temp=[]
        final=[]
        temp=intervals.split(",")
        for i in range(0,len(temp)):
            final.append([float(x) for x in temp[i].split(":")])
        return final

    """this function is handled by a thread otherwise the main thread is affected for the sleep below,
    the for loop is iterating through a list of time intervals, the first time sends the interval to the function that gets
    info from the clock and sends start and end of the interval, the function sleeps during an amount
    of time (the end of the interval-current_time) this time is required so that callbackClock 
    can verify the beginig and the end of the interval and during this time the failure is triggered.
    After that the next interval is sended. The process is repeated as many times as values in the array"""	
    def triggerClockMonitor(self):
        """failureIntervals are requested to java and stored in an arrayList"""
        #this.file="/home/osboxes/configurationFiles/failureIntervals.txt";
        lista=[]
        fileContent= []
        with open('/home/jharbin/configurationFiles/failureIntervals.txt', 'r') as filehandle:
            fileContent= [current_place.rstrip() for current_place in filehandle.readlines()]#read the file
        if(fileContent[0]==self.robotName):
            lista.append(fileContent[1])
        elif(fileContent[2]==self.robotName):
            lista.append(fileContent[3])
        elif(fileContent[4]==self.robotName):
            lista.append(fileContent[5])
        
        #failureIntervals=self.intermediary.getFailureIntervals(self.robotName)
        failureI=self.transformFailureIntervals(lista)
        
        #lista=[[inicio,fin],[inicio+30,fin+30]]
        for i in range(0,len(failureI)):
            rospy.Subscriber("/clock",Clock,self.callbackClock,(failureI[i][0],failureI[i][1])) #subscription to clock 
            now = rospy.get_rostime()
            e=failureI[i][1]-now.secs
            #print("waiting...")		
            rospy.sleep(e)
            """after the sleep time has finished then the failure is over and you have to resume the navigation;
            if you do not create another thread to execute stopFailure the next failure interval is not scheduled"""
            threadStop=Thread(target=self.stopFailure)
            threadStop.start()
            

        
    """This function gets the info from clock in secs; activates and deactivates a flag when the interval is meet, used for injecting the failure"""
    def callbackClock(self,data,args):
            if data.clock.secs==args[0]:
                threadFailure=Thread(target=self.triggerFailure)
                threadFailure.start()
                #rospy.sleep(1.)#the sleep is because this method is executed per millisecond!
            #if data.clock.secs==125:
            #	threadPi=Thread(target=self.triggerPi(666))
            #	threadPi.start()

    """This function gets the info from clock in secs (same as callbackClock); this is used for managing when to change piRetry"""		
    def callbackClockDos(self,data,args):
            if data.clock.secs==args[0]:
                threadPi=Thread(target=self.triggerPi(args[2]))#args[2] has the temporal value of piRetry
                threadPi.start()
    
    def triggerPi(self,value):
        self.piRetry=value
        

    """This function is executed when the interval of failure is reached, sets a flag so that getRooms identifies
    when to save the counter and stop sending goals to moveBase; finally cancel all goals and the robot stays still.
    NOTE goals have to be sent to the moveBase for injecting the failure otherwise will not have any effect"""	
    def triggerFailure(self):
        self.isFailureInjected=True
        self.ac.cancel_all_goals()
            #self.failure=self.mySetOfRooms #current set of rooms is saved to be later resumed

            
    """changes the falg of failure to false and executes the method that resumes the goals"""		
    def stopFailure(self):
        self.isFailureInjected=False
    

    """funtion that changes piRetry to nominal again after the time interval has finished"""
    def changePiRetryToNominal(self):
        self.piRetry=self.nominalPiRetry

    """ transformCoordinates converts string list into float list and separates the coordinates e.g. [['x,y'],['x2,y2']...] into [[x,y],[x2,y2]]"""
    def transformCoordinates(self,coordinates):
        newCor=[]
        for i in range(0,len(coordinates)):
            intCoord=[float(x) for x in coordinates[i].split(",")]#converts the string list inot int list
            newCor.append(intCoord)		
        return newCor

    """when received from java intervals are one string inside an arrayList ["a:b,c:d,e:f"], this method splits the array into [[a,b],[c,d],[e,f]]"""
    def transformFailureIntervals(self,failureIntervals):
        newInter=[]
        final=[]
        newInter=failureIntervals[0].split(",")
        for i in range(0,len(newInter)):
            final.append([int(x) for x in newInter[i].split(":")])
        return final	
        

    #vx and vy are obtained from topic /cmd_vel to obtain absolute speed
    def callbackCmdVel(self,array):
        #listSpeed = self.gateway.jvm.java.util.ArrayList()#java List definition
        listSpeed=[]
        listSpeed.append(self.speedCalculator(array.linear.x,array.linear.y))#send velocity components to speedCalculator to get the magnitude of total velocity; then add the v into array
        self.parameters[2]=listSpeed
        #self.parameters.set(2,listSpeed)
        #self.intermediary.receiveSpeed(arraySpeed)#invoques receiveCoordinates from the javaClass
        
    
    #Gets position of the robot and sends info to JAVA
    def callbackPose(self,array):
        #currentPos = self.gateway.jvm.java.util.ArrayList()#java List definition
        #l.append(self.robotName)
        currentPos=[]
        currentPos.append(array.pose.pose.position.x)#either append or add are valid
        currentPos.append(array.pose.pose.position.y)
        self.currentDistance=self.calculateDistanceBetweenPoints(float(array.pose.pose.position.x),float(array.pose.pose.position.y),self.furthestAllocatedRoom[0],self.furthestAllocatedRoom[1],)
        #self.parameters.set(1,currentPos)
        self.parameters[1]=currentPos
        #self.intermediary.receiveCoordinates(l)#invoques receiveCoordinates from the javaClass
    
    """returns the index of the largest distance"""	
    def findLargestDistance(self,arr):
            m=arr[0]
            index=0
            for i in range(0,len(arr)):
                if arr[i]>m:
                    m=arr[i]
                    index=i
            return(index)
    
    def calculateDistanceBetweenPoints(self,x1,y1,x2,y2):
        #d=sqrt((x2-x1)^2+(y2-y1)^2) distance formula
        d=math.sqrt(((x2-x1)**2)+((y2-y1)**2))
        return d

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


    def flattenList(self, nested_list):
        flat_list = []
        for sublist in nested_list:
            if hasattr(sublist, "__iter__") and not isinstance(sublist, str):
                for item in sublist:
                    flat_list.append(item)
            else:
#			except TypeError as te:
                flat_list.append(sublist)

        return flat_list



if __name__ == '__main__':
    try:
        random.seed( 100 ) #first call  
        Map_navigation()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
