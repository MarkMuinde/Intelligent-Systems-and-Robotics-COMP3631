#!/usr/bin/env python

import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion
from nav_msgs.msg import Odometry
import os


class Move():

    def __init__(self):
        self.goal_sent = False
       

        rospy.on_shutdown(self.close)



        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo(os.getcwd())
        
        self.move_base.wait_for_server()

    def move(self, coordinates, quat):

        self.goal_sent = True

        goal = MoveBaseGoal() #create a MoveBaseGoal

        #defined th header of the MoveBaseGoal
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()

        #Provide the coordinates
        goal.target_pose.pose.position.x = coordinates['x']
        goal.target_pose.pose.position.y = coordinates['y']
        goal.target_pose.pose.position.z = 0.0

        #provide the orientation
        goal.target_pose.pose.orientation.x = quat['r1']
        goal.target_pose.pose.orientation.y = quat['r2']
        goal.target_pose.pose.orientation.z = quat['r3']
        goal.target_pose.pose.orientation.w = quat['r4']

        #send the goal to the action server
        self.move_base.send_goal(goal)


        success = self.move_base.wait_for_result(rospy.Duration(60)) #is True if succesful

        state = self.move_base.get_state() # gets the state of the move_base options : (PENDING, ACRIVE, RECALLED, REJECTED, PREEMPTED, ABORTES, SUCCEEDED, LOST)
        result = False

        if success == True and state == GoalStatus.SUCCEEDED:
            result = True
        else:
            self.move_base.cancel_goal()

        self.goal_sent = False
        return result

    def callback(self, msg):
        
        self.posx = msg.pose.pose.position.x
        self.posy = msg.pose.pose.position.y
        

    def callback2(self, msg):
        self.quatx = msg.pose.pose.orientation.x
        self.quaty = msg.pose.pose.orientation.y
        self.quatz = msg.pose.pose.orientation.z
        self.quatw = msg.pose.pose.orientation.w
    
    def currentPos(self):
        odom_sub = rospy.Subscriber('/odom', Odometry, self.callback)
        rospy.sleep(0.5)

    
    def currentQuat(self):
        odom_sub2 = rospy.Subscriber('/odom', Odometry, self.callback2)
        rospy.sleep(0.5)

        #rospy.sleep(3)

    def close(self):
        if self.goal_sent:
            self.move_base.cancel_goal()
        
        rospy.loginfo("STOP")
        rospy.sleep(1)


