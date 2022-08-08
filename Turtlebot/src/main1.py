#!/usr/bin/env python

import yaml
import math
import rospy
from movement import Move
import numpy as np
from geometry_msgs.msg import Twist
from kobuki_msgs.msg import BumperEvent
from identify_circles import colourIdentifier
from img_cla_dect import characterIdentifier
import time


global bump
bump = False

def spin():
    print("spinning")
    pub = rospy.Publisher('mobile_base/commands/velocity', Twist)
    #rospy.init_node('Walker', anonymous=True)
    rate = rospy.Rate(10)
    desired_velocity = Twist()
    desired_velocity.angular.z = 0.6
    pub.publish(desired_velocity)

def processBump(data):
        global bump
        if (data.state == BumperEvent.PRESSED ):
                bump = True
        else:
                bump = False

def listen():
    sub = rospy.Subscriber('mobile_base/events/bumper', BumperEvent, processBump)

def rotate(coord,n, thetas):
    print("rotating")
    theta = thetas
    temp = theta + (math.pi*2)
        


    x = coord[0]
    y = coord[1]
    coordinate = {'x': x, 'y': y}
    quaternion = {'r1' : 0.000, 'r2' : 0.000, 'r3' : np.sin(theta/2.0), 'r4' : np.cos(theta/2.0)} 
    navigator.move(coordinate, quaternion)


    for i in range(n):
        if i < n/2:
            theta = theta + (math.pi*2)/(2*n)
        else:
            temp = temp - (math.pi*2)/(2*n)
            theta = temp

        
        quaternion = {'r1' : 0.000, 'r2' : 0.000, 'r3' : np.sin(theta/2.0), 'r4' : np.cos(theta/2.0)} 
        navigator.move(coordinate, quaternion)

        rospy.sleep(0.1)
        test = characterIdentifier()
        rospy.sleep(1)
        #print("test = ", test.scarlett_flag)
        if test.found != 0:
            print("found ----------")
            return 1
        #theta = theta + (math.pi*2)/n

    # start = time.time()
    # flag = 0
    # while True:
    #     test = characterIdentifier()
    #     flag = test.found
    #     current = time.time()
    #     elapsed = current - start
    #     if elapsed > 15:
    #         break
    #     if flag == 0:
    #         spin()
    #     else:
    #         quit()

def search(coord, thetas):
    print("searching")
    theta = thetas
    x = coord[0]
    y = coord[1]
    coordinate = {'x': x, 'y': y}
    quaternion = {'r1' : 0.000, 'r2' : 0.000, 'r3' : np.sin(theta/2.0), 'r4' : np.cos(theta/2.0)} 
    #quaternion = {'r1' : 0, 'r2' : 0, 'r3' : 1, 'r4' : theta } #np.cos(theta)+np.sin(theta)*1
    navigator.move(coordinate, quaternion)


    pub = rospy.Publisher('mobile_base/commands/velocity', Twist, queue_size = 10)
    #rospy.init_node('Walker', anonymous=True)
    rate = rospy.Rate(10)
    desired_velocity = Twist()
    desired_velocity.linear.x = 1
    while bump == False:
        pub.publish(desired_velocity)
        listen()
    desired_velocity.linear.x = -0.6
    for i in range(15):
        pub.publish(desired_velocity)
        rate.sleep()
    listen()

    

    #rospy.sleep(1)

    theta = theta + (math.pi*2)/8

    return theta

if __name__ == '__main__':


    GreenFlag = False
    GreenFlag2 = False
    RedFlag = False
    RedFlag2 = False
    room = -1
    try:
        rospy.init_node('navigation', anonymous = True)
        navigator = Move()
        
        # CHANGE HARDCODED PATH
        CONST_POINTS_PATH = '/home/csunix/sc19ms2/catkin_ws/src/group_project/world/input_points.yaml'

        from yaml.loader import SafeLoader #import safeloader from pyyaml

        with open(CONST_POINTS_PATH) as f:
            data = yaml.load(f, Loader=SafeLoader)
    

        keys = data.keys()

        room1e = data[keys[0]]
        room2e = data[keys[2]]
        room1c = data[keys[1]]
        room2c = data[keys[3]]
        centerCoord = room1c


        x = room1e[0]
        y = room1e[1]
        theta = 0
        coordinate = {'x': x, 'y': y}
        quaternion = {'r1' : 0.000, 'r2' : 0.000, 'r3' : np.sin(theta/2.0), 'r4' : np.cos(theta/2.0)}
        navigator.move(coordinate, quaternion)

        start = time.time()
        while True:
            current = time.time()
            elapsed = current - start
            spin()
            circleI = colourIdentifier()
            rospy.sleep(1)
            if circleI.green_circle_detected == 1:
                GreenFlag = 1
                break
            if elapsed > 30:
                break
        if(GreenFlag):
            x = room1c[0]
            y = room1c[1]
            #print(x)
            #print(y)
            theta = 0
            coordinate = {'x': x, 'y': y}
            quaternion = {'r1' : 0.000, 'r2' : 0.000, 'r3' : np.sin(theta/2.0), 'r4' : np.cos(theta/2.0)}
            navigator.move(coordinate, quaternion)
            centerCoord = room1c
        else:
            x = room2c[0]
            y = room2c[1]
            theta = 0
            coordinate = {'x': x, 'y': y}
            quaternion = {'r1' : 0.000, 'r2' : 0.000, 'r3' : np.sin(theta/2.0), 'r4' : np.cos(theta/2.0)}
            navigator.move(coordinate, quaternion)
            centerCoord = room2c
    

        x = 0.0       
        for i in range(6):
            if i == 5:
                break
            temp = x
            x = search(centerCoord, x)
            navigator.currentPos()
            coord = [navigator.posx, navigator.posy]
            t = rotate(coord, 4, temp)
            if t == 1:
                print("Found!")
                break
            navigator.move(coordinate, quaternion)

    except rospy.ROSInterruptException:
        rospy.loginfo("Ctrl-C Found - Quitting")

