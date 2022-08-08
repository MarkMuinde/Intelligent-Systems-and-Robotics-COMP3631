#!/usr/bin/env python

# Detecting poster colour to determine character

from __future__ import division
import cv2
import numpy as np
import rospy
import sys
import os

from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

img_path = os.path.expanduser('~/catkin_ws/src/group_project/output/cluedo_character.png')
text_path = os.path.expanduser('~/catkin_ws/src/group_project/output/cluedo_character.txt')

class characterIdentifier():

    def __init__(self):
        self.sensitivity = 5
        self.bridge = CvBridge()
        self.sub = rospy.Subscriber('camera/rgb/image_raw', Image, self.callback)
        self.scarlett_flag = 0
        self.peacock_flag = 0
        self.mustard_flag = 0
        self.plum_flag = 0

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data,"bgr8")
        except:
            print("Conversion failed!")
        # Upper and lower bounds for the four colours to identify
        hsv_scarlett_lower = np.array([5-self.sensitivity,100,100])
        hsv_scarlett_upper = np.array([5+self.sensitivity,255,255])
        hsv_peacock_lower = np.array([105-self.sensitivity,100,100])
        hsv_peacock_upper = np.array([105+self.sensitivity,255,255]) # can't see her dress
        hsv_mustard_lower = np.array([25-self.sensitivity,100,100])
        hsv_mustard_upper = np.array([25+self.sensitivity,255,255])
        hsv_plum_lower = np.array([153-self.sensitivity,100,10])
        hsv_plum_upper = np.array([153+self.sensitivity,225,255]) # not as clar as others
        # Convert the bgr image into a hsv image
        hsv_image = cv2.cvtColor(cv_image,cv2.COLOR_BGR2HSV)
        # Filter out everything but particular colours using the cv2.inRange() method
        mask1 = cv2.inRange(hsv_image,hsv_scarlett_lower,hsv_scarlett_upper)
        mask2 = cv2.inRange(hsv_image,hsv_peacock_lower,hsv_peacock_upper)
        mask3 = cv2.inRange(hsv_image,hsv_mustard_lower,hsv_mustard_upper)
        mask4 = cv2.inRange(hsv_image,hsv_plum_lower,hsv_plum_upper)

        # detecting plum
        img_plum = cv2.bitwise_and(cv_image,cv_image,mask=mask4)
        contours_plum, hierarchy = cv2.findContours(mask4, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        if len(contours_plum) > 0:
            c = max(contours_plum,key=cv2.contourArea)
            if cv2.contourArea(c) > 5500:
                x, y, w, h = cv2.boundingRect(c)
                cv2.rectangle(img_plum,(x,y),(x+w, y+h),(0,0,255),2)
                self.plum_flag = 1

        # detecting peacock
        img_peacock = cv2.bitwise_and(cv_image,cv_image,mask=mask2)
        contours_peacock, hierarchy = cv2.findContours(mask2, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        if len(contours_peacock) > 0:
            c = max(contours_peacock,key=cv2.contourArea)
            if cv2.contourArea(c) > 5500:
                x, y, w, h = cv2.boundingRect(c)
                cv2.rectangle(img_peacock,(x,y),(x+w, y+h),(0,0,255),2)
                self.peacock_flag = 1

        # detecting scarlett
        img_scarlett= cv2.bitwise_and(cv_image,cv_image,mask=mask1)
        contours_scarlett, hierarchy = cv2.findContours(mask1, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        if len(contours_scarlett) > 0:
            c = max(contours_scarlett,key=cv2.contourArea)
            if cv2.contourArea(c) > 5500:
                x, y, w, h = cv2.boundingRect(c)
                cv2.rectangle(img_scarlett,(x,y),(x+w, y+h),(0,0,255),2)
                self.scarlett_flag = 1

        # detecting mustard
        img_mustard = cv2.bitwise_and(cv_image,cv_image,mask=mask3)
        contours_mustard, hierarchy = cv2.findContours(mask3, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        if len(contours_mustard) > 0:
            c = max(contours_mustard,key=cv2.contourArea)
            if cv2.contourArea(c) > 5500:
                x, y, w, h = cv2.boundingRect(c)
                cv2.rectangle(img_mustard,(x,y),(x+w, y+h),(0,0,255),2)
                self.mustard_flag = 1

        cv2.namedWindow('camera_Feed')
        cv2.imshow('camera_Feed', cv_image)
        cv2.waitKey(3)

        file = open(text_path, "w")
        if (self.scarlett_flag == 1):
            cv2.imwrite(img_path, cv_image)
            file.write("Miss Scarlett")
            print("Saved files to current working directory!")
        elif (self.peacock_flag == 1):
            cv2.imwrite(img_path, cv_image)
            file.write("Mrs Peacock")
            print("Saved files to current working directory!")
        elif (self.mustard_flag == 1):
            cv2.imwrite(img_path, cv_image)
            file.write("Colonel Mustard")
            print("Saved files to current working directory!")
        elif (self.plum_flag == 1):
            cv2.imwrite(img_path, cv_image)
            file.write("Professor Plum")
            print("Saved files to current working directory!")
        file.close()



def main(args):
    rospy.init_node('Camera', anonymous=True)
    cI = characterIdentifier()
    try:
        rospy.spin()
    except KeyboardInterrupts:
        pass

    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
