#!/usr/bin/env python
# -*- coding: utf-8 -*-
#Exercise 2 - detecting two colours, and filtering out the third colour and background.

from __future__ import division
import cv2
import numpy as np
import rospy
import sys

from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class colourIdentifier():

    def __init__(self):
        # Initialise the value you wish to use for sensitivity in the colour detection (10 should be enough)

        # Remember to initialise a CvBridge() and set up a subscriber to the image topic you wish to use

        # We covered which topic to subscribe to should you wish to receive image data
        self.cvPub = rospy.Publisher('camera/rgb/image_raw',Image)
        self.cvBridge = CvBridge()
        self.cvSub = rospy.Subscriber('camera/rgb/image_raw', Image, self.callback)
        self.sensitivity = int()

    def callback(self, data):
        # Convert the received image into a opencv image
        # But remember that you should always wrap a call to this conversion method in an exception handler
        try:
         cvImage = self.cvBridge.imgmsg_to_cv2(data, "bgr8")
         imgCpy = cvImage.copy()
        except CvBridgeError as e:
            print(e)
        # Set the upper and lower bounds for the two colours you wish to identify
        hsv_green_lower = np.array([60 - self.sensitivity, 100, 100])
        hsv_green_upper = np.array([60 + self.sensitivity, 255, 255])

        hsv_blue_lower = np.array([90, 100, 100])
        hsv_blue_upper = np.array([130, 255, 255])


        # Convert the rgb image into a hsv image
        Hsv_image = cv2.cvtColor(cvImage, cv2.COLOR_BGR2HSV)

        # Filter out everything but particular colours using the cv2.inRange() method
        # Do this for each colour
        maskGreen = cv2.inRange(Hsv_image, hsv_green_lower, hsv_green_upper)
        maskBlue  = cv2.inRange(Hsv_image, hsv_blue_lower, hsv_blue_upper)

        # To combine the masks you should use the cv2.bitwise_or() method
        # You can only bitwise_or two images at once, so multiple calls are necessary for more than two colours
        maskCombined =  cv2.bitwise_or(maskGreen, maskBlue)

        # Apply the mask to the original image using the cv2.bitwise_and() method
        # As mentioned on the worksheet the best way to do this is to...
        #bitwise and an image with itself and pass the mask to the mask parameter (rgb_image,rgb_image, mask=mask)
        # As opposed to performing a bitwise_and on the mask and the image.
        maskFinal = cv2.bitwise_and(imgCpy, imgCpy, mask = maskCombined)


        #Show the resultant images you have created. You can show all of them or just the end result if you wish to.
        cv2.imshow('camera_Feed', cvImage)
        cv2.imshow('mask', maskFinal)
        cv2.waitKey(3)

# Create a node of your class in the main and ensure it stays up and running
# handling exceptions and such
def main(args):
    # Instantiate your class
    # And rospy.init the entire node
    cI = colourIdentifier()
    rospy.init_node('image_converter', anonymous=True)
    # Ensure that the node continues running with rospy.spin()
    # You may need to wrap rospy.spin() in an exception handler in case of KeyboardInterrupts

    # Remember to destroy all image windows before closing node
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()
# Check if the node is executing in the main path
if __name__ == '__main__':
    main(sys.argv)
