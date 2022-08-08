#!/usr/bin/env python

#Exercise 1 - Display an image of the camera feed to the screen

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

        # Remember to initialise a CvBridge() and set up a subscriber to the image topic you wish to use
        # We covered which topic to subscribe to should you wish to receive image data

        self.cvPub = rospy.Publisher('camera/rgb/image_raw',Image)
        self.cvBridge = CvBridge()
        self.cvSub = rospy.Subscriber('camera/rgb/image_raw', Image, self.callback)

    def callback(self, data):
        # Convert the received image into a opencv image
        # But remember that you should always wrap a call to this conversion method in an exception handler
        try:
         cvImage = self.cvBridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        # Show the resultant images you have created.
        cv2.namedWindow('camera_Feed')
        cv2.imshow('camera_Feed', cvImage)
        cv2.waitKey(3)


# Create a node of your class in the main and ensure it stays up and running
# handling exceptions and such
def main(args):
    # Instantiate your class
    # And rospy.init the entire node
    rospy.init_node('image_converter', anonymous=True)
    cI = colourIdentifier()
    # Ensure that the node continues running with rospy.spin()
    # You may need to wrap rospy.spin() in an exception handler in case of KeyboardInterrupts
    # Remember to destroy all image windows before closing node

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()
    # Remember to destroy all image windows before closing node

# Check if the node is executing in the main path
if __name__ == '__main__':
    main(sys.argv)
