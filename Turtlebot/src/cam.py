#!/usr/bin/env python
from __future__ import division
import cv2
import numpy as np
import rospy
import sys

from identify_circles import colourIdentifier
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class cameraFeed():

    def __init__(self):
        # Initialise a publisher to publish messages to the robot base
        # We covered which topic receives messages that move the robot in the 2nd Lab Session
        self.pub = rospy.Publisher('mobile_base/commands/velocity', Twist)
        self.rate = rospy.Rate(10) #10hz
        
        
        # Initialise any flags that signal that an coloured circle has been detected (default to false)
        # self.red_circle_detected = 0
        # self.green_circle_detected = 0
        
        # Initialise the value you wish to use for sensitivity in the colour detection (10 should be enough)
        self.sensitivity = 10
        # Initialise some standard movement messages such as a simple move forward and a message with all zeroes (stop)

        # Remember to initialise a CvBridge() and set up a subscriber to the image topic you wish to use
        self.bridge = CvBridge()
        # We covered which topic to subscribe to should you wish to receive image data
        self.Pub = rospy.Publisher('camera/rgb/image_raw', Image)
        self.sub = rospy.Subscriber('camera/rgb/image_raw', Image, self.callback)

        #indentify_circles
        self.cI = colourIdentifier()

    def callback(self, data):
        # Convert the received image into a opencv image
        # But remember that you should always wrap a call to this conversion method in an exception handler
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")   
            cp_image = cv_image.copy()
        except CvBridgeError as e :
            print("Error Converting cv image: {}".format(e.message))
            
        
        cv2.namedWindow('camera_Feed')
        cv2.imshow('camera_Feed', cv_image)
        cv2.waitKey(3)

    # Create a node of your class in the main and ensure it stays up and running
    # handling exceptions and such
def main(args):
    # Instantiate your class
    # And rospy.init the entire node
    rospy.init_node('image_converter', anonymous = True)
    cf = cameraFeed()
    # Ensure that the node continues running with rospy.spin()
    # You may need to wrap it in an exception handler in case of KeyboardInterrupts
    # Remember to destroy all image windows before closing node
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shut down")

    cv2.destroyAllWindows()

    # Check if the node is executing in the main path
if __name__ == '__main__':
    main(sys.argv)
