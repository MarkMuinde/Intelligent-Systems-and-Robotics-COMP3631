#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

def publisher():
	pub = rospy.Publisher('mobile_base/commands/velocity', Twist)
	rospy.init_node('Walker', anonymous=True)
	rate = rospy.Rate(10) #10hz
	desired_velocity = Twist()

	desired_velocity.linear.x = 1.0 #linear velocity
	desired_velocity.angular.z = 0.5 #angular velocity
	
	while not rospy.is_shutdown():
		pub.publish(desired_velocity)
		rate.sleep()


if __name__ == "__main__":
	try:
		publisher()
	except rospy.ROSInterruptException:
		pass
