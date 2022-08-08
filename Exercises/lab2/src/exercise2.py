#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

def publisher():
	pub = rospy.Publisher('mobile_base/commands/velocity', Twist)
	rospy.init_node('Walker', anonymous=True)
	rate = rospy.Rate(10) #10hz

	desired_velocity = Twist() #for moving straight
	desired_velocity_turn = Twist() #for turning the corners

	desired_velocity.linear.x = 1.0 #linear velocity

	#1.57/3 radians/sec since a range of 30 and a rate of 10hZ means 3 seconds
	#so the angular velocity becomes 1.57/3 * 3 = 1.57 radians which is approx.
	#a right angle
	desired_velocity_turn.angular.z = 1.57/3  #angular velocity

	while not rospy.is_shutdown():

		#move straight
		for i in range (30):
			pub.publish(desired_velocity)
			rate.sleep()

		#turn at a right angle
		for i in range (30):
			pub.publish(desired_velocity_turn)
			rate.sleep()


if __name__ == "__main__":
	try:
		publisher()
	except rospy.ROSInterruptException:
		pass
