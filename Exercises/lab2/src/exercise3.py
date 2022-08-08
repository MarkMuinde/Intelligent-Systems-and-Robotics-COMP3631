#!/usr/bin/env python

import rospy
from kobuki_msgs.msg import BumperEvent
from geometry_msgs.msg import Twist

#function with global boolean variable for bump detection
def bump(data):
	global bump
	if (data.state == BumperEvent.PRESSED):
		bump = True
	else:
		bump = False

def publisher():
	pub = rospy.Publisher('mobile_base/commands/velocity', Twist)
	rospy.init_node('Walker', anonymous=True)
	rate = rospy.Rate(10) #10hz

	#Subscribe to the bumber topic
	rospy.Subscriber('mobile_base/events/bumper', BumperEvent, bump)

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

				#if bumped, stop moving, else go straight
				if bump == True:
					print("just bumped!")
					desired_velocity.linear.x = 0.0
					desired_velocity_turn.angular.z = 0.0
					pub.publish(desired_velocity)
					pub.publish(desired_velocity_turn)
					rate.sleep()
				else:
					pub.publish(desired_velocity)
					rate.sleep()

			#turn at a right angle
			for i in range (30):

				#if bumped, stop moving, else turn
				if bump == True:
					print("just bumped!")
					desired_velocity.linear.x = 0.0
					desired_velocity_turn.angular.z = 0.0
					pub.publish(desired_velocity)
					pub.publish(desired_velocity_turn)
					rate.sleep()
				else:
					pub.publish(desired_velocity_turn)
					rate.sleep()


if __name__ == "__main__":
	try:
		publisher()
	except rospy.ROSInterruptException:
		pass
