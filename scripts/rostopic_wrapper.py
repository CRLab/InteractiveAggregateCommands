#!/usr/bin/env python

import rospy

class RostopicWrapperNode():
	def __init__(self):
		rospy.init_node('rostopic_wrapper_node')
		rospy.loginfo("entering rostopic wrapper server ...")
	def grasp_publisher(self):
		pub = rospy.Publisher('/grasp', std_msgs.msg.Empty, queue_size=10)
		rate = rospy.Rate(10)
		while not rospy.is_shutdown():
	        pub.publish()
	        rate.sleep()


	def place_publisher(self):
		pub = rospy.Publisher('/place', std_msgs.msg.Empty, queue_size=10)
		rate = rospy.Rate(10)
		while not rospy.is_shutdown():
	        pub.publish()
	        rate.sleep()


	def navigation_publisher(self):
		pub = rospy.Publisher('/navigation', std_msgs.msg.Empty, queue_size=10)
		rate = rospy.Rate(10)
		while not rospy.is_shutdown():
	        pub.publish()
	        rate.sleep()

	def cartesian_publisher(self):
		pub = rospy.Publisher('/cartesian', std_msgs.msg.Empty, queue_size=10)
		rate = rospy.Rate(10)
		while not rospy.is_shutdown():
	        pub.publish()
	        rate.sleep()

	def play_back_publisher(self):
		pub = rospy.Publisher('/play_back', std_msgs.msg.Empty, queue_size=10)
		rate = rospy.Rate(10)
		while not rospy.is_shutdown():
	        pub.publish()
	        rate.sleep()

	def object_rec_publisher(self):
		pub = rospy.Publisher('/run_recognition', std_msgs.msg.Empty, queue_size=10)
		rate = rospy.Rate(10)
		while not rospy.is_shutdown():
	        pub.publish()
	        rate.sleep()

