#! /usr/bin/env python

import numpy as np
import time

import rospy
from nav_msgs.msg import Odometry
import tf
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, Pose2D


class odomest(object):
	"""docstring for odomest"""
	def __init__(self):
		self.x  = 0
		self.y  = 0
		self.tt = 0

		self.loopr = rospy.Rate(20)

		rospy.Subscriber('/robotros_test/odom',Odometry, self.odomcall)

		self.odom_pub = rospy.Publisher("/Pure_odometry", Odometry, queue_size = 50)

	def odomcall(self,msg):
		v = msg.twist.twist.linear.x  #+ np.random.normal(0,0.1)
		w = msg.twist.twist.angular.z #+ np.random.normal(0,0.1)
		Ts = 0.05
		self.x = self.x + v*np.cos(self.tt)*Ts
		self.y = self.y + v*np.sin(self.tt)*Ts
		self.tt = self.tt + w*Ts

	def start(self):
		while not rospy.is_shutdown():
			odoq = Odometry()
			odom_broadcaster = tf.TransformBroadcaster()
			odom_quat = tf.transformations.quaternion_from_euler(0,0,self.tt)
			odom_broadcaster.sendTransform((self.x,self.y,0.),odom_quat,rospy.Time.now(),"pure_odom_base","pure_odom_odom")
			odoq.header.frame_id = "pure_odom_odom"
			odoq.pose.pose = Pose(Point(self.x,self.y,0.),Quaternion(*odom_quat))
			odoq.child_frame_id = "pure_odom_base"
			self.odom_pub.publish(odoq)
			self.loopr.sleep()


if __name__ == '__main__':
	rospy.init_node("odom_pure")
	oo = odomest()
	oo.start()
