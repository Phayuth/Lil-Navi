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
		self.x = 0
		self.y = 0
		self.tt = 0

		self.loopr = rospy.Rate(20)

		rospy.Subscriber('/robotros_test/odom',Odometry, self.odomcall)

		self.odom_pub = rospy.Publisher("/odomer", Odometry, queue_size = 50)

	def odomcall(self,msg):
		v = msg.twist.twist.linear.x
		w = msg.twist.twist.angular.z
		Ts = 0.05
		self.x = self.x + v*np.cos(self.tt)*Ts
		self.y = self.y + v*np.sin(self.tt)*Ts
		self.tt = self.tt + w*Ts
		#rospy.loginfo(str(euler[2])+" "+str(self.x) + " "+str(self.y) +  " "str(self.tt))

	def start(self):
		while not rospy.is_shutdown():
			odoq = Odometry()
			odom_broadcaster = tf.TransformBroadcaster()
			odom_quat = tf.transformations.quaternion_from_euler(0,0,self.tt)
			odom_broadcaster.sendTransform((self.x,self.y,0.),odom_quat,rospy.Time.now(),"base_linkd","odomd")
			odoq.header.frame_id = "odomd"
			odoq.pose.pose = Pose(Point(self.x,self.y,0.),Quaternion(*odom_quat))
			odoq.child_frame_id = "base_linkd"
			self.odom_pub.publish(odoq)
			self.loopr.sleep()


if __name__ == '__main__':
	rospy.init_node("testodom")
	oo = odomest()
	oo.start()