#! /usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Float64MultiArray

class nodod(object):
	def __init__(self):
		self.pure_x = 0
		self.pure_y = 0
		self.true_x = 0
		self.true_y = 0
		self.ekf_x = 0
		self.ekf_y = 0
		self.loopr = rospy.Rate(20)

		rospy.Subscriber("/pose2D",Pose2D,self.call1)
		rospy.Subscriber("/robotros_test/odom",Odometry,self.call2)
		rospy.Subscriber("/poseodom_pure",Odometry,self.call3)

		self.pub = rospy.Publisher("/allpose",Float64MultiArray,queue_size=10)

	def call1(self,msg):
		self.ekf_x = msg.x
		self.ekf_y = msg.y
	def call2(self,msg):
		self.true_x = msg.pose.pose.position.x
		self.true_y = msg.pose.pose.position.y
	def call3(self,msg):
		self.pure_x = msg.pose.pose.position.x
		self.pure_y = msg.pose.pose.position.y

	def start(self):
		rospy.loginfo("start")
		while not rospy.is_shutdown():
			data = Float64MultiArray()
			#data.layout.dim.label = "truex , truey, purex, purey, ekfx, ekfy"
			#data.layout.dim.size = 4
			data.data = [self.true_x,self.true_y,self.pure_x,self.pure_y,self.ekf_x,self.ekf_y]
			self.pub.publish(data)
			self.loopr.sleep()

if __name__ == '__main__':
	rospy.init_node("Pose")
	nod = nodod()
	nod.start()