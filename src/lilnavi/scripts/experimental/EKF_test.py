#! /usr/bin/env python

import numpy as np
import time

import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import tf
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, Pose2D


class EKF_node(object):
	def __init__(self,x,p):

		# Value store

		# encoder
		self.V_odom = 0
		self.w_odom = 0

		# measure
		self.x_lidr = 0
		self.y_lidr = 0
		self.t_imu  = 0

		self.prev_x = x
		self.prev_p = p
		self.pred_x = self.prev_x
		self.pred_p = self.prev_p
		self.est_x  = self.prev_x
		self.est_p  = self.prev_p

		# Loop rate
		self.loopr = rospy.Rate(20)

		# Subscribe
		rospy.Subscriber('/robotros_test/odom',Odometry, self.read_odom)
		rospy.Subscriber('/pose2D',Pose2D, self.read_lidr)
		rospy.Subscriber('/robotros_test/imu',Imu, self.read_imu)

		# Publish
		self.pub = rospy.Publisher("/odom_filter", Odometry, queue_size = 50)

	def read_odom(self,msg):
		self.V_odom = msg.twist.twist.linear.x + np.random.normal(0,0.1) # noise std = 0.1
		self.w_odom = msg.twist.twist.angular.z + np.random.normal(0,0.1)# noise std = 0.1

	def read_lidr(self,msg):
		self.x_lidr = msg.x
		self.y_lidr = msg.y

	def read_imu(self,msg):
		(r,p,y) = tf.transformations.euler_from_quaternion([msg.orientation.x,msg.orientation.y,msg.orientation.z,msg.orientation.w])
		# if y<0:
		# 	y = np.pi + (np.pi+y)
		self.t_imu = y

	def ekf_pre(self,V_odom,w_odom):
		Ts = 0.02
		Q = np.diag([0.1,0.1,0.1])**2
		JF = np.array([[1,0,-V_odom*np.sin(self.prev_x[2,0])*Ts],
			           [0,1, V_odom*np.cos(self.prev_x[2,0])*Ts],
			           [0,0,1]])
		self.pred_x = self.prev_x + np.array([[V_odom*np.cos(self.prev_x[2,0])*Ts],
			                                  [V_odom*np.sin(self.prev_x[2,0]*Ts)],
			                                  [w_odom*Ts]])
		self.pred_p = JF.dot(self.prev_p).dot(JF.T)+Q

	def ekf_upd(self,x_meas):
		Ts = 0.02
		R = np.diag([0.1,0.1,0.1])**2
		JH = np.array([[1,0,0],[0,0,1],[0,0,1]])
		y = x_meas - self.pred_x
		s = JH.dot(self.pred_p).dot(JH.T) + R
		k = self.pred_p.dot(JH.T).dot(np.linalg.inv(s))
		self.est_x = self.pred_x + k.dot(y)
		self.est_p = (np.eye(3) - k.dot(JH)).dot(self.pred_p)
		self.prev_x = self.est_x
		self.prev_p = self.est_p


if __name__ == '__main__':

	rospy.init_node('ekf_locl_fusion')
	nod = EKF_node(np.array([[0],[0],[0]]),np.eye(3))

	while not rospy.is_shutdown():
		nod.ekf_pre(nod.V_odom,nod.w_odom)
		meas = np.array([[nod.x_lidr],[nod.y_lidr],[nod.t_imu]])
		nod.ekf_upd(meas)
		odom_broadcaster = tf.TransformBroadcaster()
		odom = Odometry()
		odom_quat = tf.transformations.quaternion_from_euler(0,0,nod.est_x[2,0])
		odom_broadcaster.sendTransform((nod.est_x[0,0],nod.est_x[1,0],0.),odom_quat,rospy.Time.now(),"base_link_filter","odom_filter")
		odom.header.frame_id = "filter_odom"
		odom.pose.pose = Pose(Point(nod.est_x[0,0],nod.est_x[1,0],0.),Quaternion(*odom_quat))
		odom.child_frame_id = "filter_base_link"
		nod.pub.publish(odom)
		nod.loopr.sleep()