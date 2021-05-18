#! /usr/bin/env python

import numpy as np
import time

import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import tf
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, Pose2D


class EKF_node(object):
	def __init__(self):

		self.xest = np.array([[0],[0],[0]])
		self.pest = np.eye(3)
		self.z = np.array([[0],[0],[0]])

		self.loopr = rospy.Rate(20)

		rospy.Subscriber('/pose2D',Pose2D, self.read_lidr)

		self.pub = rospy.Publisher("/odom_filter", Odometry, queue_size = 50)

	def read_lidr(self,msg):
		yy = msg.theta
		if yy < 0:
			yy = np.pi + (np.pi+yy)
		self.z = np.array([[msg.x],[msg.y],[yy]])
		Ts = 0.05

		# input calculate
		noise = np.diag([1.,np.deg2rad(30.)])**2
		v = 1
		w = 0.5
		u = np.array([v,w]) #+ noise.dot(np.random.rand(2,1))

		# jacob F
		yaw = self.xest[2,0]
		jF = np.array([[1.,0,-v*np.sin(yaw)*Ts],[0,1.,v*np.cos(yaw)*Ts],[0,0,1.]])

		# prediction
		Q = np.diag([0.1,0.1,np.deg2rad(1.)])**2
		F = np.array([[1.,0,0],[0,1.,0],[0,0,1.]])
		B = np.array([[Ts*np.cos(self.xest[2,0]),0],[Ts*np.sin(self.xest[2,0]),0],[0,Ts]])
		xpred = F.dot(self.xest)+B.dot(u)
		ppred = jF.dot(self.pest.dot(jF.T)) + Q

		# update
		R = np.diag([0.1,0.1,0.1])**2
		y = self.z - xpred
		jH = np.diag([1.,1.,1.])
		s = (R + jH.dot(ppred.dot(jH.T)))
		K = ppred.dot((jH.T).dot(np.linalg.inv(s)))
		self.xest = xpred + K.dot(y)
		self.pest = (np.eye(3) - K.dot(jH)).dot(ppred)

if __name__ == '__main__':

	rospy.init_node('ekf_locl_fusion')
	nod = EKF_node()

	while not rospy.is_shutdown():
		odom_broadcaster = tf.TransformBroadcaster()
		odom = Odometry()
		odom_quat = tf.transformations.quaternion_from_euler(0,0,nod.xest[2,0])
		odom_broadcaster.sendTransform((nod.xest[0,0],nod.xest[1,0],0.),odom_quat,rospy.Time.now(),"base_link_filter","odom_filter")
		odom.header.frame_id = "filter_odom"
		odom.pose.pose = Pose(Point(nod.xest[0,0],nod.xest[1,0],0.),Quaternion(*odom_quat))
		odom.child_frame_id = "filter_base_link"
		nod.pub.publish(odom)
		nod.loopr.sleep()