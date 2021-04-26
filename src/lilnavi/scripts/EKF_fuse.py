#! /usr/bin/env python

import numpy as np
import time

import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import tf
from geometry_msgs.msg import Point, Pose,Quaternion, Twist, Vector3, Pose2D


# ROS Callback
def odom_callback(msg):
	global V_odom,Omg_odom
	V_odom = msg.twist.twist.linear.x
	Omg_odom = msg.twist.twist.angular.z

def lidar_callback(msg):
	global x_meas
	x_meas[0,0] = msg.x
	x_meas[1,0] = msg.y

def imu_callback(msg):
	global x_meas
	Ts = 0.01
	x_meas[2,0] = msg.angular_velocity.z*Ts


# EKF
def ekf_pre(x,p,V_odom,Omg_odom):
	Ts = 0.01
	Q = np.diag([0.1,0.1,0.1])
	JF = np.array([[1,0,-V_odom*np.sin(x[2,0])*Ts],[0,1,V_odom*np.cos(x[2,0])*Ts],[0,0,1]])
	x = x + np.array([[V_odom*np.cos(x[2,0])*Ts],[V_odom*np.sin(x[2,0]*Ts)],[Omg_odom*Ts]])
	p = JF.dot(p).dot(JF.T)+Q
	return x,p


def ekf_upd(x_meas,x,p):
	Ts = 0.01
	R = np.diag([0.1,0.1,0.1])
	JH = np.array([[1,0,0],[0,0,1],[0,0,Ts]])
	y = x_meas - x
	s = JH.dot(p).dot(JH.T) + R
	k = p.dot(JH.T).dot(np.linalg.inv(s))
	x = x + k.dot(y)
	p = (np.eye(3) - k.dot(JH)).dot(p)
	return x,p



x = np.array([[0],[0],[0]])
p = np.eye(3)
V_odom,Omg_odom=None,None
x_meas = np.array([[0],[0],[0]])

pub_odom_filter = rospy.Publisher("/odom_filter", Odometry, queue_size = 50)
odom_broadcaster = tf.TransformBroadcaster()
odom = Odometry()

if __name__ == '__main__':
	rospy.init_node('ekf_locl_fusion')
	rospy.Rate(10)
	rospy.Subscriber('/robotros_test/odom',Odometry,odom_callback)
	rospy.Subscriber('/pose2D',Pose2D,lidar_callback)
	rospy.Subscriber('/robotros_test/imu',Imu,imu_callback)
	current_time = rospy.Time.now()
	x,p = ekf_pre(x,p,V_odom,Omg_odom)
	x,p = ekf_upd(x_meas,x,p)


	odom_quat = tf.transformations.quaternion_from_euler(0,0,x[2,0])
	odom_broadcaster.sendTransform((x[0,0],x[1,0],0.),odom_quat,current_time,"base_link","odom")
	odom.header.frame_id = "/robotros_test/odom"
	odom.pose.pose = Pose(Point(x[0,0],x[1,0],0.),Quaternion(*odom_quat))
	odom.child_frame_id = "base_link"
	pub_odom_filter.publish(odom)

	rospy.spin()