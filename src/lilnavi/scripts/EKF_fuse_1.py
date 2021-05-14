#! /usr/bin/env python

import numpy as np
import time

import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import tf
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, Pose2D

# covairance

# Q = np.diag([0.1,0.1,np.deg2rad(1.)])**2
# R = np.diag([0.1,0.1,0.1])**2

def input():
	noise = np.diag([1.,np.deg2rad(30.)])**2
	v = 1
	w = 0.5
	u = np.array([v,w])+ noise.dot(np.random.rand(2,1))
	return u

# def motion_model(x,u):
# 	Ts = 0.05
# 	F = np.array([[1.,0,0],[0,1.,0],[0,0,1.]])
# 	B = np.array([[Ts*np.cos(x[2,0]),0],[Ts*np.sin(x[2,0]),0],[0,Ts]])
# 	x = F.dot(x)+B.dot(u)

def jacobF(x,u):
	Ts = 0.05
	yaw = x[2,0]
	v = u[0,0]
	jF = np.array([[1.,0,-v*np.sin(yaw)*Ts],[0,1.,v*np.cos(yaw)*Ts],[0,0,1.]])
	return jF

def jacobH():
	jH = np.diag([1.,1.,1.])
	return jH

def ekf(xest,pest,z,u):
	# pred
	Ts = 0.05
	Q = np.diag([0.1,0.1,np.deg2rad(1.)])**2
	#xpred = motion_model(xest,u)
	F = np.array([[1.,0,0],[0,1.,0],[0,0,1.]])
	B = np.array([[Ts*np.cos(xest[2,0]),0],[Ts*np.sin(xest[2,0]),0],[0,Ts]])
	xpred = F.dot(xest)+B.dot(u)
	jF = jacobF(xest,u)
	ppred = jF.dot(pest.dot(jF.T)) + Q

	# update
	R = np.diag([0.1,0.1,0.1])**2
	y = z - xpred
	jH = jacobH()
	s = (R + jH.dot(ppred.dot(jH.T)))
	K = ppred.dot((jH.T).dot(np.linalg.inv(s)))
	xest = xpred + K.dot(y)
	pest = (np.eye(3) - K.dot(jH)).dot(ppred)

	return xest,pest

def observer(msg,(xest,pest)):
	u = input()
	z = np.array([[msg.x],[msg.y],[msg.theta]])
	xest,pest = ekf(xest,pest,z,u)
	rospy.loginfo("XEST = "+str(xest)+"  ,  "+"Pest = "+str(pest))

if __name__ == '__main__':
	rospy.init_node("EKF_filter")
	xest = np.zeros([3,1])
	pest = np.eye(3)

	while not rospy.is_shutdown():
		rospy.Subscriber("/pose2D",Pose2D,observer,(xest,pest))