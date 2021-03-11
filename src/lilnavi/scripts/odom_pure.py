#! /usr/bin/env python

import socket
import pickle
import pickle_compat
import math
import time
import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose,Quaternion, Twist, Vector3
from std_msgs.msg import Float64
# Patch Pickle
pickle_compat.patch()

# UDP bind
ip = "192.168.0.105"
pt = 50505
sock = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
sock.bind((ip,pt))

# ROS
rospy.init_node('odometry_publisher')
odom_pub = rospy.Publisher("/odom", Odometry, queue_size = 50)
thetaL_pub = rospy.Publisher("/thetaL", Float64, queue_size = 50)
thetaR_pub = rospy.Publisher("/thetaR", Float64, queue_size = 50)
odom_broadcaster = tf.TransformBroadcaster()

def udprec():
	data,addr = sock.recvfrom(1024)
	load = pickle.loads(data)
	x = load[0]
	y = load[1]
	theta = load[2]
	vx = load[3]
	vy = load[4]
	omega = load[5]
	thetar = load[6]
	thetal = load[7]
	return x,y,theta,vx,vy,omega,thetar,thetal

odom = Odometry()
theR = Float64()
theL = Float64()

current_time = rospy.Time.now()
last_time = rospy.Time.now()
#r = rospy.Rate(10)

print("Started Listening")
while not rospy.is_shutdown():
	current_time = rospy.Time.now()
	try:
		x,y,theta,vx,vy,omega,thetar,thetal = udprec()
	except:
		pass
	# ros require orientation in quaternion form, so transform yaw angle to quaternion using below function
	odom_quat = tf.transformations.quaternion_from_euler(0,0,theta)

	# first , we pub the transform over tf
	odom_broadcaster.sendTransform((x,y,0.),odom_quat,current_time,"base_link","odom") #pose x y,(no z),orien of quat,time,frame_id,frame_id)

	# next , we pub odom message
	# odom = Odometry()
	odom.header.stamp = current_time
	odom.header.frame_id = "odom"

	# set pose
	odom.pose.pose = Pose(Point(x,y,0.),Quaternion(*odom_quat))

	#set velo
	odom.child_frame_id = "base_link"
	odom.twist.twist = Twist(Vector3(vx,vy,0),Vector3(0,0,omega))

	# pub
	odom_pub.publish(odom)
	thetaR_pub.publish(theR)
	thetaL_pub.publish(theL)
	last_time=current_time
	#r.sleep()
	#time.sleep(0.01)