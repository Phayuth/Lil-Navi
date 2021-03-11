#! /usr/bin/env python

import socket
import pickle
import pickle_compat
import time
import rospy
import tf
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion, Vector3

# Patch Pickle
pickle_compat.patch()

# UDP bind
ip = "192.168.0.105"
pt = 50505
sock = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
sock.bind((ip,pt))

# ROS
rospy.init_node('IMU_publisher')
imu_pub = rospy.Publisher("/imu_data", Imu, queue_size = 50)


def udprec():
	data,addr = sock.recvfrom(1024)
	load = pickle.loads(data)
	qx = load[0]
	qy = load[1]
	qz = load[2]
	qw = load[3]
	Omex = load[4]
	Omey = load[5]
	ax   = load[6]
	ay   = load[7]
	return qx,qy,qz,qw,Omex,Omey,ax,ay

current_time = rospy.Time.now()
last_time = rospy.Time.now()

imu = Imu()

print("Started Listening")
while not rospy.is_shutdown():
	current_time = rospy.Time.now()
	try:
		qx,qy,qz,qw,Omex,Omey,ax,ay = udprec()
	except:
		pass
	# IMU data
	# imu = Imu()
	imu.header.stamp = current_time
	imu.header.frame_id = "imu_frame"
	imu.orientation.x = qx
	imu.orientation.y = qy
	imu.orientation.z = qz
	imu.orientation.w = qw
	imu.angular_velocity = Vector3(Omex,Omey,0)
	imu.linear_acceleration = Vector3(ax,ay,0)

	# Pub
	imu_pub.publish(imu)

	# Update
	last_time=current_time
	time.sleep(0.01)