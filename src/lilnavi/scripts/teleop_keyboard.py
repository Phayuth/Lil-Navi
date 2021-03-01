#! /usr/bin/env python
# Import Lib
import rospy
import socket
import pickle
import pickle_compat

from geometry_msgs.msg import Twist

# Patch Pickle
pickle_compat.patch()

# IP
ip = '192.168.0.104'
pt = 50505
sock = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)

# Message V, Omega
Ms = [0,0]

def twist_callback(msg):
	V = msg.linear.x
	omg = msg.angular.z
	Ms[0]=V
	Ms[1]=omg
	Mse = pickle.dumps(Ms)
	sock.sendto(Mse,(ip,pt))
	#rospy.loginfo(rospy.get_caller_id() + "V"+str(V)+" , "+"Om"+str(omg))

def teleop():
	rospy.init_node('BBB_UDP_Teleop')
	sub_twt = rospy.Subscriber('/cmd_vel',Twist, twist_callback)
	rospy.spin()

if __name__=='__main__':
	teleop()
