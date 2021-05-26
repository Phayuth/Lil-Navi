#! /usr/bin/env python
import math
import tf
import rospy
from nav_msgs.msg import Odometry


def callb(msg):
	(r,p,y) = tf.transformations.euler_from_quaternion([msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w])
	# if y<0:
	# 	y = math.pi + (math.pi+y)
	rospy.loginfo("yaw = "+str(y))

def main():
	rospy.init_node("odom_view")
	rospy.Subscriber("/ground_truth/state",Odometry,callb) #/ground_truth/state
	rospy.spin()

if __name__ == '__main__':
	main()