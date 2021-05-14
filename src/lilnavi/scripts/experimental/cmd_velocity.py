#! /usr/bin/env python

import numpy as np

import rospy
from geometry_msgs.msg import Twist


rospy.init_node("cmd_v")
pub = rospy.Publisher("/robotros_test/cmd_vel",Twist,queue_size=10)

twt = Twist()

while not rospy.is_shutdown():
	twt.linear.x = 1 + np.random.normal(0,1)
	twt.angular.z = 0.2 + np.random.normal(0,0.1)
	pub.publish(twt)
	rospy.sleep(0.05)