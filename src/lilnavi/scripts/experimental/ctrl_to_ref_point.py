#! /usr/bin/env python

import numpy as np
import tf
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

def callback(msg):
	# Reference point
	xref = 2
	yref = 3
	tref = np.arctan2(yref,xref)

	# Current pose of robot
	xcur = msg.pose.pose.position.x
	ycur = msg.pose.pose.position.y
	(r,p,y) = tf.transformations.euler_from_quaternion([msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w])

	# Find error between ref point and current point
	delx = xref - xcur
	dely = yref - ycur
	delt = tref - y

	#control
	k1 = 4
	k2 = 0.1
	wc = k1*np.arctan(np.tan(delt))
	vc = k2*(np.sqrt((delx**2) + (dely**2)))*np.sign(np.cos(delt))

	# when the speed too small -> really on the reference point -> stop the robot from moving
	if vc < 0.0025:
		vc = 0

	# publish to command robot
	tw_p = rospy.Publisher("/robotros_test/cmd_vel", Twist, queue_size = 50)
	Twm = Twist()
	Twm.linear.x = vc
	Twm.angular.z = wc
	tw_p.publish(Twm)
	rospy.loginfo("delx = "+str(delx)+" , "+"dely = "+str(dely)+" , "+"delt = "+str(delt))

def main():
	rospy.init_node("watch_pose")
	rospy.Subscriber("/ground_truth/state",Odometry,callback)
	rospy.spin()


if __name__ == '__main__':
	main()