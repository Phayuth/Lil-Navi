#! /usr/bin/env python
import math
import tf
import rospy
from sensor_msgs.msg import Imu


def callb(msg):
	(r,p,y) = tf.transformations.euler_from_quaternion([msg.orientation.x,msg.orientation.y,msg.orientation.z,msg.orientation.w])
	if y<0:
		y = math.pi + (math.pi+y)
	rospy.loginfo("yaw = "+str(y))

def main():
	rospy.init_node("imu_view")
	rospy.Subscriber("/robotros_test/imu",Imu,callb)
	rospy.spin()

if __name__ == '__main__':
	main()