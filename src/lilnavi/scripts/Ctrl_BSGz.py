#! /usr/bin/env python

#Backstepping Control For Gazebo simulation

import numpy as np
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose,Quaternion, Twist, Vector3
from tf.transformations import euler_from_quaternion, quaternion_from_euler

# Reference Pose===================================================================================

def ref_circle(t):
	freq   = 0.005
	radius = 3

	x     = radius*np.cos(freq*t)
	y     = radius*np.sin(freq*t)

	xdot  = -radius*freq*np.sin(freq*t)
	ydot  = radius*freq*np.cos(freq*t)
	theta_ref = np.arctan2(ydot, xdot)

	xddot = -radius*(freq**2)*np.cos(freq*t)
	yddot = -radius*(freq**2)*np.sin(freq*t)

	xdddot= radius*(freq**3)*np.sin(freq*t)
	ydddot= -radius*(freq**3)*np.cos(freq*t)

	vr    = np.sqrt(((xdot**2) + (ydot**2)))
	wr    = ((xdot*yddot)-(ydot*xddot))/((xdot**2) + (ydot**2))

	vdotr = (xdot*xddot+ydot*yddot)/vr
	wdotr = ((xdot*ydddot-ydot*xdddot)/(vr**2))-((2*wr*vdotr)/vr)

	return x,y,theta_ref,vr,wr,ydot,xdot,vdotr,wdotr

def ref_8(t):
	freq   = 0.002
	a = 2.5

	x     = a*np.sin(freq*t)
	y     = a*np.sin(2*freq*t)

	xdot  = a*freq*np.cos(freq*t)
	ydot  = 2*a*freq*np.cos(2*freq*t)
	theta_ref = np.arctan2(ydot, xdot)

	xddot = -a*(freq**2)*np.sin(freq*t)
	yddot = -4*a*(freq**2)*np.sin(2*freq*t)

	xdddot= -a*(freq**3)*np.cos(freq*t)
	ydddot= -8*a*(freq**3)*np.cos(2*freq*t)

	vr    = np.sqrt(((xdot**2) + (ydot**2)))
	wr    = ((xdot*yddot)-(ydot*xddot))/((xdot**2) + (ydot**2))

	vdotr = (xdot*xddot+ydot*yddot)/vr
	wdotr = ((xdot*ydddot-ydot*xdddot)/(vr**2))-((2*wr*vdotr)/vr)

	return x,y,theta_ref,vr,wr,ydot,xdot,vdotr,wdotr

#===================================================================================================
def angnorm(theta):

	if theta>0:
		if theta>np.pi:
			theta = (2*np.pi-theta)*(-1)

	if theta<0:
		if theta*(-1)>np.pi:
			theta = (2*np.pi-(-theta))*(-1)

	return theta

class backstp_contrl:

	def __init__(self,k1,k2,k3,ka,kb,m,r,b,Iner):
		self.k1   = k1
		self.k2   = k2
		self.k3   = k3
		self.ka   = ka
		self.kb   = kb
		self.m    = m
		self.r    = r
		self.b    = b
		self.Iner = Iner

	def error(self,qr,qc):
		"""Calculate error from the current pose to the reference pose. return qe"""
		theta = qc[2,0]

		T = np.array([[np.cos(theta),np.sin(theta),0],
		             [-np.sin(theta),np.cos(theta),0],
		             [      0       ,     0       ,1]]) # (3X3)

		e = qr - qc
		return T.dot(e) # (3X1)

	def controlkinematic(self,qe,vr,wr):
		""" Control Algorithm for Kinematic, return : vc, wc"""
		return ((vr*np.cos(qe[2,0]))+self.k1*qe[0,0]),(wr+(self.k2*vr*qe[1,0])*(self.k3*np.sin(qe[2,0])))

	def controldynamics(self,vdotref,wdotref):
		""" Control Algorithm for Dynamics, z1= vref-vcur , z2 = wref-wcur, return : tua1c, tua2c """
		return 1/2*((self.m*self.r*(vdotref+self.ka*z1))+((2*self.r*self.Iner/b)*(wdotref+self.kb*z2))),1/2*((self.m*self.r*(vdotref+self.ka*z1))-((2*self.r*self.Iner/b)*(wdotref+self.kb*z2)))

sq = 0
def odm_callback(msg):
	global sq
	# Find Current Pose
	x = msg.pose.pose.position.x
	y = msg.pose.pose.position.y
	qq = msg.pose.pose.orientation
	ol = [qq.x,qq.y,qq.z,qq.w]
	(rl,pt,yw)=euler_from_quaternion(ol)
	qc = np.array([[x],[y],[yw]])

	# Find Desired Pose
	xRef,yRef,theta_ref,vr,wr,ydot,xdot,vdotref,wdotref = ref_8(sq)
	qr = np.array([[xRef],[yRef],[theta_ref]])

	# Control init for each trajectory
	#contl = backstp_contrl(0.7,20,20,100,3000,4,0.1,0.26,2.5) # For ref_circle
	contl = backstp_contrl(0.7,20,20,100,3000,4,0.1,0.26,2.5) # For ref_8

	# Find error and control
	qe = contl.error(qr,qc)
	vc,wc = contl.controlkinematic(qe,vr,wr)

	# Print some output
	rospy.loginfo("thetaref = "+str(theta_ref)+"       thetac = "+str(qc[2,0]))

	# Publish to ROS
	tw_p = rospy.Publisher("/robotros_test/cmd_vel", Twist, queue_size = 50)
	Twm = Twist()
	Twm.linear.x = vc
	Twm.angular.z = wc
	tw_p.publish(Twm)
	sq += 1

def main():

	rospy.init_node('BS_Ctrl')
	rospy.Subscriber('/ground_truth/state',Odometry, odm_callback)
	rospy.spin()

if __name__=='__main__':
	main()