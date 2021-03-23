# Outbound Odometry to ROS  X Y Theta
# Input is omegaR(tick) and omegaL(tick)
# Output is X Y Theta

import pickle
import socket
import time
import math

# Robot Parameter
#
#       ______________
#       |            |
# ||    |            |     ||
# ||----|            |-----|| r = wheel raduis (m)
# ||    |            |     ||
#       |____________|
#
# <------------------------>
#   L = Robot base (m)

# import rcpy
import rcpy
import rcpy.encoder as encoder

# rc
rcpy.set_state(rcpy.RUNNING)                #set rcpy state to running
eL,eR = encoder.encoder2,encoder.encoder3   #set encoder L and R to 2 and 3

# Theta Wheel
def thetawheel(tick):
	""""find theta wheel after gear ratio, given the encoder tick count"""
	PPR = 44
	GR  = 170
	theta_w = (2*math.pi*tick)/(PPR*GR) # rad
	return theta_w

# Forward Kinematic Internal
def FKI(Wr,Wl):
	"""Forward Kinematic, Determine V and Omega, given each wheel """
	r = 0.065 #m r= 65mm
	b = 0.200 #m b= 200mm
	velocity  = (r*Wr/2) + (r*Wl/2)
	angular_v = (r*Wr/b) - (r*Wl/b)
	return velocity,angular_v

# Forward Kinematic External
def FKE(velocity,angular_v,theta):
	"""Determine Velocity in 2D space"""
	x_dot     = math.cos(theta)*velocity
	y_dot     = math.sin(theta)*velocity
	theta_dot = angular_v
	return x_dot,y_dot,theta_dot

def udpsend(msg):
	ipd  = "192.168.0.105"
	ptd  = 50505
	sock = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
	dmsg = pickle.dumps(msg,2) # sent to protocol 2 pickle
	sock.sendto(dmsg,(ipd,ptd))

# Initial Pose
X = 0
Y = 0
Theta = math.radians(0)
Ts = 0.01
tick1_O = 0
tick2_O = 0

Msgs = [0,0,0,0,0,0,0,0]
print("=====> Outbound Odom data is Sending <=====")
try:
	while rcpy.get_state() != rcpy.EXITING:
		if rcpy.get_state() == rcpy.RUNNING:
			# Encoder
			tick1 = (-1*eL.get()) # To get positive number foward
			tick2 = eR.get()

			# Find OmegaL and OmegaR
			Wl = (thetawheel(tick1) - thetawheel(tick1_O))/Ts
			Wr = (thetawheel(tick2) - thetawheel(tick2_O))/Ts

			# Find V and Omega
			V,omega = FKI(Wr,Wl)

			# Find Velocity in 2D space
			x_dot,y_dot,theta_dot = FKE(V,omega,Theta)

			# Odometry
			X = X + x_dot*Ts
			Y = Y + y_dot*Ts
			Theta = Theta + theta_dot*Ts

			# Combine data to send
			Msgs[0] = X
			Msgs[1] = Y
			Msgs[2] = Theta
			Msgs[3] = x_dot
			Msgs[4] = y_dot
			Msgs[5] = theta_dot
			Msgs[6] = thetawheel(tick1)
			Msgs[7] = thetawheel(tick2)

			# Send data
			udpsend(Msgs)

			# Update
			tick1_O = tick1
			tick2_O = tick2
			time.sleep(0.01)

		elif rcpy.get_state() == rcpy.PAUSED:
			mL.free_spin(),mR.free_spin()

		else:
			while rcpy.get_state() != rcpy.EXITING:
				time.sleep(.5)

except KeyboardInterrupt:
	print("Done Recived and Control")
	rcpy.set_state(rcpy.EXITING)

finally:
	print("Buh Bye")