# Inbound take Velocity command from ROS to move the robot
# Input is V(m/s) and Omega(rad/s)
# Output is PWM to move Robot

import pickle
import socket
import time
import math

# Robot Parameter
#
#       ______________
#       |            |
# ||    |            |     ||
# ||----|            |-----|| r = wheel radius (m)
# ||    |            |     ||
#       |____________|
#
# <------------------------>
#   L = Robot base (m)

# import rcpy
import rcpy
import rcpy.motor as motor

# udp enable and binding
ip,pt = '192.168.0.104',50505
sock = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
sock.bind((ip,pt))
#sock.settimeout(1)

# rc
rcpy.set_state(rcpy.RUNNING)                #set rcpy state to running
mL,mR = motor.motor2,motor.motor3           #set motor L and R to 2 and 3

def invkinematic(V,omega):
	r = 0.065 #m  r = 65mm
	L = 0.200 #m  L = 200mm
	omegaR=(2*V+omega*L)/(2*r)
	omegaL=(2*V-omega*L)/(2*r)
	return omegaR,omegaL

def motconL(omega):

	""" Convert from desired wheel velocity (rad/s) to PWM value; range from [0 rad/s , 3.22 rad/s]
	From curve fit : PWM = 0.0002518*(tick/s)+0.04144
	We have : Omega_gear = (2pi/PPR/GR)*(tick/Ts)
	Thus :  => (tick/Ts) = (Omega_gear*PPR*GR/2*pi)
	We get :         PWM = A * ((Omega*PPR*GR)/(2*math.pi)) + B """

	# From Curve fit of the Left motor y = 0.00023*x+0.04505
	GR = 170
	PPR = 44
	A = 0.00023
	B = 0.04505
	pwm = A * ((Omega*PPR*GR)/(2*math.pi)) + B
	if pwm>1:
		pwm = 1
	elif pwm <-1:
		pwm = -1
	return pwm

def motconR(omega):
	# From Curve fit of the Right motor y = 0.00022*x+0.06191
	GR = 170
	PPR = 44
	A = 0.00022
	B = 0.06191
	pwm = A * ((Omega*PPR*GR)/(2*math.pi)) + B
	if pwm>1:
		pwm = 1
	elif pwm <-1:
		pwm = -1
	return pwm

def udpdta():
	data , addr = sock.recvfrom(2048)
	dataload = pickle.loads(data)
	V = dataload[0]
	omega = dataload[1]
	return V,omega

print("=====> Sever is Started and Listening <=====")
try:
	while rcpy.get_state() != rcpy.EXITING:
		if rcpy.get_state() == rcpy.RUNNING:
			V,omega = udpdta()
			omegaR,omegaL = invkinematic(V,omega)
			Rpwm = motconR(omegaR)
			Lpwm = motconL(omegaL)
			mR.set(Rpwm)
			mL.set(Lpwm)
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