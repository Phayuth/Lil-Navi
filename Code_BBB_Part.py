# Beaglebone Blue Part
# Inbound data : V, omega ---> Inverse Kinematic ---> omegaR,omegaL ---> PID ---> Motor Control
# Outbound data: odometry (X,Y,theta), IMU data <--- Forward Kinematic <--- Encoder, IMU
# import python Library

import pickle
import socket
import time
import numpy as np
#Robot Parameter
#
#		______________
#		|			 |
# ||	|			 |	   ||
# ||----|			 |-----|| r = wheel raduis
# ||	|			 |     ||
# 		|____________|
#
# <------------------------>
#   L = Robot base
r = 0.5 #mm
L = 0.5 #mm

# import rcpy
import rcpy
import rcpy.motor as motor
import rcpy.encoder as encoder

# udp enable and binding
ip,pt = '192.168.0.104',50505
ipd,ptd = '192.168.0.105',50506
sock = socket.socket(socket.AF_INET,SOCK_DGRAM)
sock.bind((ip,pt))
#sock.settimeout(1)

# rc
rcpy.set_state(rcpy.RUNNING)                #set rcpy state to running
mL,mR = motor.motor2,motor.motor3           #set motor L and R to 2 and 3
eL,eR = encoder.encoder2,encoder.encoder3   #set encoder L and R to 2 and 3

def invkinematic(V,omega)
	r = 80 #mm
	L = 230 #mm
	omegaR=(V+omega*L)/r
	omegaL=(V-omega*L)/r
	return omegaR,omegaL

def motconL(omega)
	A=
	B=
	pwm = A*x+B
	if pwm>1:
		pwm = 1
	elif pwm <-1:
		pwm = -1
	return pwm

def motconR(omega)
	A=
	B=
	pwm = A*x+B
	if pwm > 1:
		pwm = 1
	elif pwm < -1:
		pwm = -1
	return pwm

# Initial Pose
X = 0
Y = 0
Theta = math.radians(90)
Ts = 0.01
Mesb = [0,0,0]

# Forward Kinematic Internal
def FKI(Wr,Wl,rr,LB):
	velocity  = (rr*Wr/2) + (rr*Wl/2)
	angular_v = (rr*Wr/LB) - (rr*Wl/LB)
	return velocity,angular_v

# Forward Kinematic External
def FKE(velocity,angular_v,theta):
	m = np.array([[math.cos(theta),0],[math.sin(theta),0],[0,1]])
	n = np.array([[velocity],[angular_v]])
	o = m @ n # multiply matrix in numpy
	x_dot     = o[0]#math.cos(theta)*velocity    o[0]
	y_dot     = o[1]#math.sin(theta)*velocity    o[1]
	theta_dot = o[2]#angular_v                   o[2]
	return x_dot,y_dot,theta_dot

try:
    while rcpy.get_state() != rcpy.EXITING:
        if rcpy.get_state() == rcpy.RUNNING:
            data , addr = sock.recvfrom(2048)
            dataload = pickle.loads(data)
            V = dataload[0]
            omega = dataload[1]
            omegaR,omegaL = invkinematic(V,omega)
            Rpwm = motconR(omegaR)
            Lpwm = motconL(omegaL)
			mR.set(Rpwm)
			mL.set(Lpwm)
			tick1 = encoder.get()
			tick2 = encoder.get()
			Vfb,omegafb = FKI(eR,eL,r,L)
			x_dot,y_dot,theta_dot = FKE(Vfb,omegafb,Theta)
			X = X + x_dot*Ts
			Y = Y + y_dot*Ts
			Theta = Theta + theta_dot*Ts
			Mesb[0]=X
			Mesb[1]=Y
			Mesb[2]=Theta
			MSB = pickle.dumps(Mesb)
			sock.sendto(MSB,(ipd,ptd))
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