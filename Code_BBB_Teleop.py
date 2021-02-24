#Code_BBB_Teleop

import pickle
import socket
import time

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

def invkinematic(V,omega):
	r = 80 #mm
	L = 230 #mm
	omegaR=(V+omega*L)/r
	omegaL=(V-omega*L)/r
	return omegaR,omegaL

def motconL(omega):
	A=
	B=
	pwm = A*x+B
	if pwm>1:
		pwm = 1
	elif pwm <-1:
		pwm = -1
	return pwm

def motconR(omega):
	A=
	B=
	pwm = A*x+B
	if pwm > 1:
		pwm = 1
	elif pwm < -1:
		pwm = -1
	return pwm

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
			print(f'encoder count L = {tick1}, encoder count R = {tick2}')
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