# Inbound For Beaglebone Blue
# Input data : V, omega
# Output data: Rwheel omega , Lwheel omega
# import python Library
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

# udp enable and binding
ip = '192.168.0.104'
pt = 50505
sock = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
sock.bind((ip,pt))
#sock.settimeout(1)

# rc
rcpy.set_state(rcpy.RUNNING)                #set rcpy state to running
mL,mR = motor.motor2,motor.motor3           #set motor L and R to 2 and 3

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
	return pwm

def motconR(omega)
	A=
	B=
	pwm = A*x+B
	return pwm

try:
    while rcpy.get_state() != rcpy.EXITING:
        if rcpy.get_state() == rcpy.RUNNING:
            udpdata , addr = sock.recvfrom(2048)
            V,omega = pickle.loads(udpdata)
            omegaR,omegaL = invkinematic(V,omega)
            Rpwm = motconR(omegaR)
            Lpwm = motconL(omegaL)
			mR.set(Rpwm)
			mL.set(Lpwm)
			sleep(0.01)
        elif rcpy.get_state() == rcpy.PAUSED:
            mL.free_spin(),mR.free_spin()
        else:
            while rcpy.get_state() != rcpy.EXITING:
                sleep(.5)
except KeyboardInterrupt:
    print("Done Recived and Control")
    rcpy.set_state(rcpy.EXITING)

finally:
    print("Buh Bye")