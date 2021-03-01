import time
import math
#import numpy as np
import rcpy
import rcpy.encoder as encoder

#Robot Parameter
#
#       ______________
#       |            |
# ||    |            |     ||
# ||----|            |-----|| r = wheel raduis
# ||    |            |     ||
#       |____________|
#
# <------------------------>
#   L = Robot base

rcpy.set_state(rcpy.RUNNING)                #set rcpy state to running
eL,eR = encoder.encoder2,encoder.encoder3   #set encoder L and R to 2 and 3

def thetawheel(tick): # find theta wheel after gear ration, given the encoder tick count
    PPR = 44
    GR  = 170
    theta_w = (2*math.pi*tick)/(PPR*GR) # rad
    return theta_w
    
# Forward Kinematic Internal
def FKI(Wr,Wl):
	r = 0.065 #m   r= 65mm
	b = 0.200 #m   b= 200mm 
	velocity  = (r*Wr/2) + (r*Wl/2)
	angular_v = (r*Wr/b) - (r*Wl/b)
	return velocity,angular_v

# Forward Kinematic External
def FKE(velocity,angular_v,theta):
	#m = np.array([[math.cos(theta),0],[math.sin(theta),0],[0,1]])
	#n = np.array([[velocity],[angular_v]])
	#o = m @ n # multiply matrix in numpy
	x_dot     = math.cos(theta)*velocity    #o[0]
	y_dot     = math.sin(theta)*velocity    #o[1]
	theta_dot = angular_v                   #o[2]
	return x_dot,y_dot,theta_dot
	
# Initial Pose
X = 0
Y = 0
Theta = math.radians(0)
Ts = 0.01
tick1_O = 0
tick2_O = 0

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
			
			# Find Velocitry in 2D space
			x_dot,y_dot,theta_dot = FKE(V,omega,Theta)
			
			# Odometry
			X = X + x_dot*Ts
			Y = Y + y_dot*Ts
			Theta = Theta + theta_dot*Ts
			print(f'X={X},Y={Y},Theta={Theta},e1={tick1},e2={tick2},eer={tick1-tick2}')
			
			# Update For Next Time Step
			tick1_O = tick1
			tick2_O = tick2
			time.sleep(Ts)
		
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