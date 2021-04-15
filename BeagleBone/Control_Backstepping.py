# Backstepping_Control
import math
import numpy as np

# rcpy lib
import rcpy
import rcpy.motor as motor
import rcpy.encoder as encoder
import rcpy.mpu9250 as mpu9250
import numpy as np

# rc
rcpy.set_state(rcpy.RUNNING)                #set rcpy state to running
mL,mR = motor.motor2,motor.motor3           #set motor L and R to 2 and 3
eL,eR = encoder.encoder2,encoder.encoder3   #set encoder L and R to 2 and 3

# Reference Pose===================================================================================
def ref_cicle(t):
	freq   = 2*math.pi/30
	radius = 5

	x     = radius*math.cos(freq*t)
	y     = radius*math.sin(freq*t)

	xdot  = -radius*freq*math.sin(freq*t)
	ydot  = radius*freq*math.cos(freq*t)

	xddot = -radius*(freq**2)*math.cos(freq*t)
	yddot = -radius*(freq**2)*math.sin(freq*t)

	xdddot= radius*(freq**3)*math.sin(freq*t)
	ydddot= -radius*(freq**3)*math.cos(freq*t)

	vr    = math.sqrt((xdot**2 + ydot**2))
	wr    = ((xdot*yddot-ydot*xddot))/((xdot**2 + ydot**2))

	vdotr = (xdot*xddot+ydot*yddot)/vr
	wdotr = ((xdot*ydddot-ydot*xdddot)/(vr**2))-((2*wr*vdotr)/vr)

	return x,y,vr,wr,ydot,xdot,vdotr,wdotr

def ref_sin_45(t):
	freq   = 2*math.pi/30
	radius = 5

	x     = (math.cos(math.radians(45))*t)-(math.sin(math.radians(45))*radius*math.sin(freq*t))
	y     = (math.sin(math.radians(45))*t)+(math.cos(math.radians(45))*radius*math.sin(freq*t))

	xdot  = math.cos(math.radians(45))-(math.sin(math.radians(45))*radius*freq*math.cos(freq*t))
	ydot  = math.sin(math.radians(45))+(math.cos(math.radians(45))*radius*freq*math.cos(freq*t))

	xddot = math.sin(math.radians(45))*radius*(freq**2)*math.sin(freq*t)
	yddot = -math.cos(math.radians(45))*radius*(freq**2)*math.sin(freq*t)

	xdddot= math.sin(math.radians(45))*radius*(freq**3)*math.cos(freq*t)
	ydddot= -math.cos(math.radians(45))*radius*(freq**3)*math.cos(freq*t)

	vr    = math.sqrt((xdot**2 + ydot**2))
	wr    = ((xdot*yddot-ydot*xddot))/((xdot**2 + ydot**2))

	vdotr = (xdot*xddot+ydot*yddot)/vr
	wdotr = ((xdot*ydddot-ydot*xdddot)/(vr**2))-((2*wr*vdotr)/vr)

	return x,y,vr,wr,ydot,xdot,vdotr,wdotr
#===================================================================================================

class ddr_lil_robot:

	def __init__(self,GR,PPR,r,b):
		self.GR  = GR
		self.PPR = PPR
		self.r   = r
		self.b   = b

	def thetawheel(tick):
		""""find theta wheel after gear ratio, given the encoder tick count, return: theta_w"""
		return (2*math.pi*tick)/(PPR*GR) # rad

	def FKI(Wr,Wl):# Forward Kinematic Internal
		"""Forward Kinematic, Determine V and Omega, given each wheel, return: velocity, angular_v"""
		return ((r*Wr/2) + (r*Wl/2)),((r*Wr/b) - (r*Wl/b))

	def FKE(velocity,angular_v,theta):
		"""Determine Velocity in 2D space, return x_dot, y_dot, theta_dot"""
		return (math.cos(theta)*velocity),(math.sin(theta)*velocity),angular_v

	def invkinematic(V,omega):
		"""Inverse Kinematic for determine each wheel speed, return omegaR,omegaL"""
		return (2*V+omega*b)/(2*r),(2*V-omega*b)/(2*r)

	def motcon(omega,A,B):
		"""determine PWM value from controller, return pwm"""
		pwm = A * ((Omega*PPR*GR)/(2*math.pi)) + B
		if pwm>1:
			pwm = 1
		elif pwm <-1:
			pwm = -1
		return pwm

class backstp_contrl:

	def __init__(self,k1,k2,k3,ka,kb,mass,r,b,Inertial):
		self.k1   = k1
		self.k2   = k2
		self.k3   = k3
		self.ka   = ka
		self.kb   = kb
		self.mass = mass
		self.r    = r
		self.b    = b
		self.Inertial = Inertial

	def error(qr,qc):
		"""Calculate error from the current pose to the reference pose. return qe"""
		theta = qc[2,0]

		T = np.array([[math.cos(theta),math.sin(theta),0],
		             [-math.sin(theta),math.cos(theta),0],
		             [      0         ,     0         ,1]]) # (3X3)

		return T @ (qr - qc) # (3X1)

	def controlkinematic(qe,vr,wr):
		""" Control Algorithm for Kinematic, return : vc, wc"""
		return vr*math.cos(qe[2,0])+k1*qe[0,0],wr+k2*vr*qe[1,0]*k3*math.sin(qe[2,0])

	def controldynamics(vdotref,wdotref):
		""" Control Algorithm for Dynamics, z1= vref-vcur , z2 = wref-wcur, return : tua1c, tua2c """
		return 1/2*((mass*r*(vdotref+ka*z1))+((2*r*Inertial/b)*(wdotref+kb*z2))),...
				1/2*((mass*r*(vdotref+ka*z1))-((2*r*Inertial/b)*(wdotref+kb*z2)))

# Initial Pose
qc = np.array([[0],[0],[0]])
Ts = 0.01
tick1_O = 0
tick2_O = 0

robot = ddr_lil_robot(170,44,0.065,0.200) #GR = 170 PPR = 44 r = 0.065m b = 0.200m
contl = backstp_contrl(10,5,4,100,3000,4,0.065,0.2,2.5)#k1 = 10 k2 = 5 k3 = 4 ka = 100 kb = 3000 need tuning
                                                       #mass = 4 kg, radius = 0.03 m, length   = 0.3 m, Inertial = 2.5 kg*m^2

try:
	while rcpy.get_state() != rcpy.EXITING:
		if rcpy.get_state() == rcpy.RUNNING:

			# Control
			xRef,yRef,vr,wr,ydot,xdot,vdotref,wdotref = ref_cicle(Ts)
			theta_ref = math.atan2(ydot, xdot)
			qr = np.array([[xRef],[yRef],[theta_ref]])
			qe = contl.error(qr,qc)
			vc,wc = contl.controlkinematic(qe,vr,wr)

			# Control Motor Here
			omegaR,omegaL = robot.invkinematic(vc,wc)
			Lpwm = robot.motcon(omegaL,0.00023,0.04505) # Motor Left GR = 170, PPR = 44 , A = 0.00023, B = 0.04505
			Rpwm = robot.motcon(omegaR,0.00022,0.06191) # Motor RightGR = 170, PPR = 44 , A = 0.00022, B = 0.06191
			mR.set(Rpwm)
			mL.set(Lpwm)

			# Feedback
			# Encoder
			tick1 = (-1*eL.get()) # To get positive number foward
			tick2 = eR.get()

			# Find OmegaL and OmegaR
			Wl = (thetawheel(tick1) - thetawheel(tick1_O))/Ts
			Wr = (thetawheel(tick2) - thetawheel(tick2_O))/Ts

			# Find V and Omega
			V,omega = robot.FKI(Wr,Wl)

			# Find Velocity in 2D space
			x_dot,y_dot,theta_dot = robot.FKE(V,omega,qc[2,0])

			# Odometry Update
			qc = np.array([[(qc[0,0] + x_dot*Ts)],[(qc[1,0] + y_dot*Ts)],[(qc[2,0] + theta_dot*Ts)]])

			# Update
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