import numpy as np
import time
import matplotlib.pyplot as plt
import matplotlib
from drawnow import *
import math
import random
matplotlib.rc('font',family='Times New Roman')

def ref_8(t):
	freq   = 2*math.pi/30
	a      = 5
	x_ref  = a*np.sin(freq*t)
	y_ref  = a*np.sin(2*freq*t)
	xdot   = a*freq*np.cos(freq*t)
	ydot   = 2*a*freq*np.cos(2*freq*t)
	t_ref  = np.arctan2(ydot, xdot)
	xddot  = -a*(freq**2)*np.sin(freq*t)
	yddot  = -4*a*(freq**2)*np.sin(2*freq*t)
	xdddot = -a*(freq**3)*np.cos(freq*t)
	ydddot = -8*a*(freq**3)*np.cos(2*freq*t)
	v_ref  = np.sqrt(((xdot**2) + (ydot**2)))
	w_ref  = ((xdot*yddot)-(ydot*xddot))/((xdot**2) + (ydot**2))
	vdotr  = (xdot*xddot+ydot*yddot)/v_ref
	wdotr  = ((xdot*ydddot-ydot*xdddot)/(v_ref**2))-((2*w_ref*vdotr)/v_ref)

	return x_ref,y_ref,t_ref,v_ref,w_ref,vdotr,wdotr

def pose_error(qr,qc):
	theta = qc[2,0]
	T     = np.array([[math.cos(theta),math.sin(theta),0],
	                 [-math.sin(theta),math.cos(theta),0],
	                 [      0         ,     0         ,1]]) # (3X3)
	qe    = T @ (qr - qc) # (3X1)

	return qe


def controlkinematic(qe,vr,wr):
	k1 = 5 # need tuning defualt 10
	k2 = 10 # need tuning defualt 5
	k3 = 10 # need tuning defualt 4

	vc = vr*math.cos(qe[2,0])+k1*qe[0,0]
	wc = wr+k2*vr*qe[1,0]*k3*math.sin(qe[2,0])

	return vc,wc

# Covariance for EKF simulation
Q = np.diag([0.1,0.1,np.deg2rad(1.0)])**2 # variance of location on x,y,and yaw angle
R = np.diag([1.0, 1.0, 1.0])**2                # Observation x,y position covariance

#  Simulation parameter
INPUT_NOISE = np.diag([1.0, np.deg2rad(30.0)]) ** 2
AMEAS_NOISE = np.diag([0.2, 0.2, 0.2]) ** 2
DT = 0.033

def calc_input(vc,wc):
	u = np.array([[vc],[wc]])
	return u


def observation(xTrue, xd, u):
	xTrue = motion_model(xTrue, u)

	# add noise to gps x-y
	z = observation_model(xTrue) + AMEAS_NOISE @ np.random.randn(3, 1)

	# add noise to input
	ud = u + INPUT_NOISE @ np.random.randn(2, 1)

	xd = motion_model(xd, ud)

	return xTrue, z, xd, ud


def motion_model(x, u):
	F = np.array([[1.0, 0, 0],
				  [0, 1.0, 0],
				  [0, 0, 1.0]])

	B = np.array([[DT * math.cos(x[2, 0]), 0],
				  [DT * math.sin(x[2, 0]), 0],
				  [0.0, DT]])

	x = F @ x + B @ u

	return x


def observation_model(x):
	H = np.array([[1, 0, 0],
		          [0, 1, 0],
		          [0, 0, 1]])

	z = H @ x

	return z


def jacob_f(x, u):
	yaw = x[2, 0]
	v = u[0, 0]
	jF = np.array([[1.0, 0.0, -DT * v * math.sin(yaw)],
		           [0.0, 1.0,  DT * v * math.cos(yaw)],
		           [0.0, 0.0,           1.0         ]])

	return jF


def jacob_h():
	jH = np.array([[1, 0, 0],
		           [0, 1, 0],
		           [0, 0, 1]])

	return jH


def ekf_estimation(xEst, PEst, z, u):
	#  Predict
	xPred = motion_model(xEst, u)
	jF = jacob_f(xEst, u)
	PPred = jF @ PEst @ jF.T + Q

	#  Update
	jH = jacob_h()
	zPred = observation_model(xPred)
	y = z - zPred
	S = jH @ PPred @ jH.T + R
	K = PPred @ jH.T @ np.linalg.inv(S)
	xEst = xPred + K @ y
	PEst = (np.eye(len(xEst)) - K @ jH) @ PPred
	return xEst, PEst

def makeFig(): #Create a function that makes our desired plot
	plt.plot(h_xref,h_yref,label="Reference Trajactory")
	plt.plot(h_xtru,h_ytru,label="Ground Truth")
	plt.plot(h_xDR,h_yDR,'--',linewidth=2,markersize=4,label="Encoder Only")
	plt.plot(h_xest,h_yest,'-.',linewidth=2,markersize=4,label="EKF Fusion")
	plt.legend(("Reference Trajactory","True Trajactory","Encoder Only","EKF Fusion"), loc='upper left', shadow=True,fontsize=20)
	plt.tick_params(axis='both', which='major', labelsize=20)
	plt.xlabel("X(m)",fontsize=20)
	plt.ylabel("Y(m)",fontsize=20)


# Initialize ==============================================================================
# path history
h_xref = np.array([[0]])
h_yref = np.array([[0]])
h_xtru = np.array([[0]])
h_ytru = np.array([[0]])
h_xest = np.array([[0]])
h_yest = np.array([[0]])
h_xDR  = np.array([[0]])
h_yDR  = np.array([[0]])
time = 0

# init pose
qc = np.array([[0],[0],[0]]) # (3X1)
xEst = np.zeros((3, 1))
xTrue = np.zeros((3, 1))
PEst = np.eye(3)
xDR = np.zeros((3, 1))  # Dead reckoning
# =========================================================================================
for x in range(930):
#while True:
	x_ref,y_ref,t_ref,v_ref,w_ref,vdotr,wdotr = ref_8(time)
	qr       = np.array([[x_ref],[y_ref],[t_ref]])
	qe       = pose_error(qr,qc)
	vc,wc    = controlkinematic(qe,v_ref,w_ref)

	# Euler Intergral Update new path
	dq = np.array([[vc*math.cos(qc[2,0])],
		           [vc*math.sin(qc[2,0])],
		           [        wc         ]]) # (3X3)
	qc  = qc + dq * DT + np.array([[np.random.normal(0,0.001)],[np.random.normal(0,0.001)],[np.random.normal(0,0.001)]])

	# EKF filter
	u = calc_input(vc,wc)

	xTrue, z, xDR, ud = observation(xTrue, xDR, u)

	xEst, PEst = ekf_estimation(xEst, PEst, z, ud)


	# Store path
	h_xref = np.append(h_xref, np.array([[x_ref]]), axis=0)
	h_yref = np.append(h_yref, np.array([[y_ref]]), axis=0)
	h_xtru = np.append(h_xtru, np.array([[qc[0,0]]]), axis=0)
	h_ytru = np.append(h_ytru, np.array([[qc[1,0]]]), axis=0)
	h_xest = np.append(h_xest, np.array([[xEst[0,0]]]), axis=0)
	h_yest = np.append(h_yest, np.array([[xEst[1,0]]]), axis=0)
	h_xDR  = np.append(h_xDR , np.array([[xDR[0,0]]]), axis=0)
	h_yDR  = np.append(h_yDR , np.array([[xDR[1,0]]]), axis=0)
	#drawnow(makeFig)
	time += 0.033

# f = open("case2_record1_allpose.txt", "w")
# f.write("Now the file has more content!")
# f.close()

makeFig()
plt.show()