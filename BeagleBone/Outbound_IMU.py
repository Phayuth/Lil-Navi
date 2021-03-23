# import python libraries
import time
import pickle
import socket

# import rcpy library
# This automatically initizalizes the robotics cape
import rcpy
import rcpy.mpu9250 as mpu9250

def udpsend(msg):
	ipd  = "192.168.0.105"
	ptd  = 50507
	sock = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
	dmsg = pickle.dumps(msg,2) # sent to protocol 2 pickle
	sock.sendto(dmsg,(ipd,ptd))

def main():
# defaults
	enable_magnetometer = False
	show_compass = False
	show_gyro = False
	show_accel = True
	show_quat = True
	show_tb = False
	sample_rate = 100
	enable_fusion = True
	show_period = True

	# set state to rcpy.RUNNING
	rcpy.set_state(rcpy.RUNNING)

	# magnetometer ?
	mpu9250.initialize(enable_dmp=True,
					   dmp_sample_rate=sample_rate,
					   enable_fusion=enable_fusion,
					   enable_magnetometer=enable_magnetometer)

	try:

		# keep running
		while True:

			# running
			if rcpy.get_state() == rcpy.RUNNING:

				# t0 = time.perf_counter()
				# data = mpu9250.read()
				# t1 = time.perf_counter()
				# dt = t1 - t0
				# t0 = t1

				# print option data['accel'] , data['gyro'] , data['mag'] , data['head'] , data['quat'] , data['tb'] , Ts = 1000*dt
				acel = data.get('accel')
				gyro = data.get('gyro')
				quat = data.get('quat')
				ax,ay,az = acel[0],acel[1],acel[2]
				gx,gy,gz = gyro[0],gyro[1],gyro[2]
				qw,qx,qy,qz = quat[0],quat[1],quat[2],quat[3]

				# no need to sleep
				Msgsb = [qw,qx,qy,qz,gx,gy,gz,ax,ay,az]
				udpsend(Msgsb)

	except KeyboardInterrupt:
		# Catch Ctrl-C
		pass

	finally:

		# say bye
		print("\nBye Beaglebone!")

# exiting program will automatically clean up cape

if __name__ == "__main__":
	print("=====> Outbound IMU data is Sending <=====")
	main()