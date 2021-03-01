import xbox
import pickle
import socket
import time

#init instance
joy = xbox.Joystick()

######################################################################### UDP
# Define Receiver IP address and Port
ips = "192.168.0.104"
ports = 50505
socks = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
Mb4P = [0, 0, 0] # message to send

# Binding udp self to receive back
#ipr = "192.168.0.105"
#portr = 50506
#sockr = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
#sockr.bind((ipr,portr))
########################################################################

while not joy.Back():
	trigR = joy.rightTrigger()
	joyLY = joy.leftY()
	joyRY = joy.rightY()
	Mb4P[0]=joyLY
	Mb4P[1]=joyRY
	Mb4P[2]=trigR
	Message = pickle.dumps(Mb4P)      # pickling the message
	socks.sendto(Message, (ips, ports))  # send the data to the UDP
	time.sleep(0.02)
#close out when done
joy.close()

