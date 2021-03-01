#!/usr/bin/env python
import sys
sys.path.insert(0,'/home/jetbot/lilnavi_ws/src/lilnavi/src')
import xbox
import pickle
import socket
import time

# Init joy
joy = xbox.Joystick()

# IP
ip = "192.168.0.104"
pt = 50505
sock = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)

# Message
M = [0,0,0]

print("Press Back on Joystick to close")

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
