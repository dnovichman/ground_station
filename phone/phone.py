#!/usr/bin/env python
#########################################################################################
#											#
# 		Python package for reading and sending vehicle state information  	#
#		to android device and reading and publishing vehicle commands		#
#		Developers:								#
#				Frits Kuipers						#
#				f.p.kuipers@student.utwente.nl				#
#											#
#				Moses Bangura						#
#					moses.bangura@anu.edu.au			#
#					dnovichman@hotmail.com				#
#########################################################################################
import rospy
import tf
import math
import socket
import errno
import time
import numpy
from socket import error as socket_error
from std_msgs.msg import Float64MultiArray
from tf.msg import tfMessage
from rospy.numpy_msg import numpy_msg

ip = "150.203.212.202" #"130.56.95.182" # "192.168.12.111" #
#ip = "192.168.12.91"
port = 6000

vicon_val = False;
x_before = 0;
y_before = 0;
z_before = 0;
roll_before = 0;
pitch_before = 0;
yaw_before = 0;

clientsocket = 0
receivedData = [0.0,0.0,0.0,0.0,0.0]

def extract_state(state):
    global vicon_val;
    global x_before, y_before, z_before, roll_before, pitch_before, yaw_before
    global clientsocket
    global receivedData

    vicon_val = True;
    x = -state.data[1];
    y = -state.data[0];
    z = state.data[2];

    roll  = state.data[3];
    pitch = state.data[4];
    yaw   = state.data[5];
    print "%.2f" % (x)

    x_before = x;
    y_before = y;
    z_before = z;
    roll_before  = roll;
    pitch_before = pitch;
    yaw_before   = yaw;

    try:
        clientsocket.send(str(x) + " " + str(y) + " " + str(z) + " " + str(roll)+ " " + str(pitch)+ " " + str(yaw) + "\n")
        response = clientsocket.recv(1024)
        if not response:
            clientsocket.close()
            connect()

        rospy.loginfo("DESIRED: " + response)
        receivedData = response.split(" ", 5)
    except socket_error as serr:
        rospy.loginfo("Reconnecting...")       
        clientsocket.close()
        connect()
        time.sleep(1)


def callback(data):
    global vicon_val;
    vicon_val = True;
    global x_before, y_before, z_before, roll_before, pitch_before, yaw_before
    global clientsocket
    global receivedData
    x = data.transforms[0].transform.translation.x;
    y = data.transforms[0].transform.translation.y;
    z = data.transforms[0].transform.translation.z;

    rx = data.transforms[0].transform.rotation.x;
    ry = data.transforms[0].transform.rotation.y;
    rz = data.transforms[0].transform.rotation.z;
    rw = data.transforms[0].transform.rotation.w;

    roll = -math.atan2(2*ry*rz+2*rw*rx,rz**2-ry**2-rx**2+rw**2)
    pitch = -math.asin(2*rx*rz-2*rw*ry)
    yaw = -math.atan2(2*rx*ry+2*rw*rz,rx**2+rw**2-rz**2-ry**2)

    x_before = x;
    y_before = y;
    z_before = z;
    roll_before = roll;
    pitch_before = pitch;
    yaw_before = yaw;

    try:
        clientsocket.send(str(x) + " " + str(y) + " " + str(z) + " " + str(roll)+ " " + str(pitch)+ " " + str(yaw) + "\n")
        response = clientsocket.recv(1024)
        if not response:
            clientsocket.close()
            connect()

        rospy.loginfo("DESIRED: " + response)
        receivedData = response.split(" ", 5)

    except socket_error as serr:
        #rospy.loginfo(serr)
        rospy.loginfo("Reconnecting...")       
        clientsocket.close()
        connect()
        time.sleep(1)

def false_data():
	global clientsocket
	global receivedData
	x = x_before;
	y = y_before;
	z = z_before;
	roll = roll_before;
	pitch = pitch_before;
	yaw = yaw_before;

	try:
		clientsocket.send(str(x) + " " + str(y) + " " + str(z) + " " + str(roll)+ " " + str(pitch)+ " " + str(yaw) + "\n")
		response = clientsocket.recv(1024)
		if not response:
		    clientsocket.close()
		    connect()

		rospy.loginfo("DESIRED: " + response)
		receivedData = response.split(" ", 5)
	except socket_error as serr:
		#rospy.loginfo(serr)
		rospy.loginfo("Reconnecting...")       
		clientsocket.close()
		connect()
		time.sleep(1)
# copied
def valid_float(s):
    try:
        float(s)
        return True
    except ValueError:
        return False
#end copy
    

def listener():
    global receivedData
    global vicon_val;

    # Publish phone data
    rospy.init_node('phone', anonymous=True)

    # Subscribe to the /tf message
    # rospy.Subscriber("tf", tfMessage, callback, queue_size = 10)
    # Subscribe to vehicle state, this already has vicon
    rospy.Subscriber("vehicle_state", Float64MultiArray, extract_state, queue_size = 10)
    count = 0;
    while not rospy.is_shutdown():
		if vicon_val == True & count >= 20:
			vicon_val = False;
			count = 0;
		else:
			false_data();
			count = 0;


		pub = rospy.Publisher('phone', Float64MultiArray,queue_size=10)
		r = rospy.Rate(10) # 10hz
		#if isinstance((receivedData[0]), float) == True:
		if valid_float(receivedData[0]) == True:
			x 	= float(receivedData[0])
			y 	= float(receivedData[1])
			z 	= -float(receivedData[2])
			yaw 	= float(receivedData[3])
			type 	= float(receivedData[4])
			msg_send = Float64MultiArray()
			#msg_send.layout.dim = 4
			msg_send.data = [x, y, z, yaw, type]
			pub.publish(msg_send)
		r.sleep()    
		count = count + 1;


def connect():
    global clientsocket
    try:
        rospy.loginfo("Connecting to "+ip+" ...")
        clientsocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        clientsocket.settimeout(5)
        clientsocket.connect((ip, port))
    except socket_error as serr:
        rospy.loginfo("Connecting failed...")
        clientsocket.close()
        


if __name__ == '__main__':
    connect()
    listener()
