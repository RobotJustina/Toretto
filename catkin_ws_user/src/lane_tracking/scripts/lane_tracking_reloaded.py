#!/usr/bin/env python
import rospy
import math
import filter
import numpy as np
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int16 

steering = 100
speed = 0
K_dist = 0.5
K_angle_max = 16.0
K_angle_min = 16.0
K_brake = 0.1
max_speed = 800
turn_speed = 400
dist_to_lane = 100
cutoof=0.1
input_buffer = []
output_buffer = []
buffer_A = np.array([0,0,0,0,0,0])
buffer_B = np.array([0,0,0,0,0,0])
buffer_ae = np.array([0,0,0]) #buffer de angle error, con 2 estados guardados
filter_order = 5
enabled = True
pub_steering = rospy.Publisher("/manual_control/steering",Int16, queue_size = 1) 
pub_speed = rospy.Publisher("/manual_control/speed",Int16, queue_size = 1)



def processing_right_line(data):
	

	A = data.data[0]
	B = data.data[1]
	C = data.data[2]
	if(A==0 and B == 0):
		return
	
	#buffer_ae[2] = buffer_ae[1]
	#buffer_ae[1] = buffer_ae[0]
	#buffer_ae[0] = math.atan(B/A)
	#ae_prom = (buffer_ae[1]*.75) + (buffer_ae[2]*.25)
	#dif = fabs(ae_prom - buffer_ae[0])
	#if(dif < 300 or buffer_ae[2] == 0):
	    #angle_error = math.atan(B/A)
        angle_error = math.atan(B/A)
	#Buffer de entrada, usado para filtro paso bajas y quitar ruido de la linea
	
	input_buffer.insert(len(input_buffer),angle_error)
	
	filtered_angle = 0
	output_buffer.insert(len(output_buffer),0)

	for i in range(0, filter_order):
		filtered_angle = input_buffer[i] * buffer_B[i]

	for j in range(0, filter_order-1):
		filtered_angle = output_buffer[i] * buffer_A[i]
	output_buffer[5] = filtered_angle

	dist_error = math.fabs(A*160 + B*200 + C)/math.sqrt(A*A + B*B) - dist_to_lane
	speed = (-1)*(max_speed - K_brake * math.fabs(angle_error) * (max_speed - turn_speed))
	if -speed < 700:
		K_angle = K_angle_max
	elif (-speed > 1500):
			K_angle = K_angle_min
	else:
			K_angle = K_angle_max - (-speed -700)/800 * (K_angle_max - K_angle_min)
	steering = 100 + K_dist * dist_error + K_angle * angle_error
	
	print('steering:'+str(steering)+'......\n')
	print('speed:'+str(speed)+'......\n')
	input_buffer.pop(0)
	output_buffer.pop(0)
	#msg_steering = steering
	#msg_speed = speed


def processing_stop(data):
	if(data == 1):
		enabled = True
	else:
		enabled = False

def switch(data):
	if data == 1:
		butter_A[4] = butter_N5_0_1_A1
		butter_A[3] = butter_N5_0_1_A2
		butter_A[2] = butter_N5_0_1_A3
		butter_A[1] = butter_N5_0_1_A4
		butter_A[0] = butter_N5_0_1_A5

		butter_B[5] = butter_N5_0_1_B0
		butter_B[4] = butter_N5_0_1_B1
		butter_B[3] = butter_N5_0_1_B2
		butter_B[2] = butter_N5_0_1_B3
		butter_B[1] = butter_N5_0_1_B4
		butter_B[0] = butter_N5_0_1_B5
	elif data == 2:
		butter_A[4] = butter_N5_0_2_A1
		butter_A[3] = butter_N5_0_2_A2
		butter_A[2] = butter_N5_0_2_A3
		butter_A[1] = butter_N5_0_2_A4
		butter_A[0] = butter_N5_0_2_A5

		butter_B[5] = butter_N5_0_2_B0
		butter_B[4] = butter_N5_0_2_B1
		butter_B[3] = butter_N5_0_2_B2
		butter_B[2] = butter_N5_0_2_B3
		butter_B[1] = butter_N5_0_2_B4
		butter_B[0] = butter_N5_0_2_B5
	elif data == 3:	
		butter_A[4] = butter_N5_0_3_A1
		butter_A[3] = butter_N5_0_3_A2
		butter_A[2] = butter_N5_0_3_A3
		butter_A[1] = butter_N5_0_3_A4
		butter_A[0] = butter_N5_0_3_A5

		butter_B[5] = butter_N5_0_3_B0
		butter_B[4] = butter_N5_0_3_B1
		butter_B[3] = butter_N5_0_3_B2
		butter_B[2] = butter_N5_0_3_B3
		butter_B[1] = butter_N5_0_3_B4
		butter_B[0] = butter_N5_0_3_B5
	elif data == 4:
		butter_A[4] = butter_N5_0_4_A1
		butter_A[3] = butter_N5_0_4_A2
		butter_A[2] = butter_N5_0_4_A3
		butter_A[1] = butter_N5_0_4_A4
		butter_A[0] = butter_N5_0_4_A5

		butter_B[5] = butter_N5_0_4_B0
		butter_B[4] = butter_N5_0_4_B1
		butter_B[3] = butter_N5_0_4_B2
		butter_B[2] = butter_N5_0_4_B3
		butter_B[1] = butter_N5_0_4_B4
		butter_B[0] = butter_N5_0_4_B5
	

def listener():
	rospy.init_node("lane_tracking")
	if(enabled):
		rospy.Subscriber("/rightLine",Float32MultiArray,processing_right_line)
	rospy.Subscriber("/manual_control/stop",Int16,processing_stop)

        #pub_steering = rospy.Publisher("/manual_control/steering",Int16, queue_size = 1) 
        #pub_speed = rospy.Publisher("/manual_control/speed",Int16, queue_size = 1)

	np.resize(buffer_A,(filter_order+1))
	np.resize(buffer_B,(filter_order+1))
	buffer_A[5] = 1

	for i in range(0, filter_order):
		input_buffer.append(0.0)
		output_buffer.append(0.0)
	switch(cutoof)
	pub_steering.publish(int(steering))
	pub_speed.publish(int(speed))
	rospy.spin()


while not rospy.is_shutdown():
	listener()
