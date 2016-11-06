#!/usr/bin/env python

import rospy
import math
from arduino_mr.msg import arduino_input
import time

global msg

def set_vel():
	msg=arduino_input()
	msg.mode = 1

	if msg.data1 > 0.67:
		msg.data1 = msg.data1
	else:
		msg.data1 = msg.data1+0.1
	msg.data2 = 0.0	
	
	pub = rospy.Publisher('arduino_input_2at', arduino_input)

		
	return msg


if __name__ =='__main__':
	rospy.init_node('publish_vel')	
	pub = rospy.Publisher('arduino_input_2at', arduino_input)
	"""t0 = rospy.get_time()"""
	msg=arduino_input()
	msg.data1 = 0.0
	while not rospy.is_shutdown():
		try:
			"""t1 = rospy.get_time()"""
			"""if t1-t0 > 9:
				msg=arduino_input()
				msg.mode = 1
				msg.data1 = 0.0
				msg.data2 = 0.0
			else:"""
			msg = set_vel()
		
			pub.publish(msg)		
				
			rospy.Rate(2)
		except rospy.ROSInterruptException:
			pass
