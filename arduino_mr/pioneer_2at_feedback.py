#!/usr/bin/env python

import rospy
import roslaunch
from time import sleep
from arduino_feedback.msg import arduino_input

package = 'rosserial_python'
executable = 'serial_node.py'
serial_port = '/dev/ttyACM0'
node = roslaunch.core.Node(package, executable, args=serial_port)

launch = roslaunch.scriptapi.ROSLaunch()
launch.start()

process = launch.launch(node)
print process.is_alive()

class safe_shutdown():
	def __init__(self):
		rospy.init_node('safe_shutdown', anonymous=True)
		rospy.on_shutdown(self.shutdown)
		self.pub = rospy.Publisher('arduino_input_2at', arduino_input)#, queue_size=3)	
		self.r = rospy.Rate(200)

		while not rospy.is_shutdown():
			self.r.sleep()

	def shutdown(self):
		for i in range (0,100):
			self.pub.publish(mode=3, data1=0, data2=0)
			self.r.sleep()
		print 'Safe shutdown'
		process.stop()

if __name__ == '__main__':
	try:
		safe_shutdown()
	except rospy.ROSInterruptException:
		pass

		
