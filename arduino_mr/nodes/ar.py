#!/usr/bin/env python

import rospy
import roslaunch
from time import sleep
from arduino_mr.msg import arduino_input

package = 'rosserial_python'
executable = 'serial_node.py'
serial_port = '/dev/ttyACMO'
node = roslaunch.core.Node(package, executable, args=serial_port)

launch = roslaunch.scriptapi.ROSLaunch()
launch.start()

process = launch.launch(node)
print process.is_alive()

rospy.init_node('safe_shutdown', anonymous=True)
rospy.on_shutdown(self.shutdown)
r = rospy.Rate(200)
pub = rospy.Publisher('arduino_input_3dx', arduino_input, queue_size=3)		

def main_loop():
	while not rospy.is_shutdown():
		r.sleep()

def safe_shutdown():
	for i in range (0,100):
		pub.publish(mode=3, data1=0, data2=0)
		r.sleep()
	print 'Safe shutdown'
	process.stop()

if __name__ == '__main__':
	try:
		main_loop()
	except rospy.ROSInterruptException:
		pass

		
