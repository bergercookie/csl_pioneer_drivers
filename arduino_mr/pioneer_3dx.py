#!/usr/bin/env python

"""
Mon Sep 26 12:00:44 EEST 2016, Nikos Koukis
Send velocity commands to the Arduino controlling a Pioneer 3dx robot

Initial script written by: Apostolos Poulias, 2015
Maintainer: Nikos Koukis, 2016:-

TODO:
- Make sure that clock is not published  (e.g. by a rosbag)
"""

import rospy
import roslaunch
from arduino_mr.msg import arduino_input

package = 'rosserial_python'
executable = 'serial_node.py'
serial_port = '/dev/ttyACM0'
node = roslaunch.core.Node(package, executable, args=serial_port)

launch = roslaunch.scriptapi.ROSLaunch()
launch.start()

process = launch.launch(node)


class safe_shutdown():
    def __init__(self):
        rospy.init_node('safe_shutdown', anonymous=True)
        rospy.on_shutdown(self.shutdown)
        self.r = rospy.Rate(200) # Hz
        rospy.loginfo("Starging arduino_input_3dx publisher")
        self.pub = rospy.Publisher('arduino_input_3dx', arduino_input)#, queue_size=1)

        while not rospy.is_shutdown():
            self.r.sleep()

    def shutdown(self):
        rospy.loginfo("Executing Shutwdown...")
        for i in range(0, 100):
            self.pub.publish(mode=3, data1=0, data2=0)
            self.r.sleep()
        rospy.loginfo("Exiting...")
        process.stop()

if __name__ == '__main__':
    try:
        safe_shutdown()
    except rospy.ROSInterruptException:
        pass
