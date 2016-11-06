#!/usr/bin/env python

"""
Mon Sep 26 12:00:44 EEST 2016, Nikos Koukis
Send velocity commands to the Arduino controlling a Pioneer 3dx robot

Initial script written by: Apostolos Poulias, 2015
Maintainer: Nikos Koukis, 2016:-

TODO:
- Make sure that clock is not published  (e.g. by a rosbag)
- Read serial port from the command line and have a default if not given
"""


import rospy
import roslaunch
# import os
from math import *
from math import pi
from math import cos, sin
import time
from arduino_mr.msg import arduino_input, feedback_int
from mrpt_msgs.msg import Pose2DStamped
from geometry_msgs.msg import Vector3Stamped


class ATX2():
    def __init__(self):
        #
        # Robot HW properties
        #

        self.R = 0.22 / 2.0
        self.d = 0.1905 * 1.63
        # self.b = 0.1905
        # self.nnpulley = 79.66215
        self.encres = 8187.5
        # self.looptime = 15.0 * 10.0**(-3.0)

        #
        # Position estimation based only on odometry
        #
        self.x_total = 0
        self.y_total = 0
        self.theta_total = 0




        self.rate = 200
        # See http://wiki.ros.org/rospy/Overview/Publishers%20and%20Subscribers#Choosing_a_good_queue_size
        # on how to choose queue_size
        self.pub = rospy.Publisher('arduino_input_2at', arduino_input,
                                   queue_size=self.rate)	
        self.pub_velocity_odom = rospy.Publisher('/pioneer/odom',
                                                 Vector3Stamped,
                                                 queue_size=self.rate)	
        self.pub_position_odom = rospy.Publisher('/pioneer/position_odom',
                                                 Pose2DStamped,
                                                 queue_size=self.rate)	

        rospy.Subscriber("/feedback_2at", feedback_int, self.updateMROdometry)	

        self._launchRosserialNode()


    def _launchRosserialNode(self):
        """Wrapper method for launching a rosserial node."""
        rospy.loginfo("Initializing a rosserial node...")

        package = "rosserial_python"
        executable = "serial_node.py"

        param_arduino_port = "/arduino/port"
        num_tries = 1
        tries_thresh = 10
        rospy.loginfo(
            "Fetching the serial port for communicating with the arduino")
        # Wait until the arduino port parameter is available in the ROS
        # parameter server
        while not rospy.has_param(param_arduino_port):
            rospy.logwarn(
                "Arduino port is not set yet. Retrying... {}/{}".format(
                    num_tries, tries_thresh))
            time.sleep(2)

        serial_port = rospy.get_param("/arduino/port")
        node = roslaunch.core.Node(package, executable, args=serial_port)

        launch = roslaunch.scriptapi.ROSLaunch()
        launch.start()
        launch.launch(node)
        self.rosserial_process = rospy.loginfo(
            "rosserial node is initialized successfully.")


    def shutdown(self):
        for i in range(0, 100):
            self.pub.publish(mode=3, data1=0, data2=0)
            self.rate.sleep()
        print 'Safe shutdown'
        self.rosserial_process.stop()
      
    def updateMROdometry(self, msg):

        encoderR = msg.encoderR
        encoderL = msg.encoderL

        wL = 2.0 * pi * (encoderL / self.encres) / 0.015
        wR = 2.0 * pi * (encoderR / self.encres) / 0.015
        # dtheta = ((encoderR - encoderL) * 2.0 * pi * self.R) / (2.0 * self.encres * self.d)
        omega = -((wR - wL) * self.R) / (self.d * 2.0)
        u = (wR + wL) * self.R / 2.0

        #
        # Robot velocity msg
        #
        vel_msg = Vector3Stamped()
        vel_msg.header.stamp = rospy.Time.now()
        vel_msg.vector.x = u
        vel_msg.vector.y = omega
        vel_msg.vector.z = 0.0

        self.pub_velocity_odom.publish(vel_msg)

        #
        # Robot position estimation based on odometry
        #

        # velocity components wrt x, y directions
        u_x = u * cos(self.theta_total)
        u_y = u * sin(self.theta_total)

        # update position estimation - 1st order integration
        self.x_total += u_x / float(self.rate)
        self.y_total += u_y / float(self.rate)

        self.theta_total += omega / self.rate
        self.theta_total %= (2 * pi)


        # publish position
        pos_msg = Pose2DStamped()
        pos_msg.header.stamp = rospy.Time.now()
        pos_msg.pose.x = self.x_total
        pos_msg.pose.y = self.y_total
        pos_msg.pose.theta = self.theta_total
        self.pub_position_odom.publish(pos_msg)



if __name__ == '__main__':
    rospy.init_node('pioneer_atx_node')
    obj = ATX2()
    rate_it = rospy.Rate(obj.rate)
    try:
        while not rospy.is_shutdown():
            rate_it.sleep()
        #safe_shutdown()
            rospy.spin()
    except rospy.ROSInterruptException:
        pass
