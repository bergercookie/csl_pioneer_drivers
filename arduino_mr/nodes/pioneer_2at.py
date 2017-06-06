#!/usr/bin/env python

"""
Mon Sep 26 12:00:44 EEST 2016, Nikos Koukis

Send velocity commands and read odometry feedback from a Pioneer 2AT robot with
custom drivers (not powered by ARIA)

20170605 Update: 
- Do not run this script directly. Instead launch this via the wrapper
launchfile: setup_pioneer_generic.launch
- rosserial_python/serial.py node is launched separately

Initial script written by: Apostolos Poulias, 2015
Maintainer: Nikos Koukis, 2016 -

"""


import rospy
import roslaunch
from math import pi
from math import cos, sin
import time
from arduino_mr.msg import arduino_input, feedback_int
from geometry_msgs.msg import Vector3
from nav_msgs.msg import Odometry


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

        # parameter names
        self.param_names = {}
        self.param_names["port"] = "~port"

        # dictionary of default ROS parameter values
        self.def_params = {}
        self.def_params[self.param_names["port"]] = "/dev/ttyACM0"

        # frame ID names
        self.frame_IDs = {}
        self.frame_IDs["odometry"] = "pioneer_odom"
        self.frame_IDs["world"] = "world" # Maybe fetch this from the param server?

        # Message sequence nums
        self.msg_seqs = {}
        self.msg_seqs["odometry"] = 0

        self.rate = rospy.Rate(200) # Hz

        # Setup the robot odometry message
        self.odom = Odometry()
        self.odom.header.frame_id = self.frame_IDs["world"]
        self.odom.child_frame_id = self.frame_IDs["odometry"]

        self._setup_publishers_subscribers()

    def _setup_publishers_subscribers(self):

        # See
        # http://wiki.ros.org/rospy/Overview/Publishers%20and%20Subscribers#Choosing_a_good_queue_size
        # on how to choose queue_size
        self.arduino_pub = rospy.Publisher('arduino_input_2at',
                                           arduino_input,
                                           queue_size=self.rate)
        self.odometry_pub = rospy.Publisher("odom",
                                            Odometry,
                                            queue_size=self.rate)
        rospy.Subscriber("/feedback_2at",
                         feedback_int,
                         self._update_mr_odometry)


    def shutdown(self):
        """
        Safely shutdown the pioneer instance by sending bunch of 0-velocity
        commands.
        """
        rospy.loginfo('Safe shutdown initiated')

        for i in range(0, 1000):
            self.arduino_pub.publish(mode=3, data1=0, data2=0)
            self.rate.sleep()

    def _update_mr_odometry(self, msg):
        """Update the estimated robot position and velocity."""

        encoder_r = msg.encoder_r
        encoder_l = msg.encoder_l

        wheel_l = 2.0 * pi * (encoder_l / self.encres) / 0.015
        wheel_r = 2.0 * pi * (encoder_r / self.encres) / 0.015
        # dtheta = ((encoder_r - encoder_l) * 2.0 * pi * self.R) / (2.0 * self.encres * self.d)
        omega = -((wheel_r - wheel_l) * self.R) / (self.d * 2.0)
        # Rotation around Z axis only
        # TODO
        # self.odom.pose.pose.orientation += omega / float(self.rate)
        # self.odom.pose.pose.orientation %= 2 * pi

        u = (wheel_r + wheel_l) * self.R / 2.0
        # TODO - change this
        # u_x = u * cos(self.odom.pose.pose.theta)
        # u_y = u * sin(self.odom.pose.pose.theta)
        u_x = u * cos(0)
        u_y = u * sin(0)

        #
        # Update the published odometry message
        #
        self.odom.header.stamp = rospy.Time.now()
        self.odom.header.seq = self.msg_seqs["odometry"]

        # Current velocity update
        self.odom.twist.twist.linear = Vector3(x=u, y=0, z=0)
        self.odom.twist.twist.angular = Vector3(x=0, y=0, z=omega)

        # Current position update
        self.odom.pose.pose.position.x += u_x / float(self.rate)
        self.odom.pose.pose.position.y += u_y / float(self.rate)

        # Publish the odometry message
        rospy.logdebug(
            "Publishing pioneer nav_msgs::Odometry message... %s",
            self.odom)
        self.odometry_pub.publish(self.odom)

        self.msg_seqs["odometry"] += 1

if __name__ == '__main__':
    rospy.init_node('pioneer_atx_node')
    obj = ATX2()
    rate_it = rospy.Rate(obj.rate)
    try:
        while not rospy.is_shutdown():
            rate_it.sleep()
            rospy.spin()
    except rospy.ROSInterruptException:
        pass
