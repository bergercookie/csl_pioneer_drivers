#!/usr/bin/env python

"""
Mon Sep 26 12:00:44 EEST 2016, Nikos Koukis
Send velocity commands to the Arduino controlling a Pioneer 2at robot

Initial script written by: Apostolos Poulias, 2015
Maintainer: Nikos Koukis, 2016:-

TODO:
- Read serial port from the command line and have a default if not given
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
        self.param_names["arduino_port"] = "port"

        # frame ID names
        self.frame_IDs = {}
        self.frame_IDs["odometry"] = "pioneer_odom"
        self.frame_IDs["world"] = "world" # Maybe fetch this from the param server?

        # Message sequence nums
        self.msg_seqs = {}
        self.msg_seqs["odometry"] = 0

        self.rate = 200

        # Setup the robot odometry message
        self.odom = Odometry
        self.odom.header.frame_id = self.frame_IDs["world"]
        self.odom.child_frame_id = self.frame_IDs["odometry"]

        self._setup_publishers_subscribers()
        self._launch_rosserial_node()

    def _setup_publishers_subscribers(self):

        # See
        # http://wiki.ros.org/rospy/Overview/Publishers%20and%20Subscribers#Choosing_a_good_queue_size
        # on how to choose queue_size
        self.arduino_pub = rospy.Publisher('arduino_input_2at', arduino_input,
                                   queue_size=self.rate)
        self.odometry_pub = rospy.Publisher("odom",
                                            Odometry,
                                            queue_size=self.rate)
        rospy.Subscriber("/feedback_2at",
                         feedback_int,
                         self._update_mr_odometry)



    def _launch_rosserial_node(self):
        """Wrapper method for launching a rosserial node."""
        rospy.loginfo("Initializing a rosserial node...")

        package = "rosserial_python"
        executable = "serial_node.py"

        num_tries = 1
        tries_thresh = 10
        rospy.loginfo(
            "Fetching the serial port for communicating with the arduino")
        # Wait until the arduino port parameter is available in the ROS
        # parameter server
        while not rospy.has_param(self.param_names["arduino_port"]):
            rospy.logwarn(
                "Arduino port is not set yet. Retrying... {}/{}".format(
                    num_tries, tries_thresh))
            time.sleep(2)

        serial_port = rospy.get_param(self.arduino_port_param)

        # Launch the rosserial node
        node = roslaunch.core.Node(package, executable, args=serial_port)
        launch = roslaunch.scriptapi.ROSLaunch()
        launch.start()

        self.rosserial_process = launch.launch(node)
        rospy.loginfo("rosserial node is initialized successfully.")


    def shutdown(self):
        """
        Safely shutdown the pioneer instance by sending bunch of 0-velocity
        commands.
        """
        print 'Safe shutdown initiated'

        for i in range(0, 1000):
            self.arduino_pub.publish(mode=3, data1=0, data2=0)
            self.rate.sleep()
        self.rosserial_process.stop()

    def _update_mr_odometry(self, msg):
        """Update the estimated robot position and velocity."""

        encoderR = msg.encoderR
        encoderL = msg.encoderL

        wL = 2.0 * pi * (encoderL / self.encres) / 0.015
        wR = 2.0 * pi * (encoderR / self.encres) / 0.015
        # dtheta = ((encoderR - encoderL) * 2.0 * pi * self.R) / (2.0 * self.encres * self.d)
        omega = -((wR - wL) * self.R) / (self.d * 2.0)
        u = (wR + wL) * self.R / 2.0
        u_x = u * cos(self.odom.pose.pose.theta)
        u_y = u * sin(self.odom.pose.pose.theta)

        #
        # Update the published odometry message
        #
        self.odom.header.stamp = rospy.Time.now()
        self.header.seq = self.msg_seqs["odometry"]

        # Current velocity update
        self.odom.twist.twist.linear = Vector3(x=u, y=0, z=0)
        self.odom.twist.twist.angular = Vector3(x=0, y=0, z=omega)

        # Current position update
        self.odom.pose.pose.x += u_x / float(self.rate)
        self.odom.pose.pose.y += u_y / float(self.rate)
        self.odom.pose.pose.theta += omega / self.rate
        self.odom.pose.pose.theta %= 2 * pi

        # Publish the odometry message
        rospy.logdebug("Publishing pioneer nav_msgs::Odometry message... {}".format(self.odom))
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
