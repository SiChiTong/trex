#!/usr/bin/env python

"""robot_correction.py - Version 1.0 2016-10-12
Author: Jonathan Hodges

This code calculates the mean and covariance of robot position using
fused odometry and estimation based on lidar feature matching.

Subscribers:
    /odometry/filtered - robot odometry estimation
    /jfr/robot/pos/lidar - robot position estimation from lidar feature matching.

Publishers:
    /jfr/robot/correction - corrected robot position

This program is free software; you can redistribute it and/or modify it under
the terms of the GNU General Public License as published by the Free Software
Foundation; either version 2 of the License, or (at your option) any later
version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY
WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
PARTICULAR PURPOSE.  See the GNU General Public License for more details at:
http://www.gnu.org/licenses/gpl.html

"""

import roslib; roslib.load_manifest('urg_node')
import rospy
import sensor_msgs.msg
import matplotlib.pyplot as plt
import numpy as np
import scipy as sc
import scipy.signal as sg
import scipy.misc as ms
import scipy.spatial.distance as scd
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
from nav_msgs.msg import Odometry
import StringIO
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError
import urllib, base64
import os
import sys
import math
import tf
import time
import scipy.linalg as linalg
from geometry_msgs.msg import PoseWithCovarianceStamped
import std_msgs.msg

class ocs_logger():
    def __init__(self):
        rospy.init_node('ocs_logger', anonymous=True)
        # Initialize each subscriber
        self.odoms = np.array([0,0,0])
        self.rls = np.array([0,0,0])
        self.lidars = np.array([0,0,0])
        self.rbcs = np.array([0,0,0])
        self.tgs = np.array([0,0,0])
        rospy.Subscriber('/odom',Odometry, self.cb_odom)
        rospy.Subscriber('/odometry/filtered',Odometry, self.cb_rl)
        rospy.Subscriber('/jfr/robot/pos/lidar',PoseWithCovarianceStamped, self.cb_lidar)
        rospy.Subscriber('/jfr/robot/correction',PoseWithCovarianceStamped, self.cb_rbc)
        rospy.Subscriber('/jfr/target/position',PoseWithCovarianceStamped, self.cb_tg)
        while not rospy.is_shutdown():
            self.print_recent()
            rospy.sleep(0.02)

    def print_recent(self):
        print("State\tOdom\t\tOdomFil\t\tLidar\t\tRobot\t\tTarget")
        print("x\t%.6f\t%.6f\t%.6f\t%.6f\t%.6f"%(self.odoms[0],self.rls[0],self.lidars[0],self.rbcs[0],self.tgs[0]))
        print("y\t%.6f\t%.6f\t%.6f\t%.6f\t%.6f"%(self.odoms[1],self.rls[1],self.lidars[1],self.rbcs[1],self.tgs[1]))
        print("z\t%.6f\t%.6f\t%.6f\t%.6f\t%.6f"%(self.odoms[2],self.rls[2],self.lidars[2],self.rbcs[2],self.tgs[2]))

    def cb_odom(self, data):
        self.odoms, self.odomc = self.pose2state(data.pose)

    def cb_rl(self, data):
        self.rls, self.rlc = self.pose2state(data.pose)

    def cb_lidar(self, data):
        self.lidars, self.lidarc = self.pose2state(data.pose)

    def cb_rbc(self, data):
        self.rbcs, self.rbc = self.pose2state(data.pose)

    def cb_tg(self, data):
        self.tgs, self.tgc = self.pose2state(data.pose)

    def pose2state(self, pose):
        oppp = pose.pose.position
        oppo = pose.pose.orientation
        cov = [[pose.covariance[0],pose.covariance[1],pose.covariance[5]],
               [pose.covariance[6],pose.covariance[7],pose.covariance[11]],
               [pose.covariance[30],pose.covariance[31],pose.covariance[35]]]
        oppoe = tf.transformations.euler_from_quaternion([oppo.x,oppo.y,oppo.z,oppo.w])
        state = np.array([oppp.x,oppp.y,oppoe[2]])
        return state, cov



if __name__ == '__main__':
    rospy.loginfo('Logging positions to console.')
    ocs_logger()
    rospy.spin()
