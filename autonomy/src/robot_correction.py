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

class robot_correction():
    def __init__(self):
        rospy.init_node('robot_correction', anonymous=True)
        self.cov_publisher = rospy.Publisher('/jfr/robot/correction',PoseWithCovarianceStamped)
        self.robot_cov = None
        self.lidar_cov = None
        rospy.Subscriber('/odometry/filtered',Odometry, self.cb_rl)
        rospy.Subscriber('/jfr/robot/pos/lidar',PoseWithCovarianceStamped, self.cb_lidar) #lidar matching
        while not rospy.is_shutdown():
            if self.robot_cov is not None and self.lidar_cov is not None:
                self.fusion(np.array(self.robot_cov),np.array(self.lidar_cov),np.array(self.robot_state),np.array(self.lidar_state))
            else:
                rospy.sleep(0.01)

    def cb_rl(self, data):
        self.robot_cov = data.pose.covariance
        self.robot_state = self.pose2state(data.pose.pose)
        
    def cb_lidar(self, data):
        self.lidar_cov = data.pose.covariance
        self.lidar_state = self.pose2state(data.pose.pose)

    def pose2state(self, pose):
        oppp = pose.position
        oppo = pose.orientation
        oppoe = tf.transformations.euler_from_quaternion([oppo.x,oppo.y,oppo.z,oppo.w])
        state = np.array([oppp.x,oppp.y,oppp.z,oppoe[0],oppoe[1],oppoe[2]])
        return state

    def state2pose(self, state, cov):
        output = PoseWithCovarianceStamped()
        output.pose.pose.position.x = state[0]
        output.pose.pose.position.y = state[1]
        output.pose.pose.position.z = state[2]

        oppo = tf.transformations.quaternion_from_euler(state[3],state[4],state[5])
        output.pose.pose.orientation.x = oppo[0]
        output.pose.pose.orientation.y = oppo[1]
        output.pose.pose.orientation.z = oppo[2]
        output.pose.pose.orientation.w = oppo[3]
        output.pose.covariance = cov.flatten()

        h = std_msgs.msg.Header()
        h.stamp = rospy.Time.now()
        h.frame_id = 'base_link'
        output.header = h
        return output

    def fusion(self, robot_cov, lidar_cov, robot_state, lidar_state):
        gain = robot_cov/(robot_cov+lidar_cov)
        gain[np.isnan(gain)] = 10**-20
        gain = np.reshape(gain,(6,6))
        new_cov = (np.identity(6)-gain)*np.reshape(robot_cov,(6,6))
        new_cov = np.reshape(new_cov,(36,))
        new_state = robot_state+np.dot(gain,(lidar_state-robot_state))

        output = self.state2pose(new_state, new_cov)
        self.cov_publisher.publish(output)
        self.robot_cov = None
        self.lidar_cov = None


if __name__ == '__main__':
    rospy.loginfo('Looking for object...')
    robot_correction()
    rospy.spin()
