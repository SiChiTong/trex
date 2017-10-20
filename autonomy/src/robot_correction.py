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
        self.robot_whole_cov = np.array(data.pose.covariance)
        self.robot_state,self.robot_cov = self.pose2state(data.pose)
        
    def cb_lidar(self, data):
        self.lidar_state,self.lidar_cov = self.pose2state(data.pose)

    def pose2state(self, pose):
        oppp = pose.pose.position
        oppo = pose.pose.orientation
        #print(np.shape(pose.covariance))
        cov = [[pose.covariance[0],pose.covariance[1],pose.covariance[5]],
               [pose.covariance[6],pose.covariance[7],pose.covariance[11]],
               [pose.covariance[30],pose.covariance[31],pose.covariance[35]]]
        oppoe = tf.transformations.euler_from_quaternion([oppo.x,oppo.y,oppo.z,oppo.w])
        state = np.array([oppp.x,oppp.y,oppoe[2]])
        return state, cov

    def state2pose(self, state, cov):
        #print(cov)
        output = PoseWithCovarianceStamped()
        output.pose.pose.position.x = state[0]
        output.pose.pose.position.y = state[1]
        output.pose.pose.position.z = 0

        oppo = tf.transformations.quaternion_from_euler(0,0,state[2])
        output.pose.pose.orientation.x = oppo[0]
        output.pose.pose.orientation.y = oppo[1]
        output.pose.pose.orientation.z = oppo[2]
        output.pose.pose.orientation.w = oppo[3]
        new_cov = self.robot_whole_cov
        #print(new_cov)
        new_cov[0] = cov[0]
        new_cov[1] = cov[1]
        new_cov[5] = cov[2]
        new_cov[6] = cov[3]
        new_cov[7] = cov[4]
        new_cov[11] = cov[5]
        new_cov[30] = cov[6]
        new_cov[31] = cov[7]
        new_cov[35] = cov[8]
        output.pose.covariance = new_cov.flatten()

        h = std_msgs.msg.Header()
        h.stamp = rospy.Time.now()
        h.frame_id = 'base_link'
        output.header = h
        return output

    def fusion(self, robot_cov, lidar_cov, robot_state, lidar_state):
        #print(np.shape(robot_cov),np.shape(lidar_cov))
        c_mat = np.array([[1,0,0],[0,1,0],[0,0,1]])
        #print("robot_cov:")
        #print(robot_cov)
        #print("lidar_cov:")
        #print(lidar_cov)
        gain = np.dot(robot_cov,c_mat)*np.linalg.inv(np.dot(np.dot(c_mat,robot_cov),c_mat.T)+lidar_cov)
        #print("gain")
        #print(gain)
        gain[np.isnan(gain)] = 10**-20
        gain = np.reshape(gain,(3,3))
        new_cov = (np.identity(3)-gain)*np.reshape(robot_cov,(3,3))
        new_cov = np.reshape(new_cov,(9,))
        new_state = robot_state+np.dot(gain,(lidar_state-np.dot(c_mat,robot_state)))
        #print(lidar_state,robot_state,new_state)

        output = self.state2pose(new_state, new_cov)
        self.cov_publisher.publish(output)
        self.robot_cov = None
        self.lidar_cov = None


if __name__ == '__main__':
    rospy.loginfo('Looking for object...')
    robot_correction()
    rospy.spin()
