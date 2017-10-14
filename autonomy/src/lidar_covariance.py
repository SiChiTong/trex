#!/usr/bin/env python

"""lidar_covariance.py - Version 1.0 2016-10-12
Author: Jonathan Hodges

This code calculates the covariance of a robot's position based on uncertainty
in LiDAR measurements

Subscribers:
    /scan: 2-D LIDAR scan from ROS

Publishers:
    /odom/lidarcov: odometry from robot with covariance from LiDAR

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

class lidar_covariance():
    def __init__(self):
        rospy.init_node('lidar_covariance', anonymous=True)
        self.physical_robot = False
        self.lidar_param_pub = rospy.Publisher("/lidar_params",numpy_msg(Floats), queue_size=1)
        self.lidar_odom_pub = rospy.Publisher('/odom/lidarcov',Odometry, queue_size=1)
        self.xmin = 0.0
        self.xmax = 30.0
        self.umin = 0.0
        self.umax = 0.02
        if self.physical_robot:
            rospy.Subscriber("/scan/long_range",sensor_msgs.msg.LaserScan,self.cb_scan, queue_size=1)
        else:
            rospy.Subscriber("/scan",sensor_msgs.msg.LaserScan,self.cb_scan, queue_size=1)
        rospy.Subscriber('/odometry/filtered',Odometry, self.cb_odom)
        rospy.Subscriber('/odom/target',Odometry, self.cb_target)

    def prediction(self, old_cov, propogation_factor=1.1):
        new_cov = old_cov*np.multiply(np.identity(6),propogation_factor)
        return new_cov

    def kalman_gain(self, pred_cov, sig_rv):
        #dhdxt = 1
        #dhdxr = -1
        dhdxr = np.zeros((len(pred_cov),))+2
        #print("Pred_cov:")
        #print(pred_cov)
        gain = np.transpose(dhdxr)*(dhdxr*pred_cov*np.transpose(dhdxr)+sig_rv)
        #print("Gain:")
        #print(gain)
        return gain

    def correction(self, pred_cov, gain):
        dhdxr = np.zeros((len(pred_cov),))+2
        new_cov = (np.identity(6)-gain*dhdxr)*pred_cov
        return new_cov

    def cb_target(self, data):
        self.lidar_odom = data

    def odom_correction(self, pose_gain, twist_gain, pred_odom, cor_pose_cov, cor_twist_cov):
        #print(np.shape(pred_odom.pose.pose.position.x),np.shape(pose_gain),np.shape(self.lidar_odom.pose.pose.position.x))
        po = self.odom2state(pred_odom)
        lo = self.odom2state(self.lidar_odom)
        dhdxr = np.zeros((len(cor_pose_cov),))+2
        #pg = np.array([0.1,0.1,0.1,0.1,0.1,0.1])
        #print(pose_gain)
        #print(po[0],lo[0],pg[0])
        #print(pg)
        #print(po[0], lo[0])
        #no = po+pose_gain*(lo-po)
        diff = (lo-dhdxr*po)
        toadd = np.dot(pose_gain,diff)
        no = po+np.dot(pose_gain,diff)
        #print(toadd)
        #print(np.shape(no),np.shape(toadd),np.shape(diff),np.shape(pose_gain),np.shape(np.dot(pose_gain,diff)))
        new_odom = self.state2odom(pred_odom, no, cor_pose_cov)
        #print(np.shape(pose_gain*(lo-dhdxr*po)))
        #print(new_odom.pose)
        return new_odom

    def odom2state(self, odom):
        oppp = odom.pose.pose.position
        oppo = odom.pose.pose.orientation
        oppoe = tf.transformations.euler_from_quaternion([oppo.x,oppo.y,oppo.z,oppo.w])
        state = np.array([oppp.x,oppp.y,oppp.z,oppoe[0],oppoe[1],oppoe[2]])
        return state

    def state2odom(self, odom, state, cov):
        odom.pose.pose.position.x = state[0]
        odom.pose.pose.position.y = state[1]
        odom.pose.pose.position.z = state[2]

        oppo = tf.transformations.quaternion_from_euler(state[3],state[4],state[5])
        odom.pose.pose.orientation.x = oppo[0]
        odom.pose.pose.orientation.y = oppo[1]
        odom.pose.pose.orientation.z = oppo[2]
        odom.pose.pose.orientation.w = oppo[3]
        odom.pose.covariance = cov.flatten()
        return odom
        

    def cb_odom(self, data):
        #sig_rv = np.array(np.identity(6)*self.uvar)
        sig_rv = np.reshape(self.lidar_odom.pose.covariance,(6,6))
        
        pred_pose_cov = np.reshape(data.pose.covariance,(6,6))
        pred_twist_cov = np.reshape(data.twist.covariance,(6,6))
        #pred_pose_cov = self.prediction(old_pose_cov)
        #pred_twist_cov = self.prediction(old_twist_cov)
        pose_gain = self.kalman_gain(pred_pose_cov,sig_rv)

        twist_gain = self.kalman_gain(pred_twist_cov,sig_rv)
        cor_pose_cov = self.correction(pred_pose_cov,pose_gain)
        #print("cor_pose_cov:")
        #print(cor_pose_cov)
        cor_twist_cov = self.correction(pred_twist_cov,twist_gain)
        #print('%.8f\t%.8f\t%.8f\t%.8f\t%.8f'%(self.uvar,old_pose_cov[0][0],pred_pose_cov[0][0],pose_gain[0][0],cor_pose_cov[0][0]))
        cor_odom = self.odom_correction(pose_gain, twist_gain, data, cor_pose_cov, cor_twist_cov)
        #data.pose.covariance = np.reshape(cor_pose_cov,(36,))
        #data.twist.covariance = np.reshape(cor_twist_cov,(36,))
        self.lidar_odom_pub.publish(cor_odom)

    def cb_scan(self, data):
        """
        This callback runs each time a LIDAR scan is obtained from
        the /scan topic in ROS. Returns a topic /lidar_params. The
        first value is the mean lidar distance. The second value is
        the variance in lidar uncertainty.
        """

        # Set max/min angle and increment
        scan_min = data.angle_min
        scan_max = data.angle_max
        scan_inc = data.angle_increment

        scan_time = data.header.stamp.secs
        dist = np.array(data.ranges)
        dist[dist>self.xmax] = np.nan
        dist[dist==0] = np.nan
        u = (dist-self.xmin)/(self.xmax-self.xmin)*(self.umax-self.umin)+self.umin
        self.uvar = np.nanstd(u)**2
        dmn = np.nanmean(dist)

        lidar_param = np.array([dmn,self.uvar], dtype=np.float32)

        self.lidar_param_pub.publish(lidar_param)
        pass

if __name__ == '__main__':
    rospy.loginfo('Looking for object...')
    lidar_covariance()
    rospy.spin()
