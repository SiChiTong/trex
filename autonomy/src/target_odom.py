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

import rospy
#import sensor_msgs.msg
import numpy as np
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
from nav_msgs.msg import Odometry
#import StringIO
#from sensor_msgs.msg import Image
#import urllib, base64
#import os
#import sys
#import math
import tf
#import time
from geometry_msgs.msg import PoseWithCovarianceStamped

class target_odom():
    def __init__(self):
        rospy.init_node('target_odom', anonymous=True)
        self.physical_robot = False

        self.target_odom_pub = rospy.Publisher('/odom/target',Odometry, queue_size=1)
        self.xmin = 0.0
        self.xmax = 30.0
        self.locmin = 0.0
        self.locmax = 2.0
        self.loc = 0
        self.loc_thresh = 0.4
        self.cov = np.array([[0.01,0],[0,0.01]])
        self.target_loc = np.array([10.0,10.0])

        rospy.Subscriber('/odom',Odometry, self.cb_odom)
        rospy.Subscriber('/pose_with_covariance_stamped',PoseWithCovarianceStamped, self.cb_pose)
        rospy.Subscriber("/detection",numpy_msg(Floats), self.cb_detect)

    def update_target_location(self, loc_cov,ang,dist):
        self.x0 = self.current_odom.pose.pose.position.x
        self.y0 = self.current_odom.pose.pose.position.y
        X0 = self.current_odom.pose.pose.orientation.x
        Y0 = self.current_odom.pose.pose.orientation.y
        Z0 = self.current_odom.pose.pose.orientation.z
        W0 = self.current_odom.pose.pose.orientation.w
        [roll,pitch,yaw] = tf.transformations.euler_from_quaternion([X0,Y0,Z0,W0])
        self.R = np.array([[np.cos(yaw),-np.sin(yaw)],[np.sin(yaw),np.cos(yaw)]])


        dhdxr = np.zeros((len(self.cov),))+1
        #print("self.cov:")
        #print(self.cov)
        #print('dhdxr:')
        #print(dhdxr)

        gain = np.transpose(dhdxr)*(dhdxr*self.cov*np.transpose(dhdxr) + np.identity(2)*loc_cov)
        po = self.target_loc
        self.cov = self.cov*2.0
        print("ang,dist:",ang,dist)
        print("localx,localy",dist*np.cos(ang),dist*np.sin(ang))
        lo = np.dot(self.R,[dist*np.cos(ang),dist*np.sin(ang)])+[self.x0,self.y0]
        self.cov = (np.identity(2)-gain*dhdxr)*self.cov
        print('gain:')
        print(gain)
        diff = (lo-dhdxr*po)
        print('diff:')
        print(diff)
        toadd = np.dot(gain,diff)
        print('toadd:')
        print(toadd)
        self.target_loc = po+np.dot(gain,diff)
        #print('target_loc:')
        print(self.target_loc,self.cov)
        print(self.cov)

    def cb_odom(self, data):
        self.current_odom = data

    def cb_detect(self, data):
        if data.data[1] == 0:
            self.loc = 0
        else:
            self.loc = (data.data[1]-self.xmin)/(self.xmax-self.xmin)*(self.locmax-self.locmin)+self.locmin
            self.update_target_location(self.loc,data.data[0],data.data[1])

    def cb_pose(self, data):
        current_cov = np.array(data.pose.covariance)
        if self.loc > self.loc_thresh:
            new_cov = np.reshape(current_cov/(10*(1+self.loc)),(36,))
        else:
            new_cov = current_cov
        #print(self.loc,current_cov[0],new_cov[0])
        new_odom = self.current_odom
        new_odom.header = data.header
        new_odom.pose = data.pose
        new_odom.pose.covariance = new_cov
        new_odom.twist.covariance = new_cov
        self.target_odom_pub.publish(new_odom)

if __name__ == '__main__':
    rospy.loginfo('Publishing target odometry.')
    target_odom()
    rospy.spin()
