#!/usr/bin/env python

"""joint_likelihood.py - Version 1.0 2016-10-12
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
import numpy as np
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
from nav_msgs.msg import Odometry
import tf
from geometry_msgs.msg import PoseWithCovarianceStamped
import std_msgs.msg

class joint_likelihood():
    def __init__(self):
        rospy.init_node('joint_likelihood', anonymous=True)

        self.target_pub = rospy.Publisher('/target/joint_likelihood',PoseWithCovarianceStamped, queue_size=1)

        self.lidar_x = 0
        self.lidar_y = 0
        self.lidar_LOC = 0
        self.lidar_cov = [0,0,0,0]
        self.camc_x = 0
        self.camc_y = 0
        self.camc_LOC = 0
        self.camc_cov = [0,0,0,0]
        self.camg_x = 0
        self.camg_y = 0
        self.camg_LOC = 0
        self.camg_cov = [0,0,0,0]

        rospy.Subscriber('/target/lidar',PoseWithCovarianceStamped, self.cb_lidar)
        rospy.Subscriber('/target/camera_color',PoseWithCovarianceStamped, self.cb_cam_color)
        rospy.Subscriber('/target/camera_geom',PoseWithCovarianceStamped, self.cb_cam_geom)

    def cb_lidar(self, data):
        self.lidar = data
        self.lidar_x = data.pose.pose.position.x
        self.lidar_y = data.pose.pose.position.y
        self.lidar_LOC = data.pose.pose.position.z
        self.lidar_cov = [data.pose.covariance[0],data.pose.covariance[1],data.pose.covariance[6],data.pose.covariance[7]]
        #print(self.lidar_x,self.lidar_y,self.lidar_LOC,self.lidar_cov)
        self.fusion()

    def cb_cam_color(self, data):
        self.camc = data
        self.camc_x = data.pose.pose.position.x
        self.camc_y = data.pose.pose.position.y
        self.camc_LOC = data.pose.pose.position.z
        self.camc_cov = [data.pose.covariance[0],data.pose.covariance[1],data.pose.covariance[6],data.pose.covariance[7]]
        self.fusion()

    def cb_cam_geom(self, data):
        self.camg = data
        self.camg_x = data.pose.pose.position.x
        self.camg_y = data.pose.pose.position.y
        self.camg_LOC = data.pose.pose.position.z
        self.camg_cov = [data.pose.covariance[0],data.pose.covariance[1],data.pose.covariance[6],data.pose.covariance[7]]
        self.fusion()

    def fusion(self):
        print("Here",self.lidar_LOC,self.camc_LOC,self.camg_LOC)
        if self.lidar_LOC > 0 or self.camc_LOC > 0 or self.camg_LOC > 0:
            fused_msg = PoseWithCovarianceStamped()
            COV=fused_msg.pose.covariance
            h = std_msgs.msg.Header()
            h.stamp = rospy.Time.now()
            h.frame_id = 'base_link'
            fused_msg.header = h
            if self.lidar_LOC > 0 and self.camc_LOC > 0 and self.camg_LOC > 0:
                print("Fuse all sensors.")
                fused_msg.pose.pose.position.x = (self.lidar_x+self.camc_x+self.camg_x)/3
                fused_msg.pose.pose.position.y = (self.lidar_y+self.camc_y+self.camg_y)/3
                fused_msg.pose.pose.position.z = self.lidar_LOC+self.camc_LOC+self.camg_LOC 
                COV[0]=self.lidar_cov[0]*self.camc_cov[0]*self.camg_cov[0]
                COV[1]=self.lidar_cov[1]*self.camc_cov[1]*self.camg_cov[1]
                COV[6]=self.lidar_cov[2]*self.camc_cov[2]*self.camg_cov[2]
                COV[7]=self.lidar_cov[3]*self.camc_cov[3]*self.camg_cov[3]
                fused_msg.pose.covariance=COV

            elif self.lidar_LOC > 0 and self.camc_LOC > 0:
                 print("Fuse Lidar and color.")
                 fused_msg.pose.pose.position.x = (self.lidar_x+self.camc_x)/2
                 fused_msg.pose.pose.position.y = (self.lidar_y+self.camc_y)/2
                 fused_msg.pose.pose.position.z = self.lidar_LOC+self.camc_LOC
                 COV[0]=self.lidar_cov[0]*self.camc_cov[0]
                 COV[1]=self.lidar_cov[1]*self.camc_cov[1]
                 COV[6]=self.lidar_cov[2]*self.camc_cov[2]
                 COV[7]=self.lidar_cov[3]*self.camc_cov[3]
                 fused_msg.pose.covariance=COV

            elif self.lidar_LOC > 0 and self.camg_LOC > 0:
                 print("Fuse Lidar and geometry.")
                 fused_msg.pose.pose.position.x = (self.lidar_x+self.camg_x)/2
                 fused_msg.pose.pose.position.y = (self.lidar_y+self.camg_y)/2
                 fused_msg.pose.pose.position.z = self.lidar_LOC+self.camg_LOC
                 COV[0]=self.lidar_cov[0]*self.camg_cov[0]
                 COV[1]=self.lidar_cov[1]*self.camg_cov[1]
                 COV[6]=self.lidar_cov[2]*self.camg_cov[2]
                 COV[7]=self.lidar_cov[3]*self.camg_cov[3]
                 fused_msg.pose.covariance=COV

            elif self.camc_LOC > 0 and self.camg_LOC > 0:
                 print("Fuse Color and Geometry.")
                 fused_msg.pose.pose.position.x = (self.camc_x+self.camg_x)/2
                 fused_msg.pose.pose.position.y = (self.camc_y+self.camg_y)/2
                 fused_msg.pose.pose.position.z = self.camc_LOC+self.camg_LOC 
                 COV[0]=self.camc_cov[0]*self.camg_cov[0]
                 COV[1]=self.camc_cov[1]*self.camg_cov[1]
                 COV[6]=self.camc_cov[2]*self.camg_cov[2]
                 COV[7]=self.camc_cov[3]*self.camg_cov[3]
                 fused_msg.pose.covariance=COV

            elif self.lidar_LOC > 0:
                 print("Only Lidar.")
                 fused_msg.pose.pose.position.x = self.lidar_x 
                 fused_msg.pose.pose.position.y = self.lidar_y 
                 fused_msg.pose.pose.position.z = self.lidar_LOC 
                 COV[0]=self.lidar_cov[0]
                 COV[1]=self.lidar_cov[1]
                 COV[6]=self.lidar_cov[2]
                 COV[7]=self.lidar_cov[3]
                 fused_msg.pose.covariance=COV

            elif self.camc_LOC > 0:
                 print("Only Color.")
                 fused_msg.pose.pose.position.x = self.camc_x
                 fused_msg.pose.pose.position.y = self.camc_y
                 fused_msg.pose.pose.position.z = self.camc_LOC
                 COV[0]=self.camc_cov[0]
                 COV[1]=self.camc_cov[1]
                 COV[6]=self.camc_cov[2]
                 COV[7]=self.camc_cov[3]
                 fused_msg.pose.covariance=COV

            elif self.camg_LOC > 0:
                 print("Only Geometry.")
                 fused_msg.pose.pose.position.x = self.camg_x 
                 fused_msg.pose.pose.position.y = self.camg_y 
                 fused_msg.pose.pose.position.z = self.camg_LOC 
                 COV[0]=self.camg_cov[0]
                 COV[1]=self.camg_cov[1]
                 COV[6]=self.camg_cov[2]
                 COV[7]=self.camg_cov[3]
                 fused_msg.pose.covariance=COV
            print(fused_msg)
            self.target_pub.publish(fused_msg)

if __name__ == '__main__':
    rospy.loginfo('Publishing target joint likelihood')
    joint_likelihood()
    rospy.spin()
