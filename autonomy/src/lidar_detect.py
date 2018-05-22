#!/usr/bin/env python

"""findbox.py - Version 1.0 2016-10-12
Author: Jonathan Hodges

This code searches a 2-D LIDAR scan for an object within a minimum and maximum
length bound. The LIDAR scan is segmented based on null returns and large
deviations between points.

Subscribers:
    /scan: 2-D LIDAR scan from ROS

Publishers:
    /detection: array containing [angle,distance] to the median of the detected
        object in local coordinates. Contains [0,0] if no object is detected.
    /output/keyevent_image: image containing key events in the challenge. This
        code publishes the segmented LIDAR scan.

Attributes:
    plot_data - If False, do not plot LIDAR scan. Plotting the scan slows down
        the code.

This program is free software; you can redistribute it and/or modify it under
the terms of the GNU General Public License as published by the Free Software
Foundation; either version 2 of the License, or (at your option) any later
version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY
WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
PARTICULAR PURPOSE.  See the GNU General Public License for more details at:
http://www.gnu.org/licenses/gpl.html

"""

#import roslib; roslib.load_manifest('urg_node')
import rospy
import sensor_msgs.msg
import std_msgs.msg
#import matplotlib.pyplot as plt
import numpy as np
from geometry_msgs.msg import PoseWithCovarianceStamped
#import scipy as sc
import scipy.signal as sg
#import scipy.misc as ms
#import scipy.spatial.distance as scd
#from rospy.numpy_msg import numpy_msg
#from rospy_tutorials.msg import Floats
from nav_msgs.msg import Odometry
#import StringIO
#from sensor_msgs.msg import Image
#import cv2
#from cv_bridge import CvBridge, CvBridgeError
#import urllib, base64
#import os
#import sys
#import math
import tf
import tf2_ros
from tf2_geometry_msgs import do_transform_point
from geometry_msgs.msg import PointStamped
#import time

class lidar_detect():
    def __init__(self):

        # Initialize ROS node
        rospy.init_node('lidar_detect', anonymous=True)
        self.rate = rospy.Rate(10)

        # Establish publisher
        self.target_pub = rospy.Publisher('/target/lidar',PoseWithCovarianceStamped, queue_size=1)

        # LiDAR segmentation magic numbers
        self.dist_min = 0.25
        self.dist_max = 2.0 # 1m is real target, 41in for height
        self.ylen_lim = 2
        self.ang_min = -0.78
        self.ang_max = 1.57
        self.scan_dist_thresh = 15.0  # Distance threshold to split obj into 2 obj.
        self.max_range = 40
        self.old_cov = np.array([[1,0],[0,1]])
        self.old_state = np.array([0,0])

        # Define uncertainty parameters
        self.xmin = 0.0
        self.xmax = 30.0
        self.umin = 0.0
        self.umax = 0.02

        # Define LOC parameters
        self.locmin = 0.0
        self.locmax = 1.0
        self.loc = 0
        self.loc_thresh = 0.1

        # Find transformation from laser to base_link
        print("Before tf_buffer")
        tf_buffer = tf2_ros.Buffer(rospy.Duration(1200.0)) #tf buffer length
        print("After tf buffer")
        tf_listener = tf2_ros.TransformListener(tf_buffer)
        print("After tf_listener")
        self.R = None
        self.physical_robot = False
        while self.R is None:
            print("Waiting on R")
            #now = rospy.Time.now()
            #self.R = tf_buffer.lookup_transform("base_laser", "base_link", rospy.Time(0), rospy.Duration(100.0))
            #print("Got something")

            try:
                if self.physical_robot:
                    self.R = tf_buffer.lookup_transform("velodyne_new", "base_link", rospy.Time(0), rospy.Duration(100))
                else:
                    self.R = tf_buffer.lookup_transform("base_laser", "base_link", rospy.Time(0), rospy.Duration(100))
            except:
                print("Waiting on laser to base_link tf.")
                rospy.sleep(1)
        print("Found R")
        # Establish subscribers
        if self.physical_robot:
            rospy.Subscriber("/scan/long_range",sensor_msgs.msg.LaserScan,self.cb_scan, queue_size=1)
        else:
            rospy.Subscriber("/scan",sensor_msgs.msg.LaserScan,self.cb_scan, queue_size=1)
        rospy.Subscriber('/odom',Odometry, self.cb_odom)
    def cb_odom(self, data):
        self.odom = data.pose.pose
        self.x0 = self.odom.position.x
        self.y0 = self.odom.position.y
        X0 = self.odom.orientation.x
        Y0 = self.odom.orientation.y
        Z0 = self.odom.orientation.z
        W0 = self.odom.orientation.w
        [roll,pitch,yaw] = tf.transformations.euler_from_quaternion([X0,Y0,Z0,W0])
        self.Rodom = np.array([[np.cos(yaw),-np.sin(yaw)],[np.sin(yaw),np.cos(yaw)]])

    def cb_scan(self, data):
        """
        This callback runs each time a LIDAR scan is obtained from
        the /scan topic in ROS. Returns a topic /detection. If no
        object is found, /detection = [0,0]. If an object is found,
        /detection = [angle,distance] to median of scan.
        """
        self.target_pose = PoseWithCovarianceStamped()
        # Set max/min angle and increment
        scan_min = data.angle_min
        scan_max = data.angle_max
        scan_inc = data.angle_increment
        #now = rospy.get_rostime()
        scan_time = data.header.stamp.secs

        # Build angle array
        if self.physical_robot:
            y = np.arange(scan_min,scan_max,scan_inc)
        else:
            #y = np.arange(scan_min,scan_max,scan_inc)
            y = np.arange(scan_min,scan_max+0.01*scan_inc,scan_inc)

        # Pre-compute trig functions of angles
        ysin = np.sin(y)
        ycos = np.cos(y)

        # Apply a median filter to the range scans
        x = sg.medfilt(data.ranges,1)

        # Calculate the difference between consecutive range values
        x_diff1 = np.power(np.diff(x),2)

        # Convert range and bearing measurement to cartesian coordinates
        y_coord = x*ysin
        x_coord = x*ycos

        # Compute difference between consecutive values in cartesian coordinates
        y_diff = np.power(np.diff(y_coord),2)
        x_diff = np.power(np.diff(x_coord),2)

        # Compute physical distance between measurements
        dist = np.power(x_diff+y_diff,0.5)

        # Segment the LIDAR scan based on physical distance between measurements
        x2 = np.split(x,np.argwhere(dist>self.scan_dist_thresh))
        y2 = np.split(y,np.argwhere(dist>self.scan_dist_thresh))
        dist2 = np.split(dist,np.argwhere(dist>self.scan_dist_thresh))

        x_coord2 = np.split(x_coord,np.argwhere(dist>self.scan_dist_thresh))
        y_coord2 = np.split(y_coord,np.argwhere(dist>self.scan_dist_thresh))
        ran2 = np.split(data.ranges,np.argwhere(dist>self.scan_dist_thresh))

        bearing = np.array([0,0,0,0], dtype=np.float32)
        self.loc = 0
        for i in range(len(y2)):
            # Check if there are at least 4 points in an object (reduces noise)
            ylen = len(y2[i])-0
            dist2_sum = np.sum(dist2[i][1:-2])
            if ylen > self.ylen_lim and dist2_sum > self.dist_min and dist2_sum < self.dist_max and np.median(ran2[i]) <= self.max_range:
                x_pt = np.median(x_coord2[i])
                y_pt = np.median(y_coord2[i])
                if True:
                    ang = np.median(y2[i])
                    dis = np.median(x2[i])
                    mn = min(x2[i][1:ylen])
                    mx = max(x2[i][1:ylen])
                    dis = ((x_pt**2+y_pt**2))**0.5

                    if ang > self.ang_min and ang < self.ang_max:
                        newInd = np.argmin(x_coord2[i][1:])+1
                        x_pt = x_coord2[i][newInd]
                        y_pt = y_coord2[i][newInd]
                        #[x_coord_glo,y_coord_glo] = np.dot(self.Rodom,[x_coord2[i][1:],y_coord2[i][1:]])
                        #print("xMinMax: ",np.min(x_coord_glo),np.max(x_coord_glo))
                        #print("yMinMax: ",np.min(y_coord_glo),np.max(y_coord_glo))
                        #indNew = np.argmin(x_coord_glo)
                        #cornerLoc = [x_coord_glo[indNew]+0.25,y_coord_glo[indNew]+0.5]
                        #print("Left: ",np.argmin(x_coord_glo))
                        #print("Right:",cornerLoc)
                        #print(self.x0,self.y0)
                        #print(x_coord_glo[newInd]+self.x0+0.25)
                        #print(y_coord_glo[newInd]+self.y0-0.5)
                        self.loc = 2*np.exp(-dis/20)
                        c_mat = [[1,0],[0,1]]
                        obs_cov = [[1/(1+10.0*self.loc),0],[0,1/(1+10.0*self.loc)]]
                        obs_state = np.array([x_pt,y_pt])
                        cov_temp = np.dot(np.dot(c_mat,self. old_cov),np.transpose(c_mat))+obs_cov
                        cov_mult = np.dot(self.old_cov,np.transpose(c_mat))
                        try:
                            gain = np.dot(cov_mult,np.linalg.inv(cov_temp))
                        except:
                            gain = np.zeros_like(np.dot(cov_mult,cov_temp))
                        new_cov = np.dot([[1,0],[0,1]]-np.dot(gain,c_mat),self.old_cov)
                        new_state = self.old_state+np.dot(gain,obs_state-self.old_state)
                        print("Obsx\tObsy\tOldx\tOldy\tNewx\tNewy")
                        print("%.4f\t%.4f\t%.4f\t%.4f\t%.4f\t%.4f"%(x_pt,y_pt,self.old_state[0],self.old_state[1],new_state[0],new_state[1]))
                        self.old_state = new_state
                        sl = ylen
                        p = PointStamped()
                        p.point.x = x_pt
                        p.point.y = y_pt
                        p.point.z = 0.0
                        #new_p = do_transform_point(p, self.R)
                        new_p = p
                        new_p.point.x = new_state[0] + 0.6
                        new_p.point.y = new_state[1]
                        new_p.point.z = 0.0
                        u = (dist-self.xmin)/(self.xmax-self.xmin)*(self.umax-self.umin)+self.umin
                        self.uvar = np.nanstd(u)**2

                        #self.loc = (1-(dis-self.xmin)/(self.xmax-self.xmin))*(self.locmax-self.locmin)+self.locmin
                        self.target_pose.pose.pose.position.x = new_p.point.x
                        self.target_pose.pose.pose.position.y = new_p.point.y
                        self.target_pose.pose.pose.position.z = self.loc
                        print(ang,dis,x_pt,y_pt,new_p.point.x,new_p.point.y)
                        cov = self.target_pose.pose.covariance
                        if self.loc > 0.1:
                            self.target_pose.pose.covariance[0] = 0.01*0.01/(1+10.0*self.loc) #self.uvar
                            self.target_pose.pose.covariance[7] = 0.01*1.0/(1+10.0*self.loc) #self.uvar # Tomo asked me to change this value from (0.1*d) to be (1*d) [Tamer]
                        else:
                            self.target_pose.pose.covariance[0] = 10**0
                            self.target_pose.pose.covariance[7] = 10**0
                        print(dis,self.loc)
                    else:
                        pass
                        print("*Found an object outside FoV")
                        #print('fail1')
                else:
                    pass
                    #print('fail2')
            else:
                pass
                #print('fail3',ylen,dist2_sum)
        h = std_msgs.msg.Header()
        h.stamp = rospy.Time.now()
        h.frame_id = 'base_link'
        self.target_pose.header = h
        if self.loc <= 0.1:
            self.target_pose.pose.covariance[0] = 10**0
            self.target_pose.pose.covariance[7] = 10**0

        # Publish bearing to ROS on topic /detection
        self.target_pub.publish(self.target_pose)

        pass

if __name__ == '__main__':
    rospy.loginfo('Looking for object...')
    lidar_detect()
    rospy.spin()
