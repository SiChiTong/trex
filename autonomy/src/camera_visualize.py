#!/usr/bin/env python

"""camera_visualize.py - Version 1.0 2017-03-16
Author: Jonathan Hodges

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
import rospkg
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point
from geometry_msgs.msg import Quaternion, Twist
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import tf
import math
from scipy.cluster import vq
import scipy.stats

class camera_visualize():
    def __init__(self):
        # Name this node, it must be unique
        rospy.init_node('camera_visualize', anonymous=True)

        # Tweaking parameters
        self.circ_diam = 150
        self.rate = rospy.Rate(1)

        # Establish publishers and subscribers
        self.bridge = CvBridge()
        self.image_output = rospy.Publisher("/output/crosshair_image",Image,
            queue_size=1)
        self.tftree = tf.TransformListener()
        rospy.Subscriber("/usb_cam/image_raw",Image,self.callback)

    def callback(self, data):
        offset = -35
        # Convert ROS image to opencv image
        try:
            img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        #img = cv2.resize(img, (0,0), fx=0.5, fy=0.5)
        sz = np.shape(img)
        cv2.line(img,(int(sz[1]/2)-self.circ_diam/2+offset,int(sz[0]/2)),(int(sz[1]/2)+self.circ_diam/2+offset,int(sz[0]/2)),(255,255,255),4)
        cv2.line(img,(int(sz[1]/2+offset),0),(int(sz[1]/2+offset),sz[0]),(255,255,255),4)
        cv2.circle(img, (int(sz[1]/2+offset),int(sz[0]/2)), self.circ_diam/2, (255, 255, 255), 8)
        img_gray = cv2.cvtColor(img.copy(), cv2.COLOR_BGR2GRAY)
        self.image_output.publish(self.bridge.cv2_to_imgmsg(img_gray, "mono8"))
        
if __name__ == '__main__':
    try:
        camera_visualize()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("camera_visualize finished.")

