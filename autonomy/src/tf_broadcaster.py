#!/usr/bin/env python  
import roslib
import rospy

import tf
from sensor_msgs.msg import PointCloud2

class publish_stuff:
    def __init__(self):
        rospy.Subscriber('/velodyne_points',PointCloud2, self.cb_pc2)
        theta = 0.025
        self.h = None
        while not rospy.is_shutdown():
            if self.h is not None:
                br = tf.TransformBroadcaster()
                br.sendTransform((0.6, 0, 0.23),
                                 tf.transformations.quaternion_from_euler(theta, 0, -1.57),
                                 self.h,
                                 "velodyne_new",
                                 "base_link")
            rospy.sleep(0.1)
            #print(self.h)
    def cb_pc2(self, data):
        self.h = data.header.stamp

if __name__ == '__main__':
    rospy.init_node('velodyne_tf_broadcaster')
    publish_stuff()
    rospy.spin()
