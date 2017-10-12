#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import Int8

class gripper_start():
    def __init__(self):
        rospy.init_node('gripper_start')
        self.pub = rospy.Publisher('/gripper_req', Int8, queue_size=1)
        self.gripper_msg = Int8()
        self.gripper_msg.data = 1;
        for i in range(0,5):
            #self.pub.publish(self.gripper_msg)
            rospy.sleep(0.05)

if __name__ == '__main__':
    gripper_start()
	
