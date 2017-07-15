#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

class joy_teleop():
	def __init__(self):
		rospy.init_node('Joy_teleop')
		rospy.Subscriber("joy", Joy, self.callback)
		self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
		self.twist = Twist()
		while not rospy.is_shutdown():
			self.pub.publish(self.twist)
			rospy.sleep(0.05)
		
		
	def callback(self, data):
        	self.twist.linear.x = data.axes[1]*2.0735
		self.twist.angular.z = data.axes[0]*6.69
        	print self.twist
 
if __name__ == '__main__':
	joy_teleop()
	rospy.spin()
	
