#!/usr/bin/env python

"""footstep_goal.py - Version 1.0 2017-04-05
Created by Jonathan Hodges
This code publishes a goal for the vigir humanoid path planning service.

Subscribers:

Publishers:
    /goal_pose: target location for robot

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
import sys
from geometry_msgs.msg import Twist, Quaternion
import tf

class manipulate_goal():
    def __init__(self,x,y,z,roll,pitch,yaw):
        """This initializes the class.
        """
        rospy.init_node('manipulate_goal', anonymous=True) # Name this node
        # Define user inputs
        self.x = float(x); self.y = float(y); self.z = float(z)
        self.roll = float(roll); self.pitch = float(pitch); self.yaw = float(yaw)
        self.timeout = 10
        sleep_time = 0.1

        # Prepare system for a new goal
        move_state = 'SendGoal'
        rospy.set_param('move_arm_status',move_state)

        # Establish publishers and subscribers
        self.goal_pub = rospy.Publisher("/move_arm/goal", Twist, queue_size=1)
        tw = Twist()
        tw.linear.x = self.x
        tw.linear.y = self.y
        tw.linear.z = self.z
        tw.angular.x = self.roll
        tw.angular.y = self.pitch
        tw.angular.z = self.yaw

        rospy.set_param('move_arm_status','Sent')
        finished = 'false'
        while finished != 'true':
            self.goal_pub.publish(tw)
            rospy.sleep(sleep_time)
            move_state = rospy.get_param('move_arm_status')
            if move_state == 'success':
                rospy.set_param('ee_position',[self.x,self.y,self.z])
                rospy.sleep(0.1)
                finished = 'true'
                rospy.signal_shutdown('Ending Node.')
            elif move_state == 'failure':
                rospy.sleep(0.1)
                finished = 'true'
                rospy.logerr('Failed to get to desired state!')
                rospy.signal_shutdown('Failed to get to desired state!')
                rospy.sleep(0.1)
            else:
                rospy.logdebug("Current move arm status is %s",move_state)

if __name__ == '__main__':
    if len(sys.argv) != 7:
        print "Usage is: manipulate_goal x y z roll pitch yaw"
    else:
        manipulate_goal(sys.argv[1],sys.argv[2],sys.argv[3],sys.argv[4],sys.argv[5],sys.argv[6])
        #rospy.spin()

