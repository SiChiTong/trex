#!/usr/bin/env python
import rospy
import numpy as np
import std_msgs.msg
from sensor_msgs.msg import Joy
from nav_msgs.msg import Odometry
import cv2
import os
import signal
from robot_mv_cmds import *
import subprocess
import tf
from visualization_msgs.msg import MarkerArray, Marker

class trex_teleop:
    def __init__(self):
        self.resolution = 0.1
        self.arm_ready_x = 0.506
        self.arm_ready_y = -0.109
        self.arm_ready_z = 0.863
        self.arm_ready_roll = 0.0
        self.arm_ready_pitch = 0.0
        self.arm_ready_yaw = 0.0
        self.arm_ready = str(self.arm_ready_x)+" "+str(self.arm_ready_y)+" "+str(self.arm_ready_z)+" "+str(self.arm_ready_roll)+" "+str(self.arm_ready_pitch)+" "+str(self.arm_ready_yaw)
        self.arm_stow_x = 0.381
        self.arm_stow_y = -0.109
        self.arm_stow_z = 0.705
        self.subscriber = rospy.Subscriber("/joy", Joy, self.callback, queue_size=1)
        self.state = "manipulate"
        self.change_state()

    def callback(self, data):
        change_state=data.buttons[9] # Start button
        failed_flag=data.buttons[6]

        resolution_increase=data.buttons[5] # R1
        resolution_decrease=data.buttons[4] # L1

        if change_state > 0:
            self.change_state()
        elif self.state == "manipulate":
            y_control = data.axes[4]
            z_control = data.axes[5]
            if data.buttons[3] != 0:
                x_control = data.buttons[3]
            elif data.buttons[1] != 0:
                x_control = -1*data.buttons[1]
            else:
                x_control = 0
            yaw_control = 0
            if(resolution_increase>0):
                self.resolution = 2 * self.resolution
                rospy.loginfo("Resolution: %f m",self.resolution)
            elif(resolution_decrease>0):
                self.resolution = 0.5 * self.resolution
                rospy.loginfo("Resolution: %f m",self.resolution)

            elif(abs(y_control)+abs(z_control)+abs(x_control)+abs(yaw_control) > 0):
                if self.state == "manipulate":
                    move_arm_status = rospy.get_param('move_arm_status')
                    if move_arm_status != 'sent':
                        rospy.set_param('move_arm_status','sent')
                        self.control_arm(y_control,z_control, x_control)

    def change_state(self):
        if self.state == "navigate":
            new_state = "manipulate"
            rospy.loginfo("Changing robot state to "+new_state+".")
            rospy.loginfo("Killing navigation teleoperation.")
            os.killpg(os.getpgid(self.nav_state.pid), signal.SIGTERM) 
            rospy.loginfo("Navigation teleoperation killed.")
            rospy.loginfo("Moving arm to work position.")
            #arm_state = subprocess.Popen("rosrun trex_control work_ur5.py", shell=True)
            #arm_state.wait()
            moveArmTwist(self.arm_ready_x,self.arm_ready_y,self.arm_ready_z,0,0,0)
            rospy.loginfo("Arm moved to work position.")
            rospy.set_param('move_arm_status','ready')
            rospy.set_param('ee_position', [self.arm_ready_x,self.arm_ready_y,self.arm_ready_z])
        elif self.state == "manipulate":
            new_state = "navigate"
            rospy.loginfo("Changing robot state to "+new_state+".")
            rospy.loginfo("Stowing arm.")
            #arm_state = subprocess.Popen("rosrun trex_control stow_ur5.py", shell=True)
            #arm_state.wait()
            moveArmTwist(self.arm_stow_x,self.arm_stow_y,self.arm_stow_z,0,0,0)
            rospy.loginfo("Activating navigation teleoperation.")
            self.nav_state = subprocess.Popen("rosrun tele_controller joy_teleop2.py", stdout=subprocess.PIPE, shell=True, preexec_fn=os.setsid)
        self.state = new_state
        rospy.loginfo("Finished changing state.")


    def control_arm(self, y_control,z_control, x_control):
        # print 2*self.resolution
        try:
            ee_posit = rospy.get_param('ee_position')
        except:
            ee_posit = [self.arm_ready_x,self.arm_ready_y,self.arm_ready_z]
        rospy.sleep(0.1)
        self.x = ee_posit[0]
        self.y = ee_posit[1]
        self.z = ee_posit[2]
        self.roll = 0.00
        self.pitch = 0.00
        self.yaw = 0.00
        # y control
        if(y_control>0):
            self.y = self.y + self.resolution
        elif(y_control<0):
            self.y = self.y - self.resolution
        # z control
        elif(z_control>0):
            self.z = self.z + self.resolution
        elif(z_control<0):
            self.z = self.z - self.resolution
        # x control
        elif(x_control>0):
            self.x = self.x + self.resolution
        elif(x_control<0):
            self.x = self.x - self.resolution

        rospy.loginfo("Sending (x,y,z) = (%f,%f,%f)",self.x,self.y,self.z)
        move_arm_status = rospy.get_param('move_arm_status')
        rospy.sleep(0.1)

        # Move the arm. Commented during the development
        moveArmTwist(self.x,self.y,self.z,self.roll,self.pitch,self.yaw)
        move_arm_status = rospy.get_param('move_arm_status')
        rospy.sleep(0.1)
        if move_arm_status == 'success':
            rospy.loginfo("Successful move.")
        elif move_arm_status == 'failure':
            rospy.loginfo("FAILURE! Most likely out of range.")
        elif move_arm_status == 'timedout':
            rospy.loginfo("TIMEDOUT! Most likely out of range.")
        else:
            rospy.logerr("Unknown error, current status is: %s",move_arm_status)

def listener():
    rospy.init_node('teleop', anonymous=True)
    rospy.Subscriber("/joy", Joy, callback)
    #rospy.Subscriber("/joy_teleop/joy", Joy, callback)
    rospy.spin()



if __name__ == '__main__':
    rospy.init_node('teleop', anonymous=True)
    ac=trex_teleop()
    rospy.spin()


