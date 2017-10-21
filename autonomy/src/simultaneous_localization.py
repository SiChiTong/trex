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

class simultaneous_localization():
    def __init__(self):
        rospy.init_node('simultaneous_localization', anonymous=True)

        self.target_pub = rospy.Publisher('/jfr/target/position',PoseWithCovarianceStamped, queue_size=1)
        self.robot_pub = rospy.Publisher('/jfr/robot/correction',PoseWithCovarianceStamped,queue_size=1)
        self.lidar_cov = None
        self.camc_cov = None
        self.camg_cov = None
        self.rl_state = None
        self.lm_state = None
        rospy.Subscriber('/target/lidar',PoseWithCovarianceStamped, self.cb_lidar)
        rospy.Subscriber('/target/camera_color',PoseWithCovarianceStamped, self.cb_cam_color)
        rospy.Subscriber('/target/camera_geom',PoseWithCovarianceStamped, self.cb_cam_geom)
        rospy.Subscriber('/jfr/robot/pos/lidar',PoseWithCovarianceStamped, self.cb_lm)
        #rospy.Subscriber('/odometry/filtered',Odometry, self.cb_rl)
        rospy.Subscriber('/odom',Odometry, self.cb_rl)
        self.first_cov = np.identity(5)+10**0
        self.old_cov = self.first_cov.copy()
        self.old_state = [0,0,0,100,100]

        while not rospy.is_shutdown():
            if self.lidar_cov is not None and self.camc_cov is not None and self.camg_cov is not None and self.rl_state is not None and self.lm_state is not None:
                self.fusion()
            else:
                rospy.sleep(0.01)

    def pose2state(self, pose):
        x = pose.pose.position.x
        y = pose.pose.position.y
        loc = pose.pose.position.z
        cov = [[pose.covariance[0],pose.covariance[1]],[pose.covariance[6],pose.covariance[7]]]
        state = np.array([x,y])
        return state, cov, loc

    def pose2state_robot(self, pose):
        oppp = pose.pose.position
        oppo = pose.pose.orientation
        #print(np.shape(pose.covariance))
        cov = [[pose.covariance[0],pose.covariance[1],pose.covariance[5]],
               [pose.covariance[6],pose.covariance[7],pose.covariance[11]],
               [pose.covariance[30],pose.covariance[31],pose.covariance[35]]]
        oppoe = tf.transformations.euler_from_quaternion([oppo.x,oppo.y,oppo.z,oppo.w])
        state = np.array([oppp.x,oppp.y,oppoe[2]])
        return state, np.array(cov)

    def pose2yaw(self, pose):
        q = [pose.pose.orientation.x,pose.pose.orientation.y,pose.pose.orientation.z,pose.pose.orientation.w]
        euler = tf.transformations.euler_from_quaternion(q)
        yaw = euler[2]
        return yaw

    def tar2loc(self, state, cov):
        theta = np.arctan2(state[1],state[0])
        R = self.rotation_matrix(theta)
        new_cov = np.dot(np.dot(R,cov),R.T)
        return new_cov

    def state2pose(self, state, cov):
        robot_output = PoseWithCovarianceStamped()
        robot_output.pose.pose.position.x = state[0]
        robot_output.pose.pose.position.y = state[1]
        robot_cov = np.zeros((36,))
        robot_cov[0] = cov[0,0]
        robot_cov[1] = cov[0,1]
        robot_cov[5] = cov[0,2]
        robot_cov[6] = cov[1,0]
        robot_cov[7] = cov[1,1]
        robot_cov[11] = cov[1,2]
        robot_cov[30] = cov[2,0]
        robot_cov[31] = cov[2,1]
        robot_cov[35] = cov[2,2]
        robot_output.pose.covariance = robot_cov.flatten()
        oppo = tf.transformations.quaternion_from_euler(0,0,state[2])
        robot_output.pose.pose.orientation.x = oppo[0]
        robot_output.pose.pose.orientation.y = oppo[1]
        robot_output.pose.pose.orientation.z = oppo[2]
        robot_output.pose.pose.orientation.w = oppo[3]



        target_output = PoseWithCovarianceStamped()
        target_output.pose.pose.position.x = state[3]
        target_output.pose.pose.position.y = state[4]
        target_cov = np.zeros((36,))
        target_cov[0] = cov[3,3]
        target_cov[1] = cov[3,4]
        target_cov[6] = cov[4,3]
        target_cov[7] = cov[4,4]
        target_output.pose.covariance = target_cov.flatten()

        h = std_msgs.msg.Header()
        h.stamp = rospy.Time.now()
        h.frame_id = 'odom'
        robot_output.header = h
        target_output.header = h
        return target_output, robot_output

    def cb_lidar(self, data):
        self.lidar_state, lidar_cov, self.lidar_loc = self.pose2state(data.pose)
        #print("lidar_loc:",self.lidar_loc)
        self.lidar_cov = self.tar2loc(self.lidar_state,lidar_cov)

    def cb_cam_color(self, data):
        self.camc_state, camc_cov, self.camc_loc = self.pose2state(data.pose)
        #print("camc_loc:",self.camc_loc)
        self.camc_cov = self.tar2loc(self.camc_state,camc_cov)

    def cb_cam_geom(self, data):
        self.camg_state, camg_cov, self.camg_loc = self.pose2state(data.pose)
        #print("camg_loc:",self.camg_loc)
        self.camg_cov = self.tar2loc(self.camg_state,camg_cov)

    def cb_rl(self, data):
        self.rl_state, self.rl_cov = self.pose2state_robot(data.pose)
        self.rl_cov = self.rl_cov*10**1
        self.rl_pose = data.pose

    def cb_lm(self, data):
        self.lm_state, self.lm_cov = self.pose2state_robot(data.pose)
        self.lm_pose = data.pose
        self.lm_cov = self.lm_cov*10**1

    def rotation_matrix(self, yaw):
        R = np.array([[np.cos(yaw),-np.sin(yaw)],[np.sin(yaw),np.cos(yaw)]])
        return R

    def loc2glo(self, R, state, cov):
        state_glo = np.dot(R,[state[0],state[1]])
        cov_glo = np.dot(np.dot(R,cov),R.T)
        cov_glo = np.reshape(cov_glo,(2,2))
        return state_glo, cov_glo

    def build_eps_r(self,sensors):
        sensors_len = len(sensors)
        sensors_sz = len(sensors[0][:,0])
        cov = np.zeros((sensors_len*sensors_sz,sensors_len*sensors_sz))+10**-10
        ind1col = 0
        ind1row = 0
        for sensor in sensors:
            ind2row = ind1row+sensor.shape[0]
            ind2col = ind1col+sensor.shape[1]
            cov[ind1row:ind2row,ind1col:ind2col] = cov[ind1row:ind2row,ind1col:ind2col]+sensor
            ind1row = ind2row
            ind1col = ind2col
        return cov

    def build_big_eps_r(self,cov_tv,cov_rv):
        target_sz = cov_tv.shape
        robot_sz = cov_rv.shape
        eps_sz = [target_sz[0]+robot_sz[0],target_sz[1]+robot_sz[1]]
        eps = np.zeros((eps_sz[0],eps_sz[1]))+10**-10
        eps[0:robot_sz[0],0:robot_sz[1]] = cov_rv
        eps[robot_sz[0]:,robot_sz[1]:] = cov_tv
        return eps
    
    def calc_gain(self,c_mat,cov, print_flag=False):
        cov_temp = np.dot(np.dot(c_mat,self.old_cov),np.transpose(c_mat))+cov
        cov_mult = np.dot(self.old_cov,np.transpose(c_mat))
        try:
            gain = np.dot(cov_mult,np.linalg.inv(cov_temp))
        except:
            gain = np.zeros_like(np.dot(cov_mult,cov_temp))
        if print_flag:
            print("covtv:")
            print(cov)
            print("Old_cov:")
            print(self.old_cov)
            print("inverse cov_tem:")
            print(np.linalg.inv(cov_temp))
            print("gain:")
            print(gain)
        return gain

    def calc_state(self,sensors,robots,gain):
        obs_state = []
        pred_state = []
        #old_state = []
        #print(robots)
        for robot in robots:
            obs_state.append(robot[0])
            obs_state.append(robot[1])
            obs_state.append(robot[2])
            pred_state.append(self.old_state[0])
            pred_state.append(self.old_state[1])
            pred_state.append(self.old_state[2])
        for sensor in sensors:
            obs_state.append(sensor[0]+self.old_state[0])
            obs_state.append(sensor[1]+self.old_state[1])
            #pred_state.append(robot[0])
            #pred_state.append(robot[1])
            pred_state.append(self.old_state[3])
            pred_state.append(self.old_state[4])
        obs_state = np.array(obs_state)
        pred_state = np.array(pred_state)
        #old_state = np.array(old_state)
        #sum_state = obs_state+pred_state
        new_state = self.old_state+np.dot(gain,obs_state-pred_state)
        #print("obs,pred",obs_state,pred_state)
        return new_state
        

    def fusion(self):
        print_flag = False
        debug_flag = False
        state = []
        ind = []
        yaw = (self.pose2yaw(self.rl_pose)+self.pose2yaw(self.lm_pose))/2
        #print(self.camc_cov)
        lidar_cov = self.lidar_cov.copy()
        camc_cov = self.camc_cov.copy()
        camg_cov = self.camg_cov.copy()
        """
        if self.camc_cov[0,0] > 1:
            camc_cov = np.dot(self.camc_cov,np.array([[0.1,1],[1,1]]))
        else:
            camc_cov = np.copy(self.camc_cov)#*10**-1
        if self.camg_cov[0,0] > 1:
            camg_cov = np.array([[1,0],[0,1]])
        else:
            camg_cov = np.copy(self.camg_cov)#*10**-1
        if self.lidar_cov[0,0]== 0:
            lidar_cov = np.array([[1,0],[0,1]])
        else:
            lidar_cov = np.copy(self.lidar_cov)#*10**-1
        """


        R = self.rotation_matrix(yaw)
        lidar_state_glo, lidar_cov_glo = self.loc2glo(R, self.lidar_state, lidar_cov)
        camc_state_glo, camc_cov_glo = self.loc2glo(R, self.camc_state, camc_cov)
        camg_state_glo, camg_cov_glo = self.loc2glo(R, self.camg_state, camg_cov)
        cov_tv = self.build_eps_r([lidar_cov_glo,camc_cov_glo,camg_cov_glo])
        cov_rv = self.build_eps_r([self.rl_cov,self.lm_cov])

        cov = self.build_big_eps_r(cov_tv,cov_rv)
        if self.lidar_loc+self.camc_loc+self.camg_loc > 0.1:
            #print("total LOC:",self.lidar_loc+self.camc_loc+self.camg_loc)
            c_mat = np.array([[1,0,0,0,0],[0,1,0,0,0],[0,0,1,0,0],[1,0,0,0,0],[0,1,0,0,0],[0,0,1,0,0],[0,0,0,1,0],[0,0,0,0,1],[0,0,0,1,0],[0,0,0,0,1],[0,0,0,1,0],[0,0,0,0,1]])
        else:
            c_mat = np.array([[1,0,0,0,0],[0,1,0,0,0],[0,0,1,0,0],[1,0,0,0,0],[0,1,0,0,0],[0,0,1,0,0],[0,0,0,0,0],[0,0,0,0,0],[0,0,0,0,0],[0,0,0,0,0],[0,0,0,0,0],[0,0,0,0,0]])
        gain = self.calc_gain(c_mat,cov,print_flag=print_flag)
        new_cov =np.dot((np.identity(5)-np.dot(gain,c_mat)),self.old_cov)
        
        self.old_state = self.calc_state([lidar_state_glo,camc_state_glo,camg_state_glo],[self.rl_state,self.lm_state],gain)
        if debug_flag:
            np.savetxt('debug_cov_tv.txt',cov_tv,fmt='%.8e',delimiter=',')
            np.savetxt('debug_cov_rv.txt',cov_rv,fmt='%.8e',delimiter=',')
            np.savetxt('debug_cov.txt',cov,fmt='%.8e',delimiter=',')
            np.savetxt('debug_lidar_cov_glo.txt',lidar_cov_glo,fmt='%.8e',delimiter=',')
            np.savetxt('debug_camc_cov_glo.txt',camc_cov_glo,fmt='%.8e',delimiter=',')
            np.savetxt('debug_camg_cov_glo.txt',camg_cov_glo,fmt='%.8e',delimiter=',')
            np.savetxt('debug_rl_cov.txt',self.rl_cov,fmt='%.8e',delimiter=',')
            np.savetxt('debug_lm_cov.txt',self.lm_cov,fmt='%.8e',delimiter=',')
            np.savetxt('debug_gain.txt',gain,fmt='%.8e',delimiter=',')
            np.savetxt('debug_new_state.txt',self.old_state,fmt='%.8e',delimiter=',')

        target_pose, robot_pose = self.state2pose(self.old_state,new_cov)

        self.target_pub.publish(target_pose)
        self.robot_pub.publish(robot_pose)

        self.old_cov = 1.1*new_cov

        if self.lidar_loc+self.camc_loc+self.camg_loc < 0.1:
            self.old_state[3] = 100
            self.old_state[4] = 100
            self.old_cov = self.first_cov.copy()

        self.lidar_cov = None
        #self.camc_cov = None
        #self.camg_cov = None
        self.rl_state = None
        self.lm_state = None
        #print("hello.")
        rospy.sleep(0.01)

if __name__ == '__main__':
    rospy.loginfo('Publishing target position in global coordinates')
    simultaneous_localization()
    rospy.spin()
