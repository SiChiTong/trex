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
from geometry_msgs.msg import PoseWithCovarianceStamped, PointStamped
import std_msgs.msg
import math

class simultaneous_localization():
    def __init__(self):

        rospy.init_node('simultaneous_localization', anonymous=True)
        self.lm_enable = True
        self.rl_enable = True
        self.lidar_target_enable = True
        self.camc_target_enable = True
        self.camg_target_enable = True

        self.lm_state = None
        self.rl_state = None
        self.lidar_cov = None
        self.camc_cov = None
        self.camg_cov = None

        self.target_pub = rospy.Publisher('/jfr/target/position',PoseWithCovarianceStamped, queue_size=1)
        self.robot_pub = rospy.Publisher('/jfr/robot/correction',PoseWithCovarianceStamped,queue_size=1)
        self.target_H_pub = rospy.Publisher('/jfr/target/entropy',PointStamped,queue_size=1)
        self.robot_H_pub = rospy.Publisher('/jfr/robot/entropy',PointStamped,queue_size=1)
        if self.lm_enable:
            rospy.Subscriber('/jfr/robot/pos/lidar',PoseWithCovarianceStamped, self.cb_lm)
        if self.rl_enable:
            rospy.Subscriber('/odometry/filtered',Odometry, self.cb_rl)
            #rospy.Subscriber('/odom',Odometry, self.cb_rl)
        if self.lidar_target_enable:
            rospy.Subscriber('/target/lidar',PoseWithCovarianceStamped, self.cb_lidar)
        if self.camc_target_enable:
            rospy.Subscriber('/target/camera_color',PoseWithCovarianceStamped, self.cb_cam_color)
        if self.camg_target_enable:
            rospy.Subscriber('/target/camera_geom',PoseWithCovarianceStamped, self.cb_cam_geom)

        self.total_available = sum([self.lm_enable, self.rl_enable, self.lidar_target_enable, self.camc_target_enable, self.camg_target_enable])
        self.first_cov = np.identity(4)
        self.old_cov = self.first_cov.copy()
        self.first_state = np.array([0,0,100,100])
        self.old_state = self.first_state.copy()
        self.mode = 'Sequential'
        self.counter = 0
        print("Got to loop")
        while not rospy.is_shutdown():
            missing_data = False
            if self.lm_enable and self.lm_state is None:
                missing_data = True
            if self.rl_enable and self.rl_state is None:
                missing_data = True
            if self.lidar_target_enable and self.lidar_cov is None:
                missing_data = True
            if self.camc_target_enable and self.camc_cov is None:
                missing_data = True
            if self.camg_target_enable and self.camg_cov is None:
                missing_data = True
            if not missing_data:
                #print("Not missing data!")
                total_loc = self.lidar_loc+self.camc_loc+self.camg_loc
                print("Total LOC:",total_loc)
                if total_loc > 2.0 and self.mode is 'Simultaneous':
                    self.simultaneous_revert()
                    self.counter = 0
                elif total_loc <2.0 and self.mode is 'Sequential':
                    self.sequential()
                    self.counter = 0
                elif total_loc > 2.0 and self.mode is 'Sequential' and self.counter < 100:
                    self.counter = self.counter+1
                    rospy.sleep(0.01)
                elif total_loc > 2.0 and self.mode is 'Sequential' and self.counter >= 100:
                    self.swap2simultaneous()
                    self.simultaneous_revert()
                    self.counter = 0
                elif total_loc < 2.0 and self.mode is 'Simultaneous' and self.counter >= 100:
                    #self.swap2sequential()
                    #self.sequential()
                    self.counter = 0
                elif total_loc < 2.0 and self.mode is 'Simultaneous' and self.counter < 100:
                    self.counter = self.counter+1
                    rospy.sleep(0.01)
                    
            else:
                self.print_missing
                rospy.sleep(0.01)

    def swap2simultaneous(self):
        print("Swap to simultaneous.")
        self.mode = 'Simultaneous'

    def swap2sequential(self):
        print("Swap to sequential.")
        self.mode = 'Sequential'
        self.old_cov = self.first_cov.copy()
        self.old_state = self.first_state.copy()

    def print_missing(self):
        print("Missing something")
        if self.lidar_target_enable and self.lidar_cov is None:
            print("Target lidar missing.")
        if self.camc_target_enable and self.camc_cov is None:
            print("Target color missing.")
        if self.camg_target_enable and self.camg_cov is None:
            print("Target geom missing.")
        if self.rl_enable and self.rl_state is None:
            print("Robot odom missing.")
        if self.lm_enable and self.lm_state is None:
            print("Robot lidar missing.")

    def pose2state(self, pose):
        x = pose.pose.position.x
        y = pose.pose.position.y
        loc = pose.pose.position.z
        cov = [[pose.covariance[0],pose.covariance[1]],[pose.covariance[6],pose.covariance[7]]]
        state = np.array([x,y])
        return state, np.array(cov), loc

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

    def state2pose_sim(self, state, cov):
        robot_output = PoseWithCovarianceStamped()
        robot_output.pose.pose.position.x = state[0]
        robot_output.pose.pose.position.y = state[1]
        robot_cov = np.zeros((36,))
        robot_cov[0] = cov[0,0]
        robot_cov[1] = cov[0,1]
        #robot_cov[5] = cov[0,2]
        robot_cov[6] = cov[1,0]
        robot_cov[7] = cov[1,1]
        #robot_cov[11] = cov[1,2]
        #robot_cov[30] = cov[2,0]
        #robot_cov[31] = cov[2,1]
        #robot_cov[35] = cov[2,2]
        robot_output.pose.covariance = robot_cov.flatten()
        oppo = tf.transformations.quaternion_from_euler(0,0,self.yaw)
        robot_output.pose.pose.orientation.x = oppo[0]
        robot_output.pose.pose.orientation.y = oppo[1]
        robot_output.pose.pose.orientation.z = oppo[2]
        robot_output.pose.pose.orientation.w = oppo[3]

        target_output = PoseWithCovarianceStamped()
        target_output.pose.pose.position.x = state[2]
        target_output.pose.pose.position.y = state[3]
        target_cov = np.zeros((36,))
        target_cov[0] = cov[2,2]
        target_cov[1] = cov[2,3]
        target_cov[6] = cov[3,2]
        target_cov[7] = cov[3,3]
        target_output.pose.covariance = target_cov.flatten()

        h = std_msgs.msg.Header()
        h.stamp = self.t #rospy.Time.now()
        h.frame_id = 'odom'
        robot_output.header = h
        target_output.header = h
        return target_output, robot_output

    def state2pose_seq(self, state, cov):
        robot_output = PoseWithCovarianceStamped()
        robot_output.pose.pose.position.x = state[0]
        robot_output.pose.pose.position.y = state[1]
        robot_cov = np.zeros((36,))
        robot_cov[0] = cov[0,0]
        robot_cov[1] = cov[0,1]
        robot_cov[6] = cov[1,0]
        robot_cov[7] = cov[1,1]
        robot_output.pose.covariance = robot_cov.flatten()
        oppo = tf.transformations.quaternion_from_euler(0,0,self.yaw)
        robot_output.pose.pose.orientation.x = oppo[0]
        robot_output.pose.pose.orientation.y = oppo[1]
        robot_output.pose.pose.orientation.z = oppo[2]
        robot_output.pose.pose.orientation.w = oppo[3]

        h = std_msgs.msg.Header()
        h.stamp = self.t #rospy.Time.now()
        h.frame_id = 'odom'
        robot_output.header = h

        return robot_output

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
        self.rl_state, self.rl_cov, _ = self.pose2state(data.pose)
        self.rl_cov[0,0] = self.rl_cov[0,0]*10**-5
        self.rl_pose = data.pose
        self.t = data.header.stamp

    def cb_lm(self, data):
        self.lm_state, self.lm_cov, _ = self.pose2state(data.pose)
        self.lm_pose = data.pose
        self.lm_cov[0,0] = self.lm_cov[0,0]*10**5

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
    
    def calc_gain(self,c_mat,cov, old_cov, print_flag=False):
        cov_temp = np.dot(np.dot(c_mat,old_cov),np.transpose(c_mat))+cov
        cov_mult = np.dot(old_cov,np.transpose(c_mat))
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

    def calc_seq_state(self,sensors,robot,gain,target):
        sensor_state = []
        robot_state = []
        old_target = []
        for sensor in sensors:
            sensor_state.append(sensor[0])
            sensor_state.append(sensor[1])
            robot_state.append(robot[0])
            robot_state.append(robot[1])
            old_target.append(target[0])
            old_target.append(target[1])
        sensor_state = np.array(sensor_state)
        robot_state = np.array(robot_state)
        old_target = np.array(old_target)
        sum_state = robot_state+sensor_state
        new_target = target+np.dot(gain,sum_state-old_target)
        return new_target

    def calc_state(self,sensors,robots,gain):
        obs_state = []
        pred_state = []
        #old_state = []
        #print(robots)
        for robot in robots:
            obs_state.append(robot[0])
            obs_state.append(robot[1])
            #obs_state.append(robot[2])
            pred_state.append(self.old_state[0])
            pred_state.append(self.old_state[1])
            #pred_state.append(self.old_state[2])
        for sensor in sensors:
            obs_state.append(sensor[0]+self.old_state[0])
            obs_state.append(sensor[1]+self.old_state[1])
            #pred_state.append(robot[0])
            #pred_state.append(robot[1])
            pred_state.append(self.old_state[2])
            pred_state.append(self.old_state[3])
        obs_state = np.array(obs_state)
        pred_state = np.array(pred_state)
        #old_state = np.array(old_state)
        #sum_state = obs_state+pred_state
        #print("old_state")
        #print(self.old_state)
        #print("updat2state")
        #print(np.dot(gain,obs_state-pred_state))
        new_state = self.old_state+np.dot(gain,obs_state-pred_state)
        #print("obs,pred",obs_state,pred_state)
        return new_state
        
    def calculate_entropy(self, cov):
        cov = np.array(cov)*10**10
        H = PointStamped()
        h = std_msgs.msg.Header()
        h.stamp = self.t #rospy.Time.now()
        h.frame_id = 'odom'
        H.header = h
        n = float(cov.shape[0])
        inner = 2*math.pi*math.e
        det = np.linalg.det(cov)
        H.point.x = np.log(np.power(inner**n*det,0.5))
        #print(n,np.power(inner**n*det,0.5))
        return H

    def build_cmat_seq(self):
        c_mat = []
        if self.rl_enable:
            c_mat.append([1,0])
            c_mat.append([0,1])
        if self.lm_enable:
            c_mat.append([1,0])
            c_mat.append([0,1])
        c_mat = np.array(c_mat)
        return c_mat

    def build_cmat_sim(self):
        c_mat = []
        if self.rl_enable:
            c_mat.append([1,0,0,0])
            c_mat.append([0,1,0,0])
        if self.lm_enable:
            c_mat.append([1,0,0,0])
            c_mat.append([0,1,0,0])
        if self.lidar_target_enable:
            c_mat.append([0,0,1,0])
            c_mat.append([0,0,0,1])
        if self.camc_target_enable:
            c_mat.append([0,0,1,0])
            c_mat.append([0,0,0,1])
        if self.camg_target_enable:
            c_mat.append([0,0,1,0])
            c_mat.append([0,0,0,1])
        c_mat = np.array(c_mat)
        return c_mat

    def build_obs_state_seq(self):
        state = []
        if self.rl_enable:
            state.append([self.rl_state[0],self.rl_state[1]])
        if self.lm_enable:
            state.append([self.lm_state[0],self.lm_state[1]])
        state = np.array(state)
        return state

    def build_obs_cov_seq(self):
        cov = []
        if self.rl_enable and self.lm_enable:
            cov.append([self.rl_cov[0,0],self.rl_cov[0,1],0,0])
            cov.append([self.rl_cov[1,0],self.rl_cov[1,1],0,0])
            cov.append([0,0,self.lm_cov[0,0],self.lm_cov[0,1]])
            cov.append([0,0,self.lm_cov[1,0],self.lm_cov[1,1]])
        elif self.rl_enable:
            cov.append([self.rl_cov[0,0],self.rl_cov[0,1]])
            cov.append([self.rl_cov[1,0],self.rl_cov[1,1]])
        elif self.lm_enable:
            cov.append([self.lm_cov[0,0],self.lm_cov[0,1]])
            cov.append([self.lm_cov[1,0],self.lm_cov[1,1]])
        cov = np.array(cov)
        return cov

    def calculate_yaw(self):
        yaw = 0
        ct = 0
        if self.rl_enable:
            yaw = yaw+self.pose2yaw(self.rl_pose)
            ct = ct+1
        if self.lm_enable:
            yaw = yaw+self.pose2yaw(self.lm_pose)
            ct = ct+1
        yaw = yaw/ct
        return yaw

    def sequential(self):
        print_flag = False
        n_robot = sum([self.rl_enable,self.lm_enable])
        self.yaw = self.calculate_yaw()
        if False:
            n_robot_states = 2
            robot_state = np.array([self.old_state[0],self.old_state[1]])
            #lidar_state = self.lm_state
            robot_cov = np.array([[self.old_cov[0,0],self.old_cov[0,1]],[self.old_cov[1,0],self.old_cov[1,1]]])
            #lidar_cov = self.lm_cov
            obs_state = self.build_obs_state_seq()
            obs_cov = self.build_obs_cov_seq()
            c_mat = self.build_cmat_seq()
            

            print(c_mat)
            #print(robot_state)
            #print("robot_cov:")
            #print(robot_cov)
            print("robot_cov:")
            print(robot_cov)
            print("obs_cov:")
            print(obs_cov)
            inside = np.dot(np.dot(c_mat,robot_cov),c_mat.T)+obs_cov
            gain = np.dot(c_mat.T,np.linalg.inv(inside))
            #gain = np.dot(c_mat.T,np.dot
            #gain = np.dot(np.dot(c_mat,robot_cov),np.linalg.inv(np.dot(np.dot(c_mat,robot_cov),c_mat.T)+obs_cov))
            print("gain1:")
            print(gain)
            gain[np.isnan(gain)] = 10**-20
            gain = np.reshape(gain,(n_robot_states,n_robot*n_robot_states))
            #print("gain2:") 
            #print(gain)
            #print(np.shape(robot_state),np.shape(gain),np.shape(obs_state),np.shape(c_mat),np.shape(np.dot(c_mat,robot_state)))
            new_cov = np.dot((np.identity(n_robot_states)-np.dot(gain,c_mat)),robot_cov)
            print("new cov")
            print(new_cov)

            robots = []
            robots_cov = []
            if self.rl_enable:
                robots.append(self.rl_state)
                robots_cov.append(self.rl_cov)
            if self.lm_enable:
                robots.append(self.lm_state)
                robots_cov.append(self.lm_cov)

            print("old_state")
            print(robot_state)
            print("obs state")
            print(robots)
            new_state = self.calc_state([],robots,gain,robot_state)
            print("new state")
            print(new_state)
            #print(new_state)

        else:
            new_state = np.array([self.rl_state[0],self.rl_state[1]])
            new_cov = self.rl_cov
            robot_cov = new_cov
            robot_state = new_state
        robot_state_new = self.state2pose_seq(new_state, new_cov)

        #new_state = robot_state+np.dot(gain,(obs_state-np.dot(c_mat,robot_state)))
        
        robot_state_new = self.state2pose_seq(new_state, new_cov)
        self.robot_pub.publish(robot_state_new)
        self.old_state[0] = np.copy(new_state[0])
        self.old_state[1] = np.copy(new_state[1])
        #print(new_cov[0])
        self.old_cov[0,0] = np.copy(new_cov[0,0])
        self.old_cov[0,1] = np.copy(new_cov[0,1])
        self.old_cov[1,0] = np.copy(new_cov[1,0])
        self.old_cov[1,1] = np.copy(new_cov[1,1])

        robot_H = self.calculate_entropy(new_cov)
        self.robot_H_pub.publish(robot_H)



        target_cov = np.array([[self.old_cov[2,2],self.old_cov[2,3]],[self.old_cov[3,2],self.old_cov[3,3]]])
        target_state = np.array([self.old_state[2],self.old_state[3]])
        R = self.rotation_matrix(self.yaw)
        lidar_state_glo, lidar_cov_glo = self.loc2glo(R, self.lidar_state, self.lidar_cov)
        camc_state_glo, camc_cov_glo = self.loc2glo(R, self.camc_state, self.camc_cov)
        camg_state_glo, camg_cov_glo = self.loc2glo(R, self.camg_state, self.camg_cov)
        cov = self.build_eps_r([lidar_cov_glo,camc_cov_glo,camg_cov_glo])
        c_mat = np.array([[1,0],[0,1],[1,0],[0,1],[1,0],[0,1]])
        gain = self.calc_gain(c_mat,cov,target_cov,print_flag=print_flag)
        new_target_cov =np.dot((np.identity(2)-np.dot(gain,c_mat)),target_cov)+robot_cov
        
        new_target_state = self.calc_seq_state([lidar_state_glo,camc_state_glo,camg_state_glo],robot_state,gain,target_state)
        target_pose = self.state2pose_seq(new_target_state,new_target_cov)
        self.target_pub.publish(target_pose)
        self.old_state[2] = np.copy(new_target_state[0])
        self.old_state[3] = np.copy(new_target_state[1])
        self.old_cov[2,2] = np.copy(new_target_cov[0,0])
        self.old_cov[2,3] = np.copy(new_target_cov[0,1])
        self.old_cov[3,2] = np.copy(new_target_cov[1,0])
        self.old_cov[3,3] = np.copy(new_target_cov[1,1])

        target_H = self.calculate_entropy(new_target_cov)
        self.target_H_pub.publish(target_H)

        self.reset()

    def calc_gain_revert(self,c_mat,cov, print_flag=False):
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

    def calc_state_revert(self,sensors,robots,gain):
        obs_state = []
        pred_state = []
        #old_state = []
        #print(robots)
        for robot in robots:
            obs_state.append(robot[0])
            obs_state.append(robot[1])
            pred_state.append(self.old_state[0])
            pred_state.append(self.old_state[1])
        for sensor in sensors:
            obs_state.append(sensor[0]+self.old_state[0])
            obs_state.append(sensor[1]+self.old_state[1])
            #pred_state.append(robot[0])
            #pred_state.append(robot[1])
            pred_state.append(self.old_state[2])
            pred_state.append(self.old_state[3])
        obs_state = np.array(obs_state)
        pred_state = np.array(pred_state)
        #old_state = np.array(old_state)
        #sum_state = obs_state+pred_state
        new_state = self.old_state+np.dot(gain,obs_state-pred_state)
        #print("obs,pred",obs_state,pred_state)
        return new_state

    def simultaneous_revert(self):
        print_flag = False
        debug_flag = False
        state = []
        ind = []
        yaw = (self.pose2yaw(self.rl_pose)+self.pose2yaw(self.lm_pose))/2
        #print(self.camc_cov)
        lidar_cov = self.lidar_cov.copy()
        camc_cov = self.camc_cov.copy()
        camg_cov = self.camg_cov.copy()

        R = self.rotation_matrix(yaw)
        lidar_state_glo, lidar_cov_glo = self.loc2glo(R, self.lidar_state, lidar_cov)
        camc_state_glo, camc_cov_glo = self.loc2glo(R, self.camc_state, camc_cov)
        camg_state_glo, camg_cov_glo = self.loc2glo(R, self.camg_state, camg_cov)
        cov_tv = self.build_eps_r([lidar_cov_glo,camc_cov_glo,camg_cov_glo])
        cov_rv = self.build_eps_r([self.rl_cov,self.lm_cov])

        cov = self.build_big_eps_r(cov_tv,cov_rv)
        if self.lidar_loc+self.camc_loc+self.camg_loc > 0.1:
            #print("total LOC:",self.lidar_loc+self.camc_loc+self.camg_loc)
            c_mat = np.array([[1,0,0,0],[0,1,0,0],[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1],[0,0,1,0],[0,0,0,1],[0,0,1,0],[0,0,0,1]])
        else:
            c_mat = np.array([[1,0,0,0,0],[0,1,0,0,0],[0,0,1,0,0],[1,0,0,0,0],[0,1,0,0,0],[0,0,1,0,0],[0,0,0,0,0],[0,0,0,0,0],[0,0,0,0,0],[0,0,0,0,0],[0,0,0,0,0],[0,0,0,0,0]])
        gain = self.calc_gain_revert(c_mat,cov,print_flag=print_flag)
        new_cov =np.dot((np.identity(4)-np.dot(gain,c_mat)),self.old_cov)
        
        self.old_state = self.calc_state_revert([lidar_state_glo,camc_state_glo,camg_state_glo],[self.rl_state,self.lm_state],gain)
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

        target_pose, robot_pose = self.state2pose_sim(self.old_state,new_cov)
        _, target_cov, _ = self.pose2state(target_pose.pose)
        _, robot_cov, _ = self.pose2state(robot_pose.pose)
        target_H = self.calculate_entropy(target_cov)
        robot_H = self.calculate_entropy(robot_cov)

        self.target_pub.publish(target_pose)
        self.robot_pub.publish(robot_pose)

        self.target_H_pub.publish(target_H)
        self.robot_H_pub.publish(robot_H)

        self.old_cov = new_cov
        self.reset()

    def simultaneous(self):
        print_flag = False
        debug_flag = False
        state = []
        ind = []
        self.yaw = (self.pose2yaw(self.rl_pose)+self.pose2yaw(self.lm_pose))/2
        #print(self.camc_cov)
        lidar_cov = self.lidar_cov.copy()
        camc_cov = self.camc_cov.copy()
        camg_cov = self.camg_cov.copy()
        """
        if lidar_cov[0,0] != 1:
            lidar_cov[0,0] = lidar_cov[0,0]*10**-2
            lidar_cov[1,1] = lidar_cov[0,0]*10**-5
        if camc_cov[0,0] != 1:
            camc_cov[1,1] = camc_cov[1,1]*10**-8
        if camg_cov[0,0] != 1:
            camc_cov[1,1] = camc_cov[1,1]*10**-8
        """
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

        R = self.rotation_matrix(self.yaw)
        lidar_state_glo, lidar_cov_glo = self.loc2glo(R, self.lidar_state, lidar_cov)
        camc_state_glo, camc_cov_glo = self.loc2glo(R, self.camc_state, camc_cov)
        camg_state_glo, camg_cov_glo = self.loc2glo(R, self.camg_state, camg_cov)

        robots = []
        robots_cov = []
        if self.rl_enable:
            robots.append(self.rl_state)
            robots_cov.append(self.rl_cov)
        if self.lm_enable:
            robots.append(self.lm_state)
            robots_cov.append(self.lm_cov)
        sensors = []
        sensors_cov = []
        if self.lidar_target_enable:
            sensors.append(lidar_state_glo)
            sensors_cov.append(lidar_cov_glo)
        if self.camc_target_enable:
            sensors.append(camc_state_glo)
            sensors_cov.append(camc_cov_glo)
        if self.camg_target_enable:
            sensors.append(camg_state_glo)
            sensors_cov.append(camg_cov_glo)

        cov_tv = self.build_eps_r(sensors_cov)
        cov_rv = self.build_eps_r(robots_cov)

        cov = self.build_big_eps_r(cov_tv,cov_rv)
        c_mat = self.build_cmat_sim()
        """
        print("c_mat")
        print(c_mat)
        print("cov")
        print(cov)
        print("self.old_cov")
        print(self.old_cov)
        """
        gain = self.calc_gain(c_mat,cov,np.array(self.old_cov),print_flag=print_flag)
        new_cov =np.dot((np.identity(4)-np.dot(gain,c_mat)),np.array(self.old_cov))
        """
        print("gain")
        print(gain)
        print("new_cov")
        print(new_cov)
        print("old_state")
        print(self.old_state)
        """
        self.old_state = self.calc_state(sensors,robots,gain)
        """
        print("new_state")
        print(self.old_state)
        """
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

        target_pose, robot_pose = self.state2pose_sim(self.old_state,new_cov)
        _, target_cov, _ = self.pose2state(target_pose.pose)
        _, robot_cov, _ = self.pose2state(robot_pose.pose)

        print("target_cov")
        print(target_cov)
        print("robot_cov")
        print(robot_cov)
        target_H = self.calculate_entropy(target_cov)
        robot_H = self.calculate_entropy(robot_cov)

        self.target_pub.publish(target_pose)
        self.robot_pub.publish(robot_pose)

        self.target_H_pub.publish(target_H)
        self.robot_H_pub.publish(robot_H)

        self.old_cov = new_cov
        """
        if self.lidar_loc+self.camc_loc+self.camg_loc < 0.1:
            #self.old_state[3] = 100
            #self.old_state[4] = 100
            self.old_cov = self.first_cov.copy()
        """

        self.reset()

    def reset(self):
        self.old_cov = 1.05*self.old_cov
        self.lidar_cov = None
        #self.camc_cov = None
        #self.camg_cov = None
        self.rl_state = None
        self.lm_state = None
        rospy.sleep(0.01)

if __name__ == '__main__':
    rospy.loginfo('Publishing target position in global coordinates')
    simultaneous_localization()
    rospy.spin()
