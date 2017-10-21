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

class target_localization():
    def __init__(self):
        rospy.init_node('target_localization', anonymous=True)

        self.target_pub = rospy.Publisher('/jfr/target/position',PoseWithCovarianceStamped, queue_size=1)
        self.lidar_cov = None
        self.camc_cov = None
        self.camg_cov = None
        self.robot_state = None
        rospy.Subscriber('/target/lidar',PoseWithCovarianceStamped, self.cb_lidar)
        rospy.Subscriber('/target/camera_color',PoseWithCovarianceStamped, self.cb_cam_color)
        rospy.Subscriber('/target/camera_geom',PoseWithCovarianceStamped, self.cb_cam_geom)
        rospy.Subscriber('/jfr/robot/correction',PoseWithCovarianceStamped, self.cb_robot)
        #rospy.Subscriber('/odom',Odometry, self.cb_robot)
        self.old_cov = np.identity(2)+10**-3
        self.old_target = [100,100]
        #self.old_cov = dict({'lidar':np.zeros((36,))+1,'camc':np.zeros((36,))+1,'camg':np.zeros((36,))+1})
        #self.old_mean = dict({'lidar':np.zeros((36,))+1000,'camc':np.zeros((36,))+1000,'camg':np.zeros((36,))+1000})
        while not rospy.is_shutdown():
            if self.lidar_cov is not None and self.camc_cov is not None and self.camg_cov is not None and self.robot_state is not None:
                #print("HiHi")
                if self.lidar_loc+self.camc_loc+self.camg_loc >= 0.1:
                    self.fusion(self.lidar_state, np.array(self.lidar_cov), self.lidar_loc, self.camc_state, np.array(self.camc_cov), self.camc_loc, self.camg_state, np.array(self.camg_cov), self.camg_loc, self.robot_state, self.robot_cov)
                else:
                    pass
            else:
                rospy.sleep(0.01)

    def pose2state(self, pose):
        x = pose.pose.position.x
        y = pose.pose.position.y
        loc = pose.pose.position.z
        cov = [[pose.covariance[0],pose.covariance[1]],[pose.covariance[6],pose.covariance[7]]]
        state = np.array([x,y])
        return state, cov, loc

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
        output = PoseWithCovarianceStamped()
        output.pose.pose.position.x = state[0]
        output.pose.pose.position.y = state[1]
        cov_zeros = np.zeros((36,))
        cov_zeros[0] = cov[0,0]
        cov_zeros[1] = cov[0,1]
        cov_zeros[6] = cov[1,0]
        cov_zeros[7] = cov[1,1]
        output.pose.covariance = cov_zeros.flatten()

        h = std_msgs.msg.Header()
        h.stamp = rospy.Time.now()
        h.frame_id = 'odom'
        output.header = h
        return output

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

    def cb_robot(self, data):
        self.robot_state, self.robot_cov, self.robot_loc = self.pose2state(data.pose)
        self.robot_pose = data.pose

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
        cov = np.zeros((sensors_len*sensors_sz,sensors_len*sensors_sz))+10**-20
        ind1col = 0
        ind1row = 0
        for sensor in sensors:
            ind2row = ind1row+sensor.shape[0]
            ind2col = ind1col+sensor.shape[1]
            cov[ind1row:ind2row,ind1col:ind2col] = cov[ind1row:ind2row,ind1col:ind2col]+sensor
            ind1row = ind2row
            ind1col = ind2col
        return cov
    
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
        return gain

    def calc_state(self,sensors,robot,gain):
        sensor_state = []
        robot_state = []
        old_target = []
        for sensor in sensors:
            sensor_state.append(sensor[0])
            sensor_state.append(sensor[1])
            robot_state.append(robot[0])
            robot_state.append(robot[1])
            old_target.append(self.old_target[0])
            old_target.append(self.old_target[1])
        sensor_state = np.array(sensor_state)
        robot_state = np.array(robot_state)
        old_target = np.array(old_target)
        sum_state = robot_state+sensor_state
        new_target = self.old_target+np.dot(gain,sum_state-old_target)
        return new_target
        

    def fusion(self, lidar_state, lidar_cov, lidar_loc, camc_state, camc_cov, camc_loc, camg_state, camg_cov, camg_loc, robot_state, robot_cov):
        print_flag = False
        state = []
        ind = []
        yaw = self.pose2yaw(self.robot_pose)
        if camc_cov[0,0] > 100:
            camc_cov = [[10,0],[0,10]]
        if camg_cov[0,0] > 100:
            camg_cov = [[10,0],[0,10]]

        R = self.rotation_matrix(yaw)
        lidar_state_glo, lidar_cov_glo = self.loc2glo(R, lidar_state, lidar_cov)
        camc_state_glo, camc_cov_glo = self.loc2glo(R, camc_state, camc_cov)
        camg_state_glo, camg_cov_glo = self.loc2glo(R, camg_state, camg_cov)
        cov = self.build_eps_r([lidar_cov_glo,camc_cov_glo,camg_cov_glo])
        c_mat = np.array([[1,0],[0,1],[1,0],[0,1],[1,0],[0,1]])
        gain = self.calc_gain(c_mat,cov,print_flag=print_flag)
        new_cov =np.dot((np.identity(2)-np.dot(gain,c_mat)),self.old_cov)+robot_cov
        
        self.old_target = self.calc_state([lidar_state_glo,camc_state_glo,camg_state_glo],robot_state,gain)

        target_pose = self.state2pose(self.old_target,new_cov)

        self.target_pub.publish(target_pose)

        self.old_cov = new_cov
        #self.lidar_cov = None
        #self.camc_cov = None
        #self.camg_cov = None
        self.robot_state = None
        
        """
        
        #lidar_state_trans = np.dot(R,[lidar_state[0],lidar_state[1]])
        #covR = np.zeros((3,3))
        #covR[0:2,0:2] = R
        #lidar_cov_trans = np.dot(R,lidar_cov)

        #camc_theta = np.arctan2(camc_state[1],camc_state[0])
        #R = self.rotation_matrix(camc_theta)
        #camc_state_trans = np.dot(R,[camc_state[0],camc_state[1]])
        #covR = np.zeros((3,3))
        #covR[0:2,0:2] = R
        #camc_cov = np.array([[100,0],[0,100]])
        #camc_cov_trans = np.dot(R,camc_cov)

        #camg_theta = np.arctan2(camg_state[1],camg_state[0])
        #R = self.rotation_matrix(camg_theta)
        #camg_state_trans = np.dot(R,[camg_state[0],camg_state[1]])
        #covR = np.zeros((3,3))
        #covR[0:2,0:2] = R
        #camg_cov = np.array([[100,0],[0,100]])
        #camg_cov_trans = np.dot(R,camg_cov)

        #lidar_cov = lidar_cov_trans
        #camc_cov = camc_cov_trans
        #camg_cov = camg_cov_trans
        #cov = np.zeros((6,6))+10**-20
        #lidar_cov_glo = np.reshape(lidar_cov_glo,(2,2))
        #camc_cov_glo = np.reshape(camc_cov_glo,(2,2))
        #camg_cov_glo = np.reshape(camg_cov_glo,(2,2))
        #ind1col = 0
        #ind1row = 0
        #ind2row = ind1row+lidar_cov.shape[0]
        #ind2col = ind1col+lidar_cov.shape[1]
        #cov[ind1row:ind2row,ind1col:ind2col] = cov[ind1row:ind2row,ind1col:ind2col]+lidar_cov
        #ind1col = ind2col
        #ind1row = ind2row
        #ind2col = ind1col+camc_cov.shape[1]
        #ind2row = ind1row+camc_cov.shape[0]
        #cov[ind1row:ind2row,ind1col:ind2col] = cov[ind1row:ind2row,ind1col:ind2col]+camc_cov
        #ind1col = ind2col
        #ind1row = ind2row
        #ind2col = ind1col+camg_cov.shape[1]
        #ind2row = ind1row+camg_cov.shape[0]
        #cov[ind1row:ind2row,ind1col:ind2col] = cov[ind1row:ind2row,ind1col:ind2col]+camg_cov

        #c_mat = np.zeros((9,3))
        #c_mat = np.array([[1,0,0],[0,1,0],[0,0,0],[1,0,0],[0,1,0],[0,0,0],[1,0,0],[0,1,0],[0,0,0]])
        #c_mat = np.array([[1,0],[0,1],[1,0],[0,1],[1,0],[0,1]])
        #print("c_mat")
        #print(c_mat)
        #cov_temp = np.dot(np.dot(c_mat,self.old_cov),np.transpose(c_mat))+cov

        print("lidar_cov:")
        print(lidar_cov)
        print("camc_cov:")
        print(camc_cov)
        print("cov_temp:")
        print(cov_temp)
        print("covtv:")
        print(cov)
        print("Old_cov:")
        print(self.old_cov)
        print("inverse cov_tem:")
        print(np.linalg.inv(cov_temp))

        #cov_mult = np.dot(self.old_cov,np.transpose(c_mat))
        #gain = np.dot(cov_mult,np.linalg.inv(cov_temp))

        #print(cov[0,0],cov[1,1],cov[3,3],cov[4,4],cov[6,6],cov[7,7])
        #print("Gain:")
        #print(gain)
        #new_cov =np.dot((np.identity(2)-np.dot(gain,c_mat)),self.old_cov)+robot_cov
        #sensor_state = np.array([lidar_state_trans[0],lidar_state_trans[1],camc_state_trans[0],camc_state_trans[1],camg_state_trans[0],camg_state_trans[1]])
        #print("sensor_state:")
        #print(sensor_state)
        #robot_state_rp = [robot_state[0],robot_state[1],robot_state[0],robot_state[1],robot_state[0],robot_state[1]]
        #print("robot_state_rp:")
        #print(robot_state_rp)
        #sum_state = robot_state_rp+sensor_state
        #print("sum_state:")
        #print(sum_state)
        #old_target_rp = [self.old_target[0],self.old_target[1],self.old_target[0],self.old_target[1],self.old_target[0],self.old_target[1]]
        #print("old_target_rp:")
        #print(old_target_rp)
        #new_target = self.old_target+np.dot(gain,sum_state-old_target_rp)
        #print("Old target:")
        #print(self.old_target)
        #print("New target:")
        #print(new_target)
        #self.old_target = new_target
        #target_pose = self.state2pose(new_target,new_cov)

        
        #self.target_pub.publish(target_pose)

        #self.old_cov = new_cov
        #self.lidar_state = None
        #self.camc_state = None
        #self.camg_state = None
        #self.robot_state = None


        #sensor_state = np.array([lidar_state_trans[0],lidar_state_trans[1],camc_state_trans[0],camc_state_trans[1],camg_state_trans[0],camg_state_trans[1]])

        #robot_state_rp = [robot_state[0],robot_state[1],robot_state[0],robot_state[1],robot_state[0],robot_state[1]]

        #sum_state = robot_state_rp+sensor_state

        #old_target_rp = [self.old_target[0],self.old_target[1],self.old_target[0],self.old_target[1],self.old_target[0],self.old_target[1]]
        #print("old_target_rp:")
        #print(old_target_rp)
        #new_target = self.old_target+np.dot(gain,sum_state-old_target_rp)
        #print("Old target:")
        #print(self.old_target)
        #print("New target:")
        #print(new_target)
        """

if __name__ == '__main__':
    rospy.loginfo('Publishing target position in global coordinates')
    target_localization()
    rospy.spin()
