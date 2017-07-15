/** author: Sung Jik Cha
 ** credits : ros turtlebot node : https://github.com/Arkapravo/turtlebot
              arduino ros bridge : http://wiki.ros.org/ros_arduino_bridge
**/

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Int32.h>
#include <stdio.h>
#include <cmath>
#include <algorithm>
#include <iostream>
#include <sensor_msgs/Imu.h>



double rpm_act1 = 0.0;
double rpm_act2 = 0.0;
double x_pos = 0.0;
double y_pos = 0.0;
double theta = 0.0;
double gyro_x = 0.0;
double gyro_y = 0.0;
double gyro_z = 0.0;
//ros::Time current_time;
//ros::Time rpm_time(0.0);
//ros::Time last_time(0.0);

//void handle_rpm_left( const geometry_msgs::Vector3Stamped& left_rpm) {
//  rpm_act1 = left_rpm.data;
//}

//void handle_rpm_right( const geometry_msgs::Vector3Stamped& right_rpm) {
//  rpm_act2 = right_rpm.data;
//}


void handle_rpm_left( const std_msgs::Int32& left_rpm) {
  rpm_act1 = left_rpm.data;
}

void handle_rpm_right( const std_msgs::Int32& right_rpm) {
  rpm_act2 = right_rpm.data;
}

/*void handle_gyro( const geometry_msgs::Vector3& gyro) {
  gyro_x = gyro.x;
  gyro_y = gyro.y;
  gyro_z = gyro.z;
}*/

void handle_gyro( const sensor_msgs::Imu& gyro) {
  tf::Matrix3x3 m(gyro.orientation).getRPY(gyro_x,gyro_y,gyro_z);
}

int main(int argc, char** argv){
  ros::init(argc, argv, "base_controller2");

  ros::NodeHandle n;
  ros::NodeHandle nh_private_("~");
  ros::Subscriber left_sub = n.subscribe("vtrex/left_rpm", 50, handle_rpm_left);
  ros::Subscriber right_sub = n.subscribe("vtrex/right_rpm", 50, handle_rpm_right);  
  ros::Subscriber gyro_sub = n.subscribe("imu/data", 50, handle_gyro);
  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
  tf::TransformBroadcaster broadcaster;

  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();
  
  double pi = 3.1415926535897931;
  double rate = 100.0;
  double linear_scale_positive = 1.0;
  double linear_scale_negative = 1.0;
  double angular_scale_positive = 1.0;
  double angular_scale_negative = 1.0;
  double angular_scale_accel = 1.0;
  double acc_theta = 0.0;
  double acc_x = 0.0;
  double acc_max_theta = 0.0;
  double acc_max_x = 0.0;
  double alpha = 0.0;
  bool publish_tf = false;
  bool use_imu = true;
  //double dt = 0.0;
  double dx = 0.0;
  double dy = 0.0;
  double omega_odom = 0.0;
  double dth_gyro = 0.0;
  double dth = 0.0;
  double dth_prev = 0.0;
  double dth_curr = 0.0;
  double dxy_prev = 0.0;
  double v_ave = 0.0;
  double vx = 0.0;
  double vy = 0.0;
  double vth = 0.0;
  double wheel_radius = 0.165;
  double track_width = 0.62;
  char base_link[] = "/base_link";
  char odom[] = "/odom";

  nh_private_.getParam("publish_rate", rate);
  nh_private_.getParam("publish_tf", publish_tf);
  nh_private_.getParam("linear_scale_positive", linear_scale_positive);
  nh_private_.getParam("linear_scale_negative", linear_scale_negative);
  nh_private_.getParam("angular_scale_positive", angular_scale_positive);
  nh_private_.getParam("angular_scale_negative", angular_scale_negative);
  nh_private_.getParam("angular_scale_accel", angular_scale_accel);
  nh_private_.getParam("alpha", alpha);
  nh_private_.getParam("use_imu", use_imu);

  ros::Rate r(rate);
  while(n.ok()){
    ros::spinOnce();
    // ros::topic::waitForMessage<geometry_msgs::Vector3Stamped>("rpm", n, d);

    v_ave = (rpm_act1+rpm_act2)*wheel_radius*pi/(30*2);
    omega_odom = (rpm_act2-rpm_act1)*wheel_radius*pi/(30*track_width);

    current_time = ros::Time::now();
    double dt = (current_time - last_time).toSec();
    if (use_imu) dth_gyro = dt*gyro_z;
    double dtheta=omega_odom*dt*0.72/2.012;
    dth = alpha*dtheta + (1-alpha)*dth_gyro;

    dx = cos(theta) * v_ave*dt;
    dy = sin(theta) * v_ave*dt;

    x_pos += dx;
    y_pos += dy;
    theta = (theta+dtheta);

     
    //printf ("Theta: %f\n", (theta*180/pi));
    
    printf ("rpm_L, rpm_R, v_ave, omega, dt, Theta: %f,%f,%f,%f,%f,%f\n",rpm_act1,rpm_act2,v_ave,omega_odom, dt,(theta*180/pi));

    if(theta >= 2*pi) theta -= 2*pi;
    if(theta <= -2*pi) theta += 2*pi;

    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(theta);

    if(publish_tf) {
      geometry_msgs::TransformStamped t;
      t.header.frame_id = odom;
      t.child_frame_id = base_link;
      t.transform.translation.x = x_pos;
      t.transform.translation.y = y_pos;
      t.transform.translation.z = 0.0;
      t.transform.rotation = odom_quat;
      t.header.stamp = current_time;

      broadcaster.sendTransform(t);
    }

    nav_msgs::Odometry odom_msg;
    odom_msg.header.stamp = current_time;
    odom_msg.header.frame_id = odom;
    odom_msg.pose.pose.position.x = x_pos;
    odom_msg.pose.pose.position.y = y_pos;
    odom_msg.pose.pose.position.z = 0.0;
    odom_msg.pose.pose.orientation = odom_quat;
    if (rpm_act1 == 0 && rpm_act2 == 0){
      odom_msg.pose.covariance[0] = 1e-9;
      odom_msg.pose.covariance[7] = 1e-3;
      odom_msg.pose.covariance[8] = 1e-9;
      odom_msg.pose.covariance[14] = 1e6;
      odom_msg.pose.covariance[21] = 1e6;
      odom_msg.pose.covariance[28] = 1e6;
      odom_msg.pose.covariance[35] = 1e-9;
      odom_msg.twist.covariance[0] = 1e-9;
      odom_msg.twist.covariance[7] = 1e-3;
      odom_msg.twist.covariance[8] = 1e-9;
      odom_msg.twist.covariance[14] = 1e6;
      odom_msg.twist.covariance[21] = 1e6;
      odom_msg.twist.covariance[28] = 1e6;
      odom_msg.twist.covariance[35] = 1e-9;
    }
    else{
      odom_msg.pose.covariance[0] = 1e-3;
      odom_msg.pose.covariance[7] = 1e-3;
      odom_msg.pose.covariance[8] = 0.0;
      odom_msg.pose.covariance[14] = 1e6;
      odom_msg.pose.covariance[21] = 1e6;
      odom_msg.pose.covariance[28] = 1e6;
      odom_msg.pose.covariance[35] = 1e3;
      odom_msg.twist.covariance[0] = 1e-3;
      odom_msg.twist.covariance[7] = 1e-3;
      odom_msg.twist.covariance[8] = 0.0;
      odom_msg.twist.covariance[14] = 1e6;
      odom_msg.twist.covariance[21] = 1e6;
      odom_msg.twist.covariance[28] = 1e6;
      odom_msg.twist.covariance[35] = 1e3;
    }
    //vx = (dt == 0)?  0 : v_ave/dt;
    //vth = (dt == 0)? 0 : dth/dt;
    odom_msg.child_frame_id = base_link;
    odom_msg.twist.twist.linear.x = v_ave;
    odom_msg.twist.twist.linear.y = 0.0;
    odom_msg.twist.twist.angular.z = omega_odom;

    odom_pub.publish(odom_msg);
    last_time = current_time;
    r.sleep();
  }
}
