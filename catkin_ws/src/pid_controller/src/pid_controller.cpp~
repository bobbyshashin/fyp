  /**
  ******************************************************************************
  * @file    pid_controller.cpp
  * @author  Bobby SHEN
  * @version V1.5.0
  * @date    27-April-2017
  * @brief   This is a PID controller node based on ROS, modified from gaowenliang's code
  *           
  ******************************************************************************  
  */ 

#include <iostream>
#include <stdio.h>
#include <ros/ros.h>
#include <dji_sdk/dji_drone.h>

#include <std_msgs/Float32.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Odometry.h>
#include "PID_lib/pid.h"

using namespace std;
using namespace ros;
using namespace DJI::onboardSDK;

int wtf = 12; // dummy variable to make sure catkin_make is done correctly

Publisher ctrl_vel_pub;
Publisher pos_error_pub;

Subscriber pid_parameter_sub;       // For tuning PID parameters (Kp, Ki and Kd)
Subscriber target_pos_sub;          // Target position  
Subscriber current_pos_sub;         // Current position (calculated from ekf)
Subscriber pid_ctrl_limit_sub;      // For tuning velocity limits 
Subscriber marker_center_sub;       // Centroid's coordinate of detected markers
Subscriber height_sub; // Subscribe UGV's height from ultrasonic sensor

PID *pid_x;
PID *pid_y;
PID *pid_z;
PID *pid_yaw;

double Kp_horz_pos = 0.8;
double Ki_horz_pos = 0;
double Kd_horz_pos = 0.2;

double Kp_vert_pos = 0.6;
double Ki_vert_pos = 0;
double Kd_vert_pos = 0.1;

double Kp_yaw;
double Ki_yaw;
double Kd_yaw;

int img_center[2] = {376, 240}; // x & y center of image pixels

float ctrl_data[4] = {0, 0, 0, 0}; // Velocity of x, y, z and yaw

float target_position[3] = {0, 0, 1.5};
float target_yaw = 0;

float current_position[3] = {0, 0, 0};
float dt = 0.02;
float first_time = 0.0;

double pid_ctrl_limit_horz = 0.4;
double pid_ctrl_limit_vert = 0.5;
double pid_yaw_limit = 0;


void delay_s(int x) { // delay in second

    ros::Duration(x).sleep();

}

void target_pos_callback(const geometry_msgs::Vector3& target_pos) {

    /* Ignore tiny differences within 1 cm */
    if( abs(target_pos.x) < 0.01 ) 
        target_position[0] = 0;
    else 
        target_position[0] = target_pos.x;


    if( abs(target_pos.y) < 0.01 ) 
        target_position[1] = 0;
    else 
        target_position[1] = target_pos.y;

    /* Limit the height */
    if( target_pos.z < 3.2 && target_pos.z > 0.1 ) 
        target_position[2] = target_pos.z;
    else 
        target_position[2] = target_position[2];

    /* Update the target position */
    pid_x->set_point(target_position[0]);
    pid_y->set_point(target_position[1]);
    pid_z->set_point(target_position[2]);
    pid_yaw->set_point(0);
 
}


void pid_update(geometry_msgs::Vector3 current_position) {

    /* PID position update */
    ctrl_data[0] = pid_x -> update(current_position.x, dt);
    ctrl_data[1] = pid_y -> update(current_position.y, dt);
    ctrl_data[2] = pid_z -> update(current_position.z, dt);
    ctrl_data[3] = 0; //yaw

    /* Control speed limiting */
    if (ctrl_data[0] > pid_ctrl_limit_horz)
        ctrl_data[0] = pid_ctrl_limit_horz;
    if (ctrl_data[0] < -pid_ctrl_limit_horz)
        ctrl_data[0] = -pid_ctrl_limit_horz;

    if (ctrl_data[1] > pid_ctrl_limit_horz)
        ctrl_data[1] = pid_ctrl_limit_horz;
    if (ctrl_data[1] < -pid_ctrl_limit_horz)
        ctrl_data[1] = -pid_ctrl_limit_horz;
    
    if (ctrl_data[2] > pid_ctrl_limit_vert)
        ctrl_data[2] = pid_ctrl_limit_vert;
    if (ctrl_data[2] < -pid_ctrl_limit_vert)
        ctrl_data[2] = -pid_ctrl_limit_vert;

    /* Publish the output control velocity from PID controller */
    geometry_msgs::Vector3 velocity;

    velocity.x = ctrl_data[0];
    velocity.y = ctrl_data[1];
    velocity.z = ctrl_data[2];

    ctrl_vel_pub.publish(velocity);


}

void current_pos_callback(const nav_msgs::Odometry& current_pos) {

    geometry_msgs::Vector3 pos_error;

    current_position[0] = current_pos.pose.pose.position.x;
    current_position[1] = current_pos.pose.pose.position.y;
    //current_position[2] = current_pos.pose.pose.position.z;
    
    pos_error.x = target_position[0] - current_position[0];
    pos_error.y = target_position[1] - current_position[1];
    pos_error.z = target_position[2] - current_position[2];

    pos_error_pub.publish(pos_error);

    //cout << "error_x -> " << pos_error.x << "      error_y -> " << pos_error.y << endl << "      error_z -> " << pos_error.z << endl;
    geometry_msgs::Vector3 current_position_vec;
    current_position_vec.x = current_position[0];
    current_position_vec.y = current_position[1];
    current_position_vec.z = current_position[2];

    pid_update(current_position_vec);

}

void uav_height_callback(const std_msgs::Float32& msg) {

    current_position[2] = msg.data;

}

void pid_parameter_tuning_callback(const geometry_msgs::Vector3& msg) {

   
    pid_x -> set_param(msg.x, msg.y, msg.z); //Set Kp, Ki & Kd
    pid_y -> set_param(msg.x, msg.y, msg.z); //Set Kp, Ki & Kd

    cout << "PID parameters for horizontal position control have been updated!" << endl;
    cout << "Kp: " << msg.x << endl;
    cout << "Ki: " << msg.y << endl;
    cout << "Kd: " << msg.z << endl;

}

void pid_parameter_vert_tuning_callback(const geometry_msgs::Vector3& msg) {

    pid_z -> set_param(msg.x, msg.y, msg.z); //Set Kp, Ki & Kd

    cout << "PID parameters for vertical position control have been updated!" << endl;

    cout << "Kp: " << msg.x << endl;
    cout << "Ki: " << msg.y << endl;
    cout << "Kd: " << msg.z << endl;

}

void pid_ctrl_limit_callback(const geometry_msgs::Vector3& msg) {

    pid_ctrl_limit_horz = msg.x;
    pid_ctrl_limit_vert = msg.y;
    
    cout << "Speed limit has been updated!" << endl;
    cout << "Horizontal: " << pid_ctrl_limit_horz << " m/s" << endl;
    cout << "Vertical: " << pid_ctrl_limit_vert << " m/s" << endl;
}

int main(int argc, char** argv) {

    ros::init(argc, argv, "pid_controller");
    ros::NodeHandle nh;

    pid_x   = new PID( Kp_horz_pos, Ki_horz_pos, Kd_horz_pos, -5, 5, -pid_ctrl_limit_horz, pid_ctrl_limit_horz, false);
    pid_y   = new PID( Kp_horz_pos, Ki_horz_pos, Kd_horz_pos, -5, 5, -pid_ctrl_limit_horz, pid_ctrl_limit_horz, false);
    pid_z   = new PID( Kp_vert_pos, Ki_vert_pos, Kd_vert_pos, -5, 5, -pid_ctrl_limit_vert, pid_ctrl_limit_vert, false);
    pid_yaw = new PID( Kp_yaw, Ki_yaw, Kd_yaw, -5, 5, -pid_yaw_limit, pid_yaw_limit, false);

    pid_x->set_point(target_position[0]);
    pid_y->set_point(target_position[1]);
    pid_z->set_point(target_position[2]);
    pid_yaw->set_point(target_yaw);

    ros::Rate loop_rate(100);

    target_pos_sub       = nh.subscribe("/target_position",  1, target_pos_callback);
    current_pos_sub      = nh.subscribe("/ekf/ekf_odom_uav", 1, current_pos_callback);

    pid_parameter_sub    = nh.subscribe("/pid_parameter",    1, pid_parameter_tuning_callback);

    pid_ctrl_limit_sub   = nh.subscribe("/pid_ctrl_limit",   1, pid_ctrl_limit_callback);

    marker_center_sub = nh.subscribe("/marker_centers", 1, vision_ctrl_callback);
    height_sub = nh.subscribe("/uav_height", 1, uav_height_callback);
    ctrl_vel_pub         = nh.advertise<geometry_msgs::Vector3>("/ctrl_vel", 10);
    pos_error_pub        = nh.advertise<geometry_msgs::Vector3>("/position_error", 1);

    cout << "PID controller activated!" << endl;
    cout << "Last modified: " << "2017-05-02" << endl;
    
    while(ros::ok()) {

        loop_rate.sleep();
        ros::spinOnce();
        
    }

    return 0;
}

