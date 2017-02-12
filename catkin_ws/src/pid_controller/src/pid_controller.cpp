  /**
  ******************************************************************************
  * @file    pid_controller.cpp
  * @author  Bobby SHEN 
  * @version V1.2.0
  * @date    06-September-2016
  * @brief   This is a PID controller node based on ROS, modified from gaowenliang's code
  *           
  ******************************************************************************  
  */ 

#include <iostream>
#include <stdio.h>
#include <ros/ros.h>
#include <dji_sdk/dji_drone.h>

#include <std_msgs/UInt8.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Vector3.h>

#include "PID_lib/pid.h"

using namespace std;
using namespace ros;
using namespace DJI::onboardSDK;

ros::Publisher ctrl_vel_pub;
ros::Publisher is_arrived_pub;

ros::Subscriber pid_parameter_sub;       // For tuning PID parameters (Kp, Ki and Kd)
ros::Subscriber target_pos_sub;          // Target position  
ros::Subscriber current_pos_sub;         // Current position (calculated from sensor data)
ros::Subscriber pid_ctrl_limit_sub;      // For tuning velocity limits 

PID *pid_x;
PID *pid_y;
PID *pid_z;
PID *pid_yaw;

//DJIDrone* drone;

double Kp_horz_pos = 0.8;
double Ki_horz_pos = 0;
double Kd_horz_pos = 0.2;

double Kp_vert_pos = 0.6;
double Ki_vert_pos = 0;
double Kd_vert_pos = 0;

double Kp_yaw;
double Ki_yaw;
double Kd_yaw;

float ctrl_data[4] = {0,0,0,0}; // Velocity of x, y, z and yaw
float target_position[3] = {0,0,0.6};
float target_yaw = 0;

float dt = 0.02;
float first_time = 0.0;

double pid_ctrl_limit_horz = 0.3;
double pid_ctrl_limit_vert = 0.3;
double pid_yaw_limit = 1;

bool is_arrived;
double pos_error_threshold = 0.05; // 5 cm tolerance for horizontal positioning

void delay_s(int x) {

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


    if( target_pos.z < 2 && target_pos.z > 0.1 ) 
        
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

    float error_x, error_y, error_z;

    error_x = target_position[0] - current_position.x;
    error_y = target_position[1] - current_position.y;
    error_z = target_position[2] - current_position.z;

    /* Check whether the UAV has arrived at the target position */
    if ( (abs(error_x) < pos_error_threshold) && abs(error_y) < pos_error_threshold && abs(error_z) < pos_error_threshold ) 
        is_arrived = true;
    else 
        is_arrived = false;

    std_msgs::Bool msg;
    msg.data = is_arrived;

    int i = 0;
    while(i < 4) {

        is_arrived_pub.publish(msg);
        i++;
    }


    /* Print out the current error in position */
    cout << "error_x -> " << error_x << "      error_y -> " << error_y << endl << "      error_z -> " << error_z << endl;

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

void current_pos_callback(const geometry_msgs::Vector3& current_position) {

    pid_update(current_position);
    //cout << target_position[0] - current_position.x << endl;
    //cout<< "Current position has been updated" <<endl;

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

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pid_controller");
    ros::NodeHandle nh("~");
    std_msgs::UInt8 bias_correction_msg;

    //drone = new DJIDrone(nh);

    pid_x   = new PID( Kp_horz_pos, Ki_horz_pos, Kd_horz_pos, -5, 5, -pid_ctrl_limit_horz, pid_ctrl_limit_horz, false);
    pid_y   = new PID( Kp_horz_pos, Ki_horz_pos, Kd_horz_pos, -5, 5, -pid_ctrl_limit_horz, pid_ctrl_limit_horz, false);
    pid_z   = new PID( Kp_vert_pos, Ki_vert_pos, Kd_vert_pos, -5, 5, -pid_ctrl_limit_vert, pid_ctrl_limit_vert, false);
    pid_yaw = new PID( Kp_yaw, Ki_yaw, Kd_yaw, -5, 5, -pid_yaw_limit, pid_yaw_limit, false);

    pid_x->set_point(target_position[0]);
    pid_y->set_point(target_position[1]);
    pid_z->set_point(target_position[2]);
    pid_yaw->set_point(target_yaw);

    ros::Rate loop_rate(50);

    target_pos_sub       = nh.subscribe("/target_position",  1, target_pos_callback);
    current_pos_sub      = nh.subscribe("/current_position", 1, current_pos_callback);

    pid_parameter_sub    = nh.subscribe("/pid_parameter",    1, pid_parameter_tuning_callback);
    //pid_parameter_vert_sub = nh.subscribe("/pid_vert_parameter",    1, pid_parameter_vert_tuning_callback);
    //TODO Union both horz and vert pid param into one subscriber or do a better renaming with two
    pid_ctrl_limit_sub   = nh.subscribe("/pid_ctrl_limit",   1, pid_ctrl_limit_callback);

    ctrl_vel_pub         = nh.advertise<geometry_msgs::Vector3>("/ctrl_vel", 10);
    is_arrived_pub       = nh.advertise<std_msgs::Bool>("/is_arrived", 1);

    cout<< "PID controller has been started!"<<endl;
    
    while(ros::ok())
    {

        loop_rate.sleep();
        ros::spinOnce();
        
    }

    return 0;
}

