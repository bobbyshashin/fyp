#include <ros/ros.h>
#include "pid.h"
#include <iostream>
#include <stdio.h>
#include <stdint.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <pthread.h>
#include <math.h>
#include "controlcan.h"
#include <Eigen/Eigen>

#include <ctime>
#include <cstdlib>
#include "unistd.h"
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

using namespace ros;

VCI_BOARD_INFO pInfo;

//#define DEBUG

// Wheel PID controllers
PID *pid1;
PID *pid2;
PID *pid3;
PID *pid4;
// Body speed
PID *pid_x;
PID *pid_y;
PID *pid_yaw;

float Kp1 = 0.6;
float Ki1 = 0.15;
float Kd1 = 0.1;

float Kp2 = 0.45;
float Ki2 = 0.1;
float Kd2 = 0.2;

float Kp_body;
float Ki_body;
float Kd_body;

int16_t targetWheelSpeed[4] = {0, 0, 0, 0};
int16_t currentWheelSpeed[4] = {0,0,0,0}; 

float targetBodyVelocity[3] = {0, 0, 0}; // Vx, Vy, Omega(yaw)
float currentBodyVelocity[3] = {0, 0, 0};

//In global frame
float yaw;
float initial_yaw;

float ctrl[4] = {0,0,0,0}; // control data sent to CAN
int threshold = 10;

float dt = 0.1;

float pid_ctrl_limit = 4000;
float integralLimit = 2000;

Publisher can_transmit_pub;
Publisher can_receive_pub;
Publisher pid_param_pub;
Publisher speed_pub;
Publisher error_pub;
Publisher control_pub;

//Subscriber keyboard_speed_sub;
Subscriber target_speed_sub;		// Get user input speed
Subscriber global_to_body_angle_sub;
Subscriber targetBodyVelocity_sub;	// From ugv logic
Subscriber sensorData_sub;		// From N3 SDK
Subscriber initial_angle_sub;

//geometry_msgs::Vector3 pid_param;
//geometry_msgs::Quaternion speed;
//geometry_msgs::Quaternion control;
//geometry_msgs::Quaternion error;
