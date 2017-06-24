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
PID *pidVel1;
PID *pidVel2;
PID *pidVel3;
PID *pidVel4;
// Body yaw
PID *pidYaw1;
PID *pidYaw2;
PID *pidYaw3;
PID *pidYaw4;

float Kp1 = 0.6;
float Ki1 = 0.15;
float Kd1 = 0.1;

float Kp2 = 0.45;
float Ki2 = 0.1;
float Kd2 = 0.2;

float pid_ctrl_limit = 4000;
float integralLimit = 2000;

float Kp_vel = 500;
float Ki_vel = 100;
float Kd_vel = 100;

float Kp_yaw = 5000;
float Ki_yaw = 1000;
float Kd_yaw = 1500;

int16_t targetWheelSpeed[4] = {0, 0, 0, 0};
int16_t currentWheelSpeed[4] = {0,0,0,0}; 

float targetBodyVelocity[3] = {0, 0, 0}; // Vx, Vy, Omega(yaw)
float currentBodyVelocity[3] = {0, 0, 0};

//In global frame
float yaw = 0;
float initial_yaw = 0;
//float target_yaw = 0;
bool firstYaw = false;

float ctrl[4] = {0,0,0,0}; // control data sent to CAN
int threshold = 10;

float dt = 0.1;

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
