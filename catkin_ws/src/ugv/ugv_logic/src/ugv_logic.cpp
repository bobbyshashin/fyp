#include <ros/ros.h>
#include <iostream>
#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include <ctime>
#include <cstdlib>

#include <std_msgs/UInt8.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include "ugv_logic.h"

using namespace std;
using namespace ros;

Publisher target_position_pub;
Publisher targetWheelSpeed_pub;
Publisher targetBodySpeed_pub;

MISSION_STATUS current_mission = TEST;

int current_target_index = 0;
bool activation_flag = false;
float initial_angle; // the angle from global frame to local frame
float local_yaw; // from local frame to body frame

/* The orientation to current target(marker) 
   from x-axis of local frame, 
   relative to UGV's current location in local frame 
   (unrelated to UGV's own orientation) */
float current_target_orientation;
float yaw_error; // Note: z-axis pointing downwards

float current_position[2] = {0, 0}; // Current position in local frame, subscribed from EKF node
float target_position[2] = {0, 0}; 
float targetBodySpeed[3] = {0, 0, 0};
int targetWheelSpeed[4] = {0, 0, 0, 0};
float marker_position[2][2] = {{0, 0}, {0, 0}};

int magic = 1;

void setWheelSpeed(int lf, int lb, int rb, int rf) { // Left forward, left backward, right backward, right forward

    targetWheelSpeed[0] = lf;
    targetWheelSpeed[1] = lb;
    targetWheelSpeed[2] = rb;
    targetWheelSpeed[3] = rf;

}

void setBodySpeed(float x, float y, float omega){ // X-axis, Y-axis, angular velocity (looking from above, clockwise as +)

    targetBodySpeed[0] = x;
    targetBodySpeed[1] = y;
    targetBodySpeed[2] = omega;

}

void publishWheelSpeed() {

    geometry_msgs::Quaternion msg;
 
    msg.x = targetWheelSpeed[0];
    msg.y = targetWheelSpeed[1];
    msg.z = targetWheelSpeed[2];
    msg.w = targetWheelSpeed[3];

    targetWheelSpeed_pub.publish(msg);

}

void publishBodySpeed(){

    geometry_msgs::Vector3 msg;

    msg.x = targetBodySpeed[0];
    msg.y = targetBodySpeed[1];
    msg.z = targetBodySpeed[2];

    targetBodySpeed_pub.publish(msg);
}

void go() {

    int left_speed_compensation;
    int right_speed_compensation;
    int multiplier;

    if(abs(yaw_error) > 2)
        multiplier = 2000;
    else if (abs(yaw_error) > 1)
        multiplier = 1500;
    else if (abs(yaw_error) > 0.1)
       multiplier = 1000;
    else
	   multiplier = 0;

    // Left wheels
    left_speed_compensation = - yaw_error * multiplier;
    // Right wheels
    right_speed_compensation = yaw_error * multiplier;

    int left = targetWheelSpeed[0] + left_speed_compensation;
    int right = targetWheelSpeed[2] + right_speed_compensation;

    setWheelSpeed(left, left, right, right);
    publishWheelSpeed();

}

void move_to() {

    geometry_msgs::Vector3 target_pos;

    target_pos.x = target_position[0];
    target_pos.y = target_position[1];
    target_pos.z = yaw_error;

    target_position_pub.publish(target_pos);

}

bool arrived() {

    return (abs(target_position[0] - current_position[0]) < 0.5) && (abs(target_position[1] - current_position[1]) < 0.5);

}

void marker_position_callback(const geometry_msgs::Vector3& msg) {

    int id = msg.z;

    if(id == 10) {
        marker_position[0][0] = msg.x;
        marker_position[0][1] = msg.y;
        cout << "First target position received: " << endl << "X: " << msg.x << endl << "Y: " << msg.y << endl;
    }
    else if(id == 40) {
        marker_position[1][0] = msg.x;
        marker_position[1][1] = msg.y;
        cout << "Second target position received: " << endl << "X: " << msg.x << endl << "Y: " << msg.y << endl;
    }

}

void current_position_callback(const nav_msgs::Odometry& msg) {

    current_position[0] = msg.pose.pose.position.x;
    current_position[1] = msg.pose.pose.position.y;
    
    if(current_target_index > 0) {

        float target_x = marker_position[current_target_index][0];
    	float target_y = marker_position[current_target_index][1];

    	if(target_x > 0 && target_y != 0)
    	    current_target_orientation = atan2((target_x - current_position[0]), (target_y - current_position[1]));
    	else
	       cout << "Current target's position is invalid!" << endl;
    
        yaw_error = local_yaw - current_target_orientation;
	    cout << "Yaw error: " << yaw_error << endl;
    }
}

void orientation_callback(const geometry_msgs::Vector3& msg) {

    local_yaw = msg.y - initial_angle;
}

void initial_angle_callback(const std_msgs::Float32& msg) {

    initial_angle = msg.data;
}

void activation_callback(const std_msgs::UInt8& msg) {

    if(msg.data == 3 && !activation_flag) {
        current_mission = GOING_TO_TARGET;
        cout << "UGV activated!" << endl;
	    activation_flag = true;
    }

}

void controlTest(){

    ROS_INFO("Chassis control test starts...");

    ROS_INFO("Set speed: (2, 0, 0)");
    setBodySpeed(2, 0, 0);
    publishBodySpeed();
    Duration(2).sleep();

    ROS_INFO("Set speed: (0, 2, 0)");
    setBodySpeed(0, 2, 0);
    publishBodySpeed();
    Duration(2).sleep();

    ROS_INFO("Set speed: (-2, 0, 0)");
    setBodySpeed(-2, 0, 0);
    publishBodySpeed();
    Duration(2).sleep();

    ROS_INFO("Set speed: (0, -2, 0)");
    setBodySpeed(0, -2, 0);
    publishBodySpeed();
    Duration(2).sleep();

    ROS_INFO("Stop");
    setBodySpeed(0, 0, 0);
    publishBodySpeed();
    Duration(2).sleep();

}

void mission_run() {

    switch(current_mission) {

	case INIT: 

	    break;

	case STAND_BY:

	    break;

	case GOING_TO_TARGET:

        target_position[0] = marker_position[current_target_index][0];
        target_position[1] = marker_position[current_target_index][1];

	    if( !arrived() ) {
 	        
	        setWheelSpeed(1500, 1500, 1500, 1500);
	        go();

	    }
	    else {
	    cout << "Arrived at first target!" << endl;
            if(current_target_index != 1){
                current_target_index++;
                target_position[0] = marker_position[current_target_index][0];
                target_position[1] = marker_position[current_target_index][1];
            }
            else
		        current_mission = ARRIVED;
	    }

	    break;

	case ARRIVED:

        setWheelSpeed(0, 0, 0, 0);
        publishWheelSpeed();

	    break;
    case TEST:

        controlTest();
        break;

    }

}


int main(int argc, char *argv[]) {

    ros::init(argc, argv, "ugv_logic");
    ros::NodeHandle nh;

    ros::Rate loop_rate(100);

    Subscriber current_position_sub = nh.subscribe("/ekf/ekf_odom_ugv", 1, current_position_callback);
    Subscriber orientation_sub = nh.subscribe("/n3_sdk/orientation", 1, orientation_callback);
    Subscriber initial_angle_sub = nh.subscribe("/initial_angle", 1, initial_angle_callback);
    Subscriber activation_sub = nh.subscribe("/ugv_activation", 1, activation_callback);
    Subscriber marker_position_sub = nh.subscribe("/marker_position", 1, marker_position_callback);

    targetWheelSpeed_pub = nh.advertise<geometry_msgs::Quaternion>("/target_wheel_speed", 1);
    targetBodySpeed_pub = nh.advertise<geometry_msgs::Vector3>("/ugv_targetBodyVelocity", 1);

    while (ros::ok()) {

        mission_run();
        ros::spinOnce();
        loop_rate.sleep();

    }

}
