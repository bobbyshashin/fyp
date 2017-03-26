#include <ros/ros.h>
#include <iostream>
#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include <ctime>
#include <cstdlib>

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

Subscriber current_position_sub;
Subscriber orientation_sub;
Subscriber initial_angle_sub;
Subscriber activation_sub;

Publisher target_position_pub;
Publisher target_speed_pub;

MISSION_STATUS current_mission = GOING_TO_TARGET;

int current_target_index = 0;

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
int target_speed[4] = {0, 0, 0, 0};
//Vector<geometry_msgs::Point2f> marker_position;
geometry_msgs::Point marker_position[2];

void publish_speed() {

    geometry_msgs::Quaternion msg;
 
    msg.x = target_speed[0];
    msg.y = target_speed[1];
    msg.z = target_speed[2];
    msg.w = target_speed[3];

    target_speed_pub.publish(msg);

}

void go() {

    int left_speed_compensation;
    int right_speed_compensation;

    if( abs(yaw_error) > 0.1 ) {

	// Left wheels
	left_speed_compensation = - yaw_error * 1000;
	// Right wheels
	right_speed_compensation = yaw_error * 1000;
	//TODO
    }

    else {
    	
	left_speed_compensation = 0;
	right_speed_compensation = 0;
    }

    int left = target_speed[0] + left_speed_compensation;
    int right = target_speed[2] + right_speed_compensation;

    target_speed[0] = left;
    target_speed[1] = left;
    target_speed[2] = right;
    target_speed[3] = right;

    publish_speed();

}


void set_speed(int lf, int lb, int rb, int rf) { // Left forward, left backward, right backward, right forward

    target_speed[0] = lf;
    target_speed[1] = lb;
    target_speed[2] = rb;
    target_speed[3] = rf;

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

void current_position_callback(const nav_msgs::Odometry& msg) {

    current_position[0] = msg.pose.pose.position.x;
    current_position[1] = msg.pose.pose.position.y;
    
    if(current_target_index > 0) {

        float target_x = marker_position[current_target_index].x;
    	float target_y = marker_position[current_target_index].y;

    	if(target_x > 0 && target_y != 0)
    	    current_target_orientation = atan2((target_x - current_position[0]), (target_y - current_position[1]));
    	else
	    cout << "Current target's position is invalid!" << endl;
    
        yaw_error = local_yaw - current_target_orientation;

    }
}

void orientation_callback(const geometry_msgs::Vector3& msg) {

    local_yaw = msg.y - initial_angle;
    /* Test ***********
    local_yaw = msg.y;
    yaw_error = local_yaw;
    cout << "Yaw error: " << yaw_error << endl;
    *******Test ends */
}

void initial_angle_callback(const std_msgs::Float32& msg) {

    initial_angle = msg.data;

}

void activation_callback(const std_msgs::UInt8& msg) {

    current_mission = msg.data;

}

void mission_run() {

    switch(current_mission) {

	case INIT: 

	    break;

	case STAND_BY:

	    break;

	case GOING_TO_TARGET:

	    if( !arrived() ) {
 	        
	        set_speed(1500, 1500, 1500, 1500);
	        go();

	    }
	    else {

		set_speed(0, 0, 0, 0);
		publish_speed();
		current_mission = ARRIVED;
	    }

	    break;

	case ARRIVED:

	    break;







    }





}


int main(int argc, char *argv[]) {

    ros::init(argc, argv, "ugv_logic");
    ros::NodeHandle nh;

    ros::Rate loop_rate(100);	

    current_position_sub = nh.subscribe("/ekf/ekf_odom_ugv", 1, current_position_callback);
    orientation_sub = nh.subscribe("/n3_sdk/orientation", 1, orientation_callback);
    initial_angle_sub = nh.subscribe("/initial_angle", 1, initial_angle_callback);
    activation_sub = nh.subscribe("/ugv_activation", 1, activation_callback);

    target_speed_pub = nh.advertise<geometry_msgs::Quaternion>("/ugv_target_speed", 1);

    while (ros::ok()) {

    mission_run();
    ros::spinOnce();
    loop_rate.sleep();

    }

}
