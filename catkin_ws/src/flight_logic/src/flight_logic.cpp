#include <ros/ros.h>
#include <stdio.h>
#include <cstdlib>

#include "flight_logic.h"
#include <std_msgs/UInt8.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Vector3.h>

using namespace std;
using namespace ros;

int wtffff = 1111; // dummy variable to make sure catkin_make is done correctly
#define ZIGZAG
//#define SQUARE

Publisher stm32_cmd_pub;
Publisher target_pos_pub;
Publisher cmd_msg_pub;
Publisher ugv_activation_pub;
Publisher trackingPID_pub;

MISSION_STATUS current_mission = INIT;

bool arrived = false;
float pos_error_threshold = 0.4; // 40 cm tolerance for horizontal & vertical positioning

float current_position[3] = {0, 0, 0};
float target_position[3] = {0, 0, 0};
float ugv_position[2] = {0, 0};

std_msgs::UInt8 cmd_msg;

bool is_arrived() { 

    /* Check whether the UAV has arrived at the target position */
    if( abs(target_position[0] - current_position[0]) < pos_error_threshold &&
        abs(target_position[1] - current_position[1]) < pos_error_threshold &&
	    abs(target_position[2] - current_position[2]) < pos_error_threshold  )
        return true;
    else 
        return false;

}

void ugv_pos_callback(const nav_msgs::Odometry& msg) {

    //cout << "ugv pos updated" << endl;
    ugv_position[0] = msg.pose.pose.position.x;
    ugv_position[1] = msg.pose.pose.position.y;

}

void wait_key() { 

    cin.get();
    cin.ignore();
}

// Delay for x seconds
void delay_s(int x) {

    ros::Duration(x).sleep();

}

void move_to(double x, double y, double z, bool wait_until_arrival = true) {

    target_position[0] = x;
    target_position[1] = y;
    target_position[2] = z;

    geometry_msgs::Vector3 target_pos;
    target_pos.x = x;
    target_pos.y = y;
    target_pos.z = z;

    int counter = 0;
    /* Publish 10 times */
    while(counter < 10) {
        
        target_pos_pub.publish(target_pos);
        counter++;

    }

    while( !is_arrived() && wait_until_arrival) {
	    ros::spinOnce();
    }

    return; 
}

void send_command(int cmd) {

    cmd_msg.data = cmd;
    for(int i = 0; i < 100; i++) {
        // publish 10 times
        cmd_msg_pub.publish(cmd_msg);
        i++;
    }

}

void current_pos_callback(const nav_msgs::Odometry& msg){

    current_position[0] = msg.pose.pose.position.x;
    current_position[1] = msg.pose.pose.position.y;
    //current_position[2] = msg.z;
    //cout << "Current position has been updated!" << endl;
}
void height_callback(const std_msgs::Float32& msg){

    current_position[2] = msg.data;

}

int mission_run() { 
    

    switch(current_mission) {

        case INIT: {

            ROS_INFO("Initializing...");
            delay_s(2); // Wait for 2 seconds

            send_command(INIT);
            ROS_INFO("Obtained control successfully");
            current_mission = TAKEOFF;

            delay_s(1); // Wait for 1 second, proceeding to next stage

            break;
        }
        

	    case TAKEOFF: {

            send_command(TAKEOFF);
            ROS_INFO("Taking off...");
            delay_s(3); // Taking off, wait for 3 seconds

            current_mission = SEARCH_FOR_TAGS; 
	        delay_s(1);

	        break; 
	    }
 
        case STAND_BY: {
 
            send_command(STAND_BY);
            current_mission = LANDING;
            delay_s(1);
       
            break;
        }



        case LANDING: {

            send_command(LANDING);
            ROS_INFO("Landing...");

            delay_s(3);
            send_command(RELEASE_CONTROL);

            break;
        }

        

        case SEARCH_FOR_TAGS: {

   	        send_command(SEARCH_FOR_TAGS);
	        delay_s(2);

            ROS_INFO("Searching for tags...");

            #ifdef SQUARE
            move_to(0, 0, 2.5);

            move_to(4, 0, 2.5);
 	        cout << "Arrived at first waypoint!" << endl;
            move_to(4, 5, 2.5);
	        cout << "Arrived at second waypoint!" << endl;
            move_to(0, 5, 2.5);
            cout << "Arrived at third waypoint!" << endl;
            move_to(0, 0, 2.5);
 	        cout << "Back to origin!" << endl;
            #endif

            #ifdef ZIGZAG
            move_to(0, 0, 2.5);
            printMap(0);

            move_to(2, 0, 2.5);
            printMap(1);

            move_to(2, 5, 2.5);
            printMap(2);

            move_to(4, 5, 2.5);
            printMap(3);

            move_to(4, 0, 2.5);
            printMap(4);

            move_to(6, 0, 2.5);
            printMap(5);

            move_to(6, 5, 2.5);
            printMap(6);

            move_to(0, 0, 2.5);
            cout << "Back to origin!" << endl;
            #endif


	        std_msgs::UInt8 msg;
	        msg.data = 3;
	        for(int i=0; i<20; i++) {
	    	   // Publish 20 times, wait for 100ms in total
	    	   ugv_activation_pub.publish(msg);
		       usleep(5000);
	        }

            current_mission = UGV_TRACKING;

            break;
        }

        case UGV_TRACKING: {
            cout << "UAV Position: " << endl << " x: " << current_position[0] << endl << "y: " << current_position[1] << endl << "z: " << current_position[2] << endl;
            cout << "UGV Position: " << endl << " x: " << ugv_position[0] << endl << "y: " << ugv_position[1] << endl;

            geometry_msgs::Vector3 msg;
            msg.x = 0.7;
            msg.y = 0.1;
            msg.z = 0.15;

	        for(int i=0; i<10; i++) 
	            trackingPID_pub.publish(msg);

            pos_error_threshold = 0.1;
            move_to(ugv_position[0], ugv_position[1], 2.5, false); // Won't wait until arrived

            break;
        }

        case RELEASE_CONTROL: {
            send_command(RELEASE_CONTROL);
            break;
        }
    }
}


int main(int argc, char **argv) {

    ros::init(argc, argv, "flight_logic");
    ros::NodeHandle nh;

    stm32_cmd_pub       = nh.advertise<std_msgs::String>("/stm32_cmd", 1);
    target_pos_pub      = nh.advertise<geometry_msgs::Vector3>("/target_position", 1);
    cmd_msg_pub         = nh.advertise<std_msgs::UInt8>("/sdk_cmd", 1);
    ugv_activation_pub  = nh.advertise<std_msgs::UInt8>("/ugv_activation", 1);
    trackingPID_pub     = nh.advertise<geometry_msgs::Vector3>("/pid_parameter", 1);

    Subscriber ugv_pos_sub         = nh.subscribe("/ekf/ekf_odom_ugv", 1, ugv_pos_callback);
    Subscriber current_pos_sub     = nh.subscribe("/ekf_odom_uav", 1, current_pos_callback);
    Subscriber height_sub          = nh.subscribe("/height", 1, height_callback);
    ros::Rate loop_rate(50);

    ROS_INFO("Flight Logic control starts!");
    cout << "Last modified: " << "2017-03-18" << endl;

    while(ros::ok()) {

        mission_run();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
