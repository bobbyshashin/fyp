#include <ros/ros.h>
#include <stdio.h>
#include <cstdlib>

#include "flight_logic.h"

#include <std_msgs/UInt8.h>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Vector3.h>

//using namespace DJI::onboardSDK;
using namespace std;
using namespace ros;

MISSION_STATUS current_mission = INIT;
MISSION_STATUS next_mission;

bool vision_taking_control;

std_msgs::UInt8 cmd_msg;
//double target_position[3] = {0, 0, 0};
//TODO Global target position and local target position

//TODO Add 2 Watchdogs for timing
ros::Publisher target_pos_pub;     // Publish the target position to PID controller
ros::Publisher cmd_msg_pub;        // Publish the control message (obtain control, takeoff, landing, etc.) to API
ros::Publisher pid_param_pub;      // Publish the updated PID parameters set to PID controller
ros::Publisher pid_ctrl_limit_pub; // Publish the updated velocity limit to PID controller 
ros::Publisher stm32_cmd_pub;      // Publish the commands to STM32 transceiver node

//ros::Subscriber vision_activation_sub;
ros::Subscriber box_searcher_sub;
//ros::Subscriber target_searcher_sub; // Search for targets with specific tags
ros::Subscriber ctrl_vel_sub;  // Subscribe the desired velocity from PID controller

/*
bool is_arrived() { 

    if( position_error[0] > 0.15 ||
        position_error[1] > 0.15 ||
        position_error[2] > 0.15 )
        return false;

    else
        return true;

}
*/

void wait_key() { 

    cin.get();
    cin.ignore();
}

// Delay for x seconds
void delay_s(int x) {

    ros::Duration(x).sleep();

}

void move_to(double x, double y, double z) {

    //target_position[0] = x;
    //target_position[1] = y;
    //target_position[2] = z;
    /*
    while( !is_arrived() ) {

        usleep(2000);

    }
    */
    int counter = 0;
    geometry_msgs::Vector3 target_pos;
    
    target_pos.x = x;
    target_pos.y = y;
    target_pos.z = z;

    /* Publish 5 times */
    while(counter < 5) {
        
        target_pos_pub.publish(target_pos);
        counter++;

    }
}

void send_command(int cmd) {

	cmd_msg.data = cmd;

    for(int i = 0; i < 10; i++) {
        // publish 10 times
        cmd_msg_pub.publish(cmd_msg);
        i++;
    }

}


/*
void vision_activation_callback(const std_msgs::UInt32& msg) {

    geometry_msgs::Vector3 pid_param;
    geometry_msgs::Vector3 pid_ctrl_limit;

    if(msg == 1) {

        // PID parameters for vision positioning 
        //TODO To be adjusted
        pid_param.x = 0; // Kp
        pid_param.y = 0; // Ki
        pid_param.z = 0; // Kd

        pid_ctrl_limit.x = 0;
        pid_ctrl_limit.y = 0;

        vision_taking_control = true;   


    } 

    else if(msg == 0) {
        
        // PID parameters for normal positioning

        //TODO To be adjusted
        pid_param.x = 0; // Kp
        pid_param.y = 0; // Ki
        pid_param.z = 0; // Kd

        pid_ctrl_limit.x = 0;
        pid_ctrl_limit.y = 0;

        vision_taking_control = false;


    }

    pid_param_pub.publish(pid_param);
    pid_ctrl_limit_pub.publish(pid_ctrl_limit); 


}
*/

int mission_run() { 
    

    switch(current_mission) {

        case INIT: {

            ROS_INFO("Initializing...");
            delay_s(2); // Wait for 2 seconds

            ROS_INFO("Press any key to request for control");
            wait_key();

            send_command(INIT);

            ROS_INFO("Obtained control successfully");
            current_mission = TAKEOFF;

            break;
        }
        

	    case TAKEOFF: {
	    
 	        ROS_INFO("Press any key to takeoff");
            wait_key();

            send_command(TAKEOFF);

            ROS_INFO("Taking off...");
            
            delay_s(6); // Taking off, wait for 6 seconds

            current_mission = STAND_BY;
            //current_mission = next_mission;

            break;
        }
        

        case STAND_BY: {
 
            send_command(STAND_BY);

            ROS_INFO("Next step: Set height to 1.2m");
            ROS_INFO("Press any key to continue");
            wait_key(); 
            move_to(0, 0, 1.2);

            ROS_INFO("Next step: Move to (2,0)");
            ROS_INFO("Press any key to continue");
            wait_key(); 
            move_to(2, 0, 1.2);

            ROS_INFO("Next step: Move to (2,2)");
            ROS_INFO("Press any key to continue");
            wait_key(); 
            move_to(2, 2, 1.2);

            ROS_INFO("Next step: Move to (0,2)");
            ROS_INFO("Press any key to continue");
            wait_key(); 
            move_to(0, 2, 1.2);

            ROS_INFO("Next step: Move to (0,0)");
            ROS_INFO("Press any key to continue");
            wait_key(); 
            move_to(0, 0, 1.2);
             
            ROS_INFO("Mission accomplished!");
            ROS_INFO("Press any key to land");
            wait_key(); 
            ROS_INFO("Landing...");
            usleep(6000000); // Landing, wait for 6 seconds

            current_mission = LANDING;
       
            break;
        }



        case LANDING: {

            ROS_INFO("Press any key to land");
            wait_key();

            send_command(LANDING);
            ROS_INFO("Landing...");

            delay_s(4);

            send_command(RELEASE_CONTROL);
            //current_mission = next_mission;

            break;
        }

        

        case GO_HOME: {

        	ROS_INFO("Going home...");

        	move_to(0, 0, 1.2);

        	current_mission = LANDING;

        	break;
        }

    }




   

}

int main(int argc, char **argv) {

    ros::init(argc, argv, "flight_logic");
    
    ros::NodeHandle nh;

    //vision_taking_control = false;

    stm32_cmd_pub = nh.advertise<std_msgs::String>("/stm32_cmd", 1);
    target_pos_pub = nh.advertise<geometry_msgs::Vector3>("/target_position", 1);
    cmd_msg_pub = nh.advertise<std_msgs::UInt8>("/sdk_cmd", 10);
    //vision_activation_sub = nh.subscribe("/vision_activation", 1, vision_activation_callback);
    //target_searcher_sub = nh.subscribe("/target_coordinate", 1);

    ros::Rate loop_rate(50);
    ROS_INFO("Flight Logic: Flow control starts");

    while(ros::ok()) {

        mission_run();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
