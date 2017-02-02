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
        position_error[2] > 0.15  )
        return false;

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
    cmd_msg_pub.publish(cmd_msg);

}

//void velocity_callback(const geometry_msgs::Vector3& velocity) {

    /* Control logic: 0x51 **
    **  x-axis: velocity   **
    **  y-axis: velocity   **
    **  z-axis: position   */

    //drone->attitude_control(0x59, velocity.x, velocity.y, velocity.z, 0); 

//}

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

            cmd_msg.data = INIT;
            cmd_msg_pub.publish(cmd_msg);
	   
            ROS_INFO("Obtained control successfully");
            current_mission = TAKEOFF;

            break;
        }
        

        case COLLECTING_OCTOPUS: {

            std_msgs::String cmd;
            cmd.data = "Grab";
            stm32_cmd_pub.publish(cmd);
            delay_s(5); // Wait for 5 seconds

            ROS_INFO("Ready for takeoff");
            current_mission = TAKEOFF;
            next_mission = ZONE4_BOX_AIMING;
            break;
        }

	    case TAKEOFF: {

            cmd_msg.data = TAKEOFF;

            int i;
            while(i<10) { 

                cmd_msg_pub.publish(cmd_msg);
                i++;
            }

	        ROS_INFO("Press any key to takeoff");
            wait_key();
  
            ROS_INFO("Taking off...");
            
            delay_s(6); // Taking off, wait for 6 seconds

            current_mission = STAND_BY;
            //current_mission = next_mission;

            break;
        }
        

        case STAND_BY: {
 
            cmd_msg.data = STAND_BY;
            cmd_msg_pub.publish(cmd_msg);

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


	    

        case ZONE4_BOX_AIMING: {

            ROS_INFO("Octopus Bomber ready for action!");
            ROS_INFO("Press any key to start mission");
            wait_key(); 

            ROS_INFO("Mission starts!");
   
            ROS_INFO("Moving to ZONE 4!");
            move_to(6, 0, 1.5);

            //Waiting for vision:
            while( !vision_taking_control ) {
            
                //TODO check for timeout
                
            }


            ROS_INFO("Vision-based Positioning System taking control!");

            //TODO Vision PID feedback: Small overshoot & Move slowly with small pitch angle
            //TODO LED screen monitoring starts at the same time
            current_mission = RELEASING_OCTOPUS;

	        break;
        }
        

        case RELEASING_OCTOPUS: {
            
            ROS_INFO("Ready for bombing!");
            ROS_INFO("Press any key to release octopus");
            wait_key(); 

            std_msgs::String cmd;
            cmd.data = "Release";
            stm32_cmd_pub.publish(cmd);
            
            ROS_INFO("Octopus released");

            current_mission = ZONE3_BOX_AIMING;

	        break;
        }
        
        
        case ZONE3_BOX_AIMING: {


            ROS_INFO("Mission Code: Hippo");
            ROS_INFO("Press any key to start mission");
            wait_key(); 

            ROS_INFO("Mission starts!");
   
            ROS_INFO("Moving to ZONE 3!");
            move_to(6, -6, 1.5);

            while( !vision_taking_control ) {
            
                //TODO check for timeout
                
            }
            //TODO Once boxes are detected:
            ROS_INFO("Vision-based Positioning System taking control!");

            //TODO Vision PID feedback: Small overshoot & smooth movement with a small pitch angle


            current_mission = LANDING;
            next_mission = COLLECTING_HIPPO;

            break;
        }

        


        case COLLECTING_HIPPO: {

            ROS_INFO("Ready for hippo collection");
            ROS_INFO("Press any key to grab the hippos");
            wait_key(); 

            std_msgs::String cmd;
            cmd.data = "Grab";
            stm32_cmd_pub.publish(cmd);
            //TODO vision feedback: sucessfully grabbed?

            current_mission = TAKEOFF;
            next_mission = ZONE2_BOX_AIMING;
            
	        break;
        }
        

	    case ZONE2_BOX_AIMING: {

            ROS_INFO("Hippo Bomber ready for action!");
            ROS_INFO("Press any key to start mission");
            wait_key(); 

            ROS_INFO("Mission starts!");

            ROS_INFO("Moving to ZONE 2!");
            move_to(0, -6, 1.5);
            
            while( !vision_taking_control ) {
            
                //TODO check for timeout
                
            }


            ROS_INFO("Vision-based Positioning System taking control!");
            current_mission = RELEASING_HIPPO;

            break;
        }
        

        case RELEASING_HIPPO: {

            
            ROS_INFO("Ready for boming!!");
            ROS_INFO("Press any key to release hippo");
            wait_key(); 

            std_msgs::String cmd;
            cmd.data = "Release";
            stm32_cmd_pub.publish(cmd);
 
            ROS_INFO("Hippo released");

            //TODO Go back to zone 3 to grab the remaining hippos
            current_mission = ZONE3_BOX_AIMING;

            break; 
        }
        


        case LANDING: {

            cmd_msg.data = LANDING;
            int i;
            while(i<10) {

                cmd_msg_pub.publish(cmd_msg);
                i++;

            }
            ROS_INFO("Landing...");

            delay_s(4);

            ROS_INFO("Press any key to release control");
            wait_key();

            //cmd_msg.data = RELEASE_CONTROL;
            //cmd_msg_pub.publish(cmd_msg);
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

    vision_taking_control = false;

    stm32_cmd_pub = nh.advertise<std_msgs::String>("/stm32_cmd", 1);
    target_pos_pub = nh.advertise<geometry_msgs::Vector3>("/target_position", 1);
    cmd_msg_pub = nh.advertise<std_msgs::UInt8>("/sdk_cmd", 10);
    //vision_activation_sub = nh.subscribe("/vision_activation", 1, vision_activation_callback);
    //target_searcher_sub = nh.subscribe("/target_coordinate", 1);

    ros::Rate loop_rate(50);
    ROS_INFO("Flight Logic: Flow control starts");

    while( ros::ok() ) {

        mission_run();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}