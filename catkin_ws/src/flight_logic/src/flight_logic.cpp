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

int wtffff = 1111; // dummy variable to make sure catkin_make is done correctly

MISSION_STATUS current_mission = INIT;
MISSION_STATUS next_mission;

bool vision_taking_control;
bool arrived = false;

float pos_error_threshold = 0.4; // 15 cm tolerance for horizontal & vertical positioning
float position_error[3] = {0, 0, 0};
float current_position[3] = {0, 0, 0};
float target_position[3] = {0, 0, 0};

float ugv_position[2] = {0, 0};

//bool marker_status[3] = {true, true, false}; // Origin, UGV, 1st target, respectively
map<int, bool> marker_status;
map<int, Tuple> marker_position;

int marker_id[3] = {10, 20, 30};

//float marker_position[3][2] = {{0, 0}, {0, 0}, {0, 0}}; // In local frame

std_msgs::UInt8 cmd_msg;
//double target_position[3] = {0, 0, 0};
//TODO Global target position and local target position

//TODO Add 2 Watchdogs for timing
Publisher target_pos_pub;     // Publish the target position to PID controller
Publisher cmd_msg_pub;        // Publish the control message (obtain control, takeoff, landing, etc.) to API
Publisher pid_param_pub;      // Publish the updated PID parameters set to PID controller
Publisher pid_ctrl_limit_pub; // Publish the updated velocity limit to PID controller 
Publisher stm32_cmd_pub;      // Publish the commands to STM32 transceiver node
Publisher ugv_activation_pub; // Publish the activation message to UGV
//ros::Subscriber vision_activation_sub;
//ros::Subscriber box_searcher_sub;
//ros::Subscriber target_searcher_sub; // Search for targets with specific tags
Publisher tracking_pub;
Subscriber ctrl_vel_sub;  // Subscribe the desired velocity from PID controller
Subscriber pos_error_sub; // Subscribe the flag for arrival at target position
Subscriber ugv_pos_sub; // Subscribe the current position of UGV from ekf node
Subscriber current_pos_sub; // Subscribe the current position from pid controller (originally from ekf and Guidance ultrasonic)
Subscriber detected_marker_sub; // Subscribe the detected markers and record their locations in local frame

bool is_arrived() { 

    /* Check whether the UAV has arrived at the target position */
    /*
    if ( abs(position_error[0]) < pos_error_threshold && 
         abs(position_error[1]) < pos_error_threshold) //&& 
         //abs(position_error[2]) < pos_error_threshold )
    */
    if( abs(target_position[0] - current_position[0]) < pos_error_threshold &&
        abs(target_position[1] - current_position[1]) < pos_error_threshold &&
	abs(target_position[2] - current_position[2]) < pos_error_threshold  )
        return true;
    else 
        return false;

}


void position_error_callback(const geometry_msgs::Vector3& msg) {

    position_error[0] = msg.x;
    position_error[1] = msg.y;
    position_error[2] = msg.z;

    //arrived = is_arrived();
    //cout << "Position error updated!" << endl;

}

void markerDetectorCallback(const nav_msgs::Odometry& msg) {

    int id = msg.twist.twist.linear.x;

    if(!marker_status[id]) { // First time detected this marker

	    switch(id) {

	        case 10: { // Marker at Origin
		
		        //TODO EKF odom UAV update

		        marker_status[id] = true;

	        break;
            }

	        case 20: { // Marker on UGV
		
		        //TODO EKF odom UGV update
	 	        float ugv_x = current_position[0] - msg.pose.pose.position.x;
		        float ugv_y = current_position[1] - msg.pose.pose.position.y;	
		        marker_status[id] = true;

	        break;
            }   

	        case 30: { // Marker of 1st target

		        float temp_x = current_position[0] - msg.pose.pose.position.x;
		        float temp_y = current_position[1] - msg.pose.pose.position.y;
		        marker_position[id].vec[0] = temp_x;		
		        marker_position[id].vec[1] = temp_y;		

		        marker_status[id] = true;

	        break;
            }

	    }
    }

    else { // Not the first time detected
	
	    switch(id) {

	       case 10: {
	           //TODO something
           
	           break;
            }

	       case 20: {
		       //TODO something
	           break;
            }

	       case 30: {

		       float updated_x = marker_position[id].vec[0] + msg.pose.pose.position.x;
		       float updated_y = marker_position[id].vec[1] + msg.pose.pose.position.y;

	           break;
            }

	   }
    }
}

void ugv_pos_callback(const nav_msgs::Odometry& msg) {

    //cout << "ugv pos updated" << endl;
    ugv_position[0] = msg.pose.pose.position.x;
    ugv_position[1] = msg.pose.pose.position.y;

}
void publishMarkerPosition() {

    //TODO publish all the detected markers' position to UGV


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

    //target_position[0] = x;
    //target_position[1] = y;
    //target_position[2] = z;
    
    //TODO: might cause problem of infinity loop, check for timeout?

    target_position[0] = x;
    target_position[1] = y;
    target_position[2] = z;

    int counter = 0;
    geometry_msgs::Vector3 target_pos;
    
    target_pos.x = x;
    target_pos.y = y;
    target_pos.z = z;

    /* Publish 10 times */
    while(counter < 10) {
        
        target_pos_pub.publish(target_pos);
        counter++;

    }

    while( !is_arrived() && wait_until_arrival) {

	    ros::spinOnce();
        // usleep(20000);
        //cout << "spinOnce done!" << endl;

    }
    //cout << "cur_pos: " << current_position[0] << " " << current_position[1] << " ,tar_pos: " << x << " " << y  << endl;
    return; 
}

void send_command(int cmd) {

    cmd_msg.data = cmd;

    for(int i = 0; i < 100; i++) {
        // publish 10 times
        cmd_msg_pub.publish(cmd_msg);
        i++;
    }
    //cout << "Message sent! " << cmd <<endl;
}

void current_pos_callback(const geometry_msgs::Vector3& msg) {

    current_position[0] = msg.x;
    current_position[1] = msg.y;
    current_position[2] = msg.z;

    //cout << "Current position has been updated!" << endl;
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
            //wait_key();

            send_command(INIT);

            //ROS_INFO("Obtained control successfully");
            current_mission = TAKEOFF;

            delay_s(1); // Wait for 1 second, proceeding to next stage

            break;
        }
        

	    case TAKEOFF: {
	    
 	    ROS_INFO("Press any key to takeoff");
            //wait_key();

            send_command(TAKEOFF);

            ROS_INFO("Taking off...");
            
            delay_s(3); // Taking off, wait for 6 seconds

            //current_mission = SEARCH_FOR_TAGS;

            current_mission = SEARCH_FOR_TAGS; 
	    delay_s(1); // Wait for 1 second, proceeding to next stage 
	    //current_mission = next_mission; 
	    break; 
	    }
 
        case STAND_BY: {
 
            send_command(STAND_BY);

            ROS_INFO("Next step: Set height to 1.2m");
            //ROS_INFO("Press any key to continue");
            //wait_key(); 
            move_to(0, 0, 1.2);

            ROS_INFO("Next step: Move to (2,0)");
            //ROS_INFO("Press any key to continue");
            //wait_key(); 
            move_to(2, 0, 1.2);

            ROS_INFO("Next step: Move to (2,2)");
            //ROS_INFO("Press any key to continue");
            //wait_key(); 
            move_to(2, 2, 1.2);

            ROS_INFO("Next step: Move to (0,2)");
            //ROS_INFO("Press any key to continue");
            //wait_key(); 
            move_to(0, 2, 1.2);

            ROS_INFO("Next step: Move to (0,0)");
            //ROS_INFO("Press any key to continue");
            //wait_key(); 
            move_to(0, 0, 1.2);
             
            ROS_INFO("Mission accomplished!");
            //ROS_INFO("Press any key to land");
            //wait_key(); 
            ROS_INFO("Landing...");
            delay_s(6); // Landing, wait for 6 seconds

            current_mission = LANDING;

            delay_s(1);
       
            break;
        }



        case LANDING: {

            ROS_INFO("Press any key to land");
            //wait_key();

            send_command(LANDING);
            ROS_INFO("Landing...");

            delay_s(3);

            send_command(RELEASE_CONTROL);
            //current_mission = next_mission;

            break;
        }

        

        case SEARCH_FOR_TAGS: {

   	        send_command(STAND_BY);
	        delay_s(2); // Wait for 2 seconds

            ROS_INFO("Searching for tags...");

            move_to(0, 0, 2);

            move_to(4, 0, 2);
 	        cout << "Arrived at first waypoint!" << endl;
            move_to(4, 5, 2);
	        cout << "Arrived at second waypoint!" << endl;
            move_to(0, 5, 2);
            cout << "Arrived at third waypoint!" << endl;
            move_to(0, 0, 2);
 	        cout << "Back to origin!" << endl;

	        std_msgs::UInt8 msg;
	        msg.data = 3;
	        for(int i=0; i<50; i++) {
	    	   // Publish 50 times, wait for 1 second in total
	    	   ugv_activation_pub.publish(msg);
		       usleep(20000);
	        }

            current_mission = UGV_TRACKING;

            break;
        }

        case UGV_TRACKING: {
            //cout << "UGV POS: " << ugv_position[0] << "  " << ugv_position[1] << endl;
            geometry_msgs::Vector3 msg;
            msg.x = 0.7;
            msg.y = 0.1;
            msg.z = 0.15;
	    for(int i=0; i<10; i++) 
	        tracking_pub.publish(msg);

            pos_error_threshold = 0.1;
            move_to(ugv_position[0], ugv_position[1], 2, false); // Don't wait until arrival
            //ros::spinOnce();//current_mission = LANDING;

            break;
        }
        case RELEASE_CONTROL: {

            break;
        }
    }




   

}


int main(int argc, char **argv) {

    ros::init(argc, argv, "flight_logic");
    
    ros::NodeHandle nh;

    //vision_taking_control = false;
    marker_status[marker_id[0]] = true;
    marker_status[marker_id[1]] = true;
    marker_status[marker_id[2]] = false;

    stm32_cmd_pub = nh.advertise<std_msgs::String>("/stm32_cmd", 1);
    target_pos_pub = nh.advertise<geometry_msgs::Vector3>("/target_position", 1);
    cmd_msg_pub = nh.advertise<std_msgs::UInt8>("/sdk_cmd", 1);
    ugv_activation_pub = nh.advertise<std_msgs::UInt8>("/ugv_activation", 1);
    //vision_activation_sub = nh.subscribe("/vision_activation", 1, vision_activation_callback);
    //target_searcher_sub = nh.subscribe("/target_coordinate", 1);
    tracking_pub = nh.advertise<geometry_msgs::Vector3>("/pid_parameter", 1);
    ugv_pos_sub = nh.subscribe("/ekf/ekf_odom_ugv", 1, ugv_pos_callback);
    pos_error_sub = nh.subscribe("/position_error", 1, position_error_callback);
    current_pos_sub = nh.subscribe("/current_position", 1, current_pos_callback);
    detected_marker_sub = nh.subscribe("/detected_markers", 10, markerDetectorCallback);
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
