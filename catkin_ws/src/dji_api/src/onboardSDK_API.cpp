#include <ros/ros.h>
#include <stdio.h>
#include <dji_sdk/dji_drone.h>
#include <cstdlib>
#include <stdlib.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include "flight_logic.h"
#include <std_msgs/UInt8.h>

using namespace DJI::onboardSDK;
using namespace std;

MISSION_STATUS mission_status = INIT;
   
unsigned char ctrl_flag;

ros::Subscriber api_ctrl_sub;
ros::Subscriber api_cmd_sub;
DJIDrone* drone;

void ctrl_vel_callback(const geometry_msgs::Vector3& ctrl_velocity) {

    if(mission_status == STAND_BY || mission_status == SEARCH_FOR_TAGS || mission_status == UGV_TRACKING){

        drone->attitude_control(0x4b, ctrl_velocity.x, ctrl_velocity.y, ctrl_velocity.z, 0);

        //cout << "Velocity_x, Velocity_y, Velocity_z" << endl << (double)ctrl_velocity.x << " " << (double)ctrl_velocity.y << " " << (double)ctrl_velocity.z << endl;
    }
    //else
    //    cout << "Drone is not in the STAND_BY mode" << endl; 
}

void sdk_cmd_callback(const std_msgs::UInt8& msg) {
    switch(msg.data){

        case INIT:
            // request for control
            if(drone->request_sdk_permission_control()){

                mission_status = TAKEOFF;
                cout << "Command sent: Obtain control" << endl;
            }

            else
                cout << "Request control failed" << endl;

            break;

        case TAKEOFF:
            // takeoff 
            if(drone->takeoff()){
                sleep(10); //Wait for completely takeoff
                mission_status = STAND_BY;
                cout << "Command sent: Takeoff" << endl;  
            }
            else
                cout << "Take off failed" << endl;
            break;

        case STAND_BY:
            // stand by
            mission_status = STAND_BY;
            break;

        case LANDING:
            // landing
            if(drone->landing()){
                mission_status = LANDING;
                cout << "Command sent: Landing" << endl;
            }
            else
                cout << "Landing failed" << endl;
            break;

        case RELEASE_CONTROL:
            /* release control ability*/
            if(drone->release_sdk_permission_control()){
                mission_status = RELEASE_CONTROL;
                cout << "Command sent: Release control" << endl;
            }
            else
                cout << "Release control failed" << endl;
            break;

        case SEARCH_FOR_TAGS:

            mission_status = SEARCH_FOR_TAGS;
            break;

        case UGV_TRACKING:
            mission_status = UGV_TRACKING;
            break;

        case 9:
            /* Test here */
            for(int i = 0; i < 100; i++) {

                drone->attitude_control(0x5b, 0.3, 0, 1.2, 0);
                usleep(50000);
            } 
            break;

        default:
            break;
    }
}



int main(int argc, char *argv[])
{
    int main_operate_code = 0;
    int temp32;

    bool valid_flag = false;
    bool err_flag = false;
    ros::init(argc, argv, "dji_client");
    ROS_INFO("dji_client_modified Start!");
    ros::NodeHandle nh;
    drone = new DJIDrone(nh);

    api_ctrl_sub = nh.subscribe("/ctrl_vel", 1, ctrl_vel_callback);
    api_cmd_sub  = nh.subscribe("/sdk_cmd",  1, sdk_cmd_callback);
   
    cout << "Publish done!" << endl;   
    cout << "Last modified: 2017-05-02" << endl;
    
    while(ros::ok()){
        ros::spinOnce();
    }

    return 0;
}
    

