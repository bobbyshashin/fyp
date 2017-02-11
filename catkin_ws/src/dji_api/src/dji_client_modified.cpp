/** @file client.cpp
 *  @version 3.1.8
 *  @date July 29th, 2016
 *
 *  @brief
 *  All the exampls for ROS are implemented here. 
 *
 *  @copyright 2016 DJI. All rights reserved.
 *
 */



#include <ros/ros.h>
#include <stdio.h>
#include <dji_sdk/dji_drone.h>
#include <cstdlib>
#include <stdlib.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include "flight_logic.h"

using namespace DJI::onboardSDK;
using namespace std;
MISSION_STATUS mission_status = INIT;
   
unsigned char ctrl_flag;
ros::Subscriber api_ctrl_sub;
ros::Subscriber api_cmd_sub;
DJIDrone* drone;
/*
//! Function Prototypes for Mobile command callbacks - Core Functions
void ObtainControlMobileCallback(DJIDrone *drone);
void ReleaseControlMobileCallback(DJIDrone *drone);
void TakeOffMobileCallback(DJIDrone *drone);
void LandingMobileCallback(DJIDrone *drone);
void GetSDKVersionMobileCallback(DJIDrone *drone);
void ArmMobileCallback(DJIDrone *drone);
void DisarmMobileCallback(DJIDrone *drone);
void GoHomeMobileCallback(DJIDrone *drone);
void TakePhotoMobileCallback(DJIDrone *drone);
void StartVideoMobileCallback(DJIDrone *drone);
void StopVideoMobileCallback(DJIDrone *drone);
//! Function Prototypes for Mobile command callbacks - Custom Missions
void DrawCircleDemoMobileCallback(DJIDrone *drone);
void DrawSquareDemoMobileCallback(DJIDrone *drone);
void GimbalControlDemoMobileCallback(DJIDrone *drone);
void AttitudeControlDemoMobileCallback(DJIDrone *drone);
void LocalNavigationTestMobileCallback(DJIDrone *drone);
void GlobalNavigationTestMobileCallback(DJIDrone *drone);
void WaypointNavigationTestMobileCallback(DJIDrone *drone);
void VirtuaRCTestMobileCallback(DJIDrone *drone);

//! For LAS logging
void StartMapLASLoggingMobileCallback(DJIDrone *drone);
void StopMapLASLoggingMobileCallback(DJIDrone *drone);
void StartCollisionAvoidanceCallback(DJIDrone *drone);
void StopCollisionAvoidanceCallback(DJIDrone *drone);
*/
void sdk_ctrl_callback(const geometry_msgs::Vector3& ctrl_velocity) {
    if(mission_status == STAND_BY){
        drone->attitude_control(0x5b, ctrl_velocity.x, ctrl_velocity.y, 1.2, 0); //Notice here! Third one should be the height in this mode!!!
        cout << "Velocity_x, Velocity_y, Velocity_z" << endl << (double)ctrl_velocity.x << " " << (double)ctrl_velocity.y << " " << (double)ctrl_velocity.z << endl;
    }
    else
        cout << "Drone is not in the STAND_BY mode" << endl; 
}

void sdk_cmd_callback(const std_msgs::UInt8& msg) {
    switch(msg.data){
        case 1:
        /* request control ability*/
            if(drone->request_sdk_permission_control()){
                mission_status = TAKEOFF;
                cout << "Command sent: Obtain control" << endl;
            }
            else
                cout << "Request control failed" << endl;
            break;
        case 2:
            /* take off */
            if(drone->takeoff()){
                sleep(10); //Wait for completely takeoff
                mission_status = STAND_BY;
                cout << "Command sent: Takeoff" << endl;  
            }
            else
                cout << "Take off failed" << endl;
            break;
        case 999:
            /* Test here */
            for(int i = 0; i < 100; i++)
            {
                drone->attitude_control(0x5b, 0.3, 0, 1.2, 0);
                usleep(50000);
            } 
            break;
        case 4:
            /* landing*/
            if(drone->landing()){
                mission_status = LANDING;
                cout << "Command sent: Landing" << endl;
            }
            else
                cout << "Landing failed" << endl;
            break;
        case 5:
            /* release control ability*/
            if(drone->release_sdk_permission_control()){
                mission_status = RELEASE_CONTROL;
                cout << "Command sent: Release control" << endl;
            }
            else
                cout << "Release control failed" << endl;
            break;
        default:
            break;
    }
}
static void Display_Main_Menu(void)
{
    printf("\r\n");
    printf("+-------------------------- < Main menu > ------------------------+\n");
	printf("| [1]  SDK Version Query        | [20] Set Sync Flag Test          |\n");
	printf("| [2]  Request Control          | [21] Set Msg Frequency Test      |\n");	
	printf("| [3]  Release Control          | [22] Waypoint Mission Upload     |\n");	
	printf("| [4]  Takeoff                  | [23] Hotpoint Mission Upload     |\n");	
	printf("| [5]  Landing                  | [24] Followme Mission Upload     |\n");	
	printf("| [6]  Go Home                  | [25] Mission Start               |\n");	
	printf("| [7]  Gimbal Control Sample    | [26] Mission Pause               |\n");	
	printf("| [8]  Attitude Control Sample  | [27] Mission Resume              |\n");	
	printf("| [9]  Draw Circle Sample       | [28] Mission Cancel              |\n");	
	printf("| [10] Draw Square Sample       | [29] Mission Waypoint Download   |\n");	
	printf("| [11] Take a Picture           | [30] Mission Waypoint Set Speed  |\n");	
	printf("| [12] Start Record Video       | [31] Mission Waypoint Get Speed  |\n");	 
	printf("| [13] Stop Record Video        | [32] Mission Hotpoint Set Speed  |\n");	
	printf("| [14] Local Navigation Test    | [33] Mission Hotpoint Set Radius |\n");	
	printf("| [15] Global Navigation Test   | [34] Mission Hotpoint Reset Yaw  |\n");	
	printf("| [16] Waypoint Navigation Test | [35] Mission Followme Set Target |\n");	
	printf("| [17] Arm the Drone            | [36] Mission Hotpoint Download   |\n");	
	printf("| [18] Disarm the Drone         | [37] Enter Mobile commands mode  |\n");
    printf("| [19] Virtual RC Test           \n");
    printf("+-----------------------------------------------------------------+\n");
    printf("input 1/2/3 etc..then press enter key\r\n");
    printf("use `rostopic echo` to query drone status\r\n");
    printf("----------------------------------------\r\n");
}



int main(int argc, char *argv[])
{
    int main_operate_code = 0;
    int temp32;
    int x_center, y_center, yaw_local;
    bool valid_flag = false;
    bool err_flag = false;
    ros::init(argc, argv, "dji_client_modified");
    ROS_INFO("dji_client_modified Start!");
    ros::NodeHandle nh;
    drone = new DJIDrone(nh);
    api_ctrl_sub = nh.subscribe("sdk_ctrl", 1, sdk_ctrl_callback);
    api_cmd_sub  = nh.subscribe("sdk_cmd",  1, sdk_cmd_callback);
    /*
	//virtual RC test data
	uint32_t virtual_rc_data[16];

	//set frequency test data
	uint8_t msg_frequency_data[16] = {1,2,3,4,3,2,1,2,3,4,3,2,1,2,3,4};
	//waypoint action test data
    dji_sdk::WaypointList newWaypointList;
    dji_sdk::Waypoint waypoint0;
    dji_sdk::Waypoint waypoint1;
    dji_sdk::Waypoint waypoint2;
    dji_sdk::Waypoint waypoint3;
    dji_sdk::Waypoint waypoint4;

	//groundstation test data
	dji_sdk::MissionWaypointTask waypoint_task;
	dji_sdk::MissionWaypoint 	 waypoint;
	dji_sdk::MissionHotpointTask hotpoint_task;
	dji_sdk::MissionFollowmeTask followme_task;
	dji_sdk::MissionFollowmeTarget followme_target;
    uint8_t userData = 0;
    */
    ros::spinOnce();
    
    //! Setting functions to be called for Mobile App Commands mode 
    /*
    drone->setObtainControlMobileCallback(ObtainControlMobileCallback, &userData);
    drone->setReleaseControlMobileCallback(ReleaseControlMobileCallback, &userData);
    drone->setTakeOffMobileCallback(TakeOffMobileCallback, &userData);
    drone->setLandingMobileCallback(LandingMobileCallback, &userData);
    drone->setGetSDKVersionMobileCallback(GetSDKVersionMobileCallback, &userData);
    drone->setArmMobileCallback(ArmMobileCallback, &userData);
    drone->setDisarmMobileCallback(DisarmMobileCallback, &userData);
    drone->setGoHomeMobileCallback(GoHomeMobileCallback, &userData);
    drone->setTakePhotoMobileCallback(TakePhotoMobileCallback, &userData);
    drone->setStartVideoMobileCallback(StartVideoMobileCallback,&userData);
    drone->setStopVideoMobileCallback(StopVideoMobileCallback,&userData);
    drone->setDrawCircleDemoMobileCallback(DrawCircleDemoMobileCallback, &userData);
    drone->setDrawSquareDemoMobileCallback(DrawSquareDemoMobileCallback, &userData);
    drone->setGimbalControlDemoMobileCallback(GimbalControlDemoMobileCallback, &userData);
    drone->setAttitudeControlDemoMobileCallback(AttitudeControlDemoMobileCallback, &userData);
    drone->setLocalNavigationTestMobileCallback(LocalNavigationTestMobileCallback, &userData);
    drone->setGlobalNavigationTestMobileCallback(GlobalNavigationTestMobileCallback, &userData);
    drone->setWaypointNavigationTestMobileCallback(WaypointNavigationTestMobileCallback, &userData);
    drone->setVirtuaRCTestMobileCallback(VirtuaRCTestMobileCallback, &userData);

    drone->setStartMapLASLoggingMobileCallback(StartMapLASLoggingMobileCallback, &userData);
    drone->setStopMapLASLoggingMobileCallback(StopMapLASLoggingMobileCallback, &userData);
    drone->setStartCollisionAvoidanceCallback(StartCollisionAvoidanceCallback, &userData);
    drone->setStopCollisionAvoidanceCallback(StopCollisionAvoidanceCallback, &userData);
    */
	
    //Display_Main_Menu();
    while(0) //Change to while(1) when using it 
    {
        ros::spinOnce();
        std::cout << "Enter Input Val: ";
        while(!(std::cin >> temp32)){
            std::cin.clear();
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            std::cout << "Invalid input.  Try again: ";
	    }  

        if(temp32>0 && temp32<38)
        {
            main_operate_code = temp32;         
        }
        else
        {
            printf("ERROR - Out of range Input \n");
            Display_Main_Menu();
            continue;
        }
        switch(main_operate_code)
        {

            case 1:
                /* request control ability*/
                drone->request_sdk_permission_control();
                break;
            case 2:
                /* take off */
                drone->takeoff();
                break;
            case 3:
                for(int i = 0; i < 100; i++)
                {
                    drone->attitude_control(0x5b, 0.3, 0, 1.2, 0);
                    usleep(50000);
                } 
		        break;
            case 4:
                /* landing*/
                drone->landing();
                break;
            case 5:
                /* release control ability*/
                drone->release_sdk_permission_control();
                break;
            default:
                break;
        }
        main_operate_code = -1;
        Display_Main_Menu();
    }

    return 0;
}
    

