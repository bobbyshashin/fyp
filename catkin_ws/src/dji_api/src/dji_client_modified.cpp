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
ros::Publisher guidance_bias_correction_pub;
ros::Subscriber api_ctrl_sub;
ros::Subscriber api_cmd_sub;
DJIDrone* drone;

void ctrl_vel_callback(const geometry_msgs::Vector3& ctrl_velocity) {

    if(mission_status == STAND_BY){

        drone->attitude_control(0x4b, ctrl_velocity.x, ctrl_velocity.y, ctrl_velocity.z, 0);

        //cout << "Velocity_x, Velocity_y, Velocity_z" << endl << (double)ctrl_velocity.x << " " << (double)ctrl_velocity.y << " " << (double)ctrl_velocity.z << endl;
    }

    //else
    //    cout << "Drone is not in the STAND_BY mode" << endl; 
}

void sdk_cmd_callback(const std_msgs::UInt8& msg) {
    switch(msg.data){

        case INIT:
            /* request control ability*/
	    //drone->request_sdk_permission_control();

            if(drone->request_sdk_permission_control()){

                mission_status = TAKEOFF;
                cout << "Command sent: Obtain control" << endl;
            }

            else
                cout << "Request control failed" << endl;

            break;

        case TAKEOFF:
            /* take off */
            if(drone->takeoff()){
                sleep(10); //Wait for completely takeoff
                mission_status = STAND_BY;
                cout << "Command sent: Takeoff" << endl;  
            }
            else
                cout << "Take off failed" << endl;
            break;

        case STAND_BY:
            /* stand by */
            mission_status = STAND_BY;
            break;

        case LANDING:
            /* landing*/
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
static void Display_Main_Menu(void) {
    printf("\r\n");
    printf("+-------------------------- < Main menu > ------------------------+\n");
	printf("| [1]  Request Control        | [20] Set Sync Flag Test          |\n");
	printf("| [2]  Release Control          | [21] Set Msg Frequency Test      |\n");	
	printf("| [3]  Takeoff          | [22] Waypoint Mission Upload     |\n");	
	printf("| [4]  Landing                  | [23] Hotpoint Mission Upload     |\n");	
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
    //int x_center, y_center, yaw_local;
    bool valid_flag = false;
    bool err_flag = false;
    ros::init(argc, argv, "dji_client");
    ROS_INFO("dji_client_modified Start!");
    ros::NodeHandle nh;
    drone = new DJIDrone(nh);

    guidance_bias_correction_pub = nh.advertise<std_msgs::UInt8>("/guidance/bias_correction", 1);
    api_ctrl_sub = nh.subscribe("/ctrl_vel", 1, ctrl_vel_callback);
    api_cmd_sub  = nh.subscribe("/sdk_cmd",  1, sdk_cmd_callback);
    
    std_msgs::UInt8 guidance_bias_correction;
    guidance_bias_correction.data = 1;
    for(int j=0;j<100;j++) {
        guidance_bias_correction_pub.publish(guidance_bias_correction);
}
    cout << "Publish done!" << endl;   

    
    while(ros::ok()){
	//guidance_bias_correction_pub.publish(guidance_bias_correction);
	ros::spinOnce();
	}
	
    
    //ros::spin();
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
    

