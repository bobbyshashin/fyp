#include <stdio.h>
#include <cstdlib>

#include "dji_api.h"
#include "std_msgs/UInt8.h"
#include "dji_api/api_ctrl_data.h"
#include "dji_sdk/dji_drone.h"
#include "flight_logic.h"

using namespace std;
using namespace DJI::onboardSDK;
using namespace ros;

#define ROS_LOOP_RATE 50

unsigned char ctrl_flag;
ros::Subscriber api_ctrl_sub;
ros::Subscriber api_cmd_sub;
//ros::Subscriber api_ctrl_sub;
//dji_drone_api_ctrl send_data;
Ctrl_data ctrl_data;

MISSION_STATUS mission_status = INIT;

ros::Time frame_time;

DJIDrone* drone;

typedef bool (DJIDrone::*_takeoff_func)();
typedef bool (DJIDrone::*_landing_func)();
typedef bool (DJIDrone::*_obtain_ctrl_func)();
typedef bool (DJIDrone::*_release_ctrl_func)();
typedef bool (DJIDrone::*_api_ctrl_data_func)(unsigned char ctrl_flag, float x, float y, float z, float yaw);


_takeoff_func         takeoff_func;
_landing_func 	      landing_func;
_obtain_ctrl_func     obtain_ctrl_func;
_release_ctrl_func    release_ctrl_func;
_api_ctrl_data_func   api_ctrl_func;
//string frame_id;

/*
void sdk_api_ctrl_callback(const dji_api::api_ctrl_data::ConstPtr& msg)
{
    frame_time           = ros::Time::now();
    send_data.seq ++;
    send_data.stamp      = frame_time.toSec();
    send_data.frame_id   = "dji_api_ctrl_data";

    send_data.roll_or_x  = msg->ctrl_data.quaternion.x;
    send_data.pitch_or_y = msg->ctrl_data.quaternion.y;
    send_data.yaw        = msg->ctrl_data.quaternion.z;
    send_data.thr_z      = msg->ctrl_data.quaternion.w;

    send_data.ctrl_flag = 0;
    if(msg->level_frame == GROUND_LEVEL)
    {
        send_data.ctrl_flag |= 0x00;
        cout<<"level_frame GROUND_LEVEL"<<endl;
    }
    else if(msg->level_frame == BODY_LEVEL)
    {
        send_data.ctrl_flag |= 0x02;
        cout<<"level_frame BODY_LEVEL"<<endl;

    }

    /////////////////////////////////////////////////////////
    if(msg->torsion_frame == GROUND_TORSION)
    {
        send_data.ctrl_flag |= 0x00;
        cout<<"torsion_frame GROUND_TORSION"<<endl;

    }
    else if(msg->torsion_frame == BODY_TORSION)
    {
        send_data.ctrl_flag |= 0x01;
        cout<<"torsion_frame GROUND_TORSION"<<endl;

    }


    if(msg->horiz_mode == FMU_API_HORI_ATTI_TILT_ANG)
    {
        send_data.ctrl_flag |= 0x00;
        cout<<"horiz_mode FMU_API_HORI_ATTI_TILT_ANG"<<endl;

    }
    else if(msg->horiz_mode == FMU_API_HORI_VEL)
    {
        send_data.ctrl_flag |= 0x40;
        cout<<"horiz_mode FMU_API_HORI_VEL"<<endl;

    }
    else if(msg->horiz_mode == FMU_API_HORI_POS)
    {
        send_data.ctrl_flag |= 0x80;
        cout<<"horiz_mode FMU_API_HORI_POS"<<endl;

    }
    ////////////////////////////////////////////////////////
    if(msg->vert_mode == FMU_API_VERT_VEL)
    {
        send_data.ctrl_flag |= 0x00;
        cout<<"vert_mode FMU_API_VERT_VEL"<<endl;

    }
    else if(msg->vert_mode == FMU_API_VERT_POS)
    {
        send_data.ctrl_flag |= 0x10;
        cout<<"vert_mode FMU_API_VERT_POS"<<endl;

    }
    else if(msg->vert_mode == FMU_API_VERT_THR)
    {
        send_data.ctrl_flag |= 0x20;
        cout<<"vert_mode FMU_API_VERT_THR"<<endl;

    }
    ///////////////////////////////////////////////////////
    if(msg->yaw_mode == FMU_API_YAW_ANG)
    {
        send_data.ctrl_flag |= 0x00;
        cout<<"yaw_mode FMU_API_YAW_ANG"<<endl;

    }
    else if(msg->yaw_mode == FMU_API_YAW_RATE)
    {
        send_data.ctrl_flag |= 0x08;
        cout<<"yaw_mode FMU_API_YAW_RATE"<<endl;

    }


    (drone->*api_ctrl_func)(send_data.ctrl_flag, send_data.roll_or_x, send_data.pitch_or_y, send_data.thr_z, send_data.yaw);
      
    

    printf(" roll %f pitch %f yaw %f thz %f\n",  (float)send_data.roll_or_x, (float)send_data.pitch_or_y, (float)send_data.yaw, (float)(send_data.thr_z));

    printf("ctrl_flag: %x \n",send_data.ctrl_flag);


}
*/

void sdk_ctrl_callback(const geometry_msgs::Vector3& ctrl_velocity) {

    //TODO change the check condition
    // Check whether the drone is ready for mission
    if(mission_status != INIT && mission_status != TAKEOFF) {

        /* Control logic: 0x53 **
        **  x-axis: velocity   **
        **  y-axis: velocity   **
        **  z-axis: position   */

        (drone->*api_ctrl_func)(0x53, ctrl_velocity.x, ctrl_velocity.y, ctrl_velocity.z, 0); // lock yaw to be zero

        printf("Velocity_x %f Velocity_y %f Yaw %f Position_z %f\n",  (double)ctrl_velocity.x, (double)ctrl_velocity.y, 0.0, (double)ctrl_velocity.z);
    }

     
}


void sdk_cmd_callback(const std_msgs::UInt8& msg) {

    switch (msg.data) {

    //TODO: mission status mismatched

    case 0: // Obtain Control

        if( (drone->*obtain_ctrl_func)() ) {

            mission_status = TAKEOFF;
            cout << "Command sent: Obtain control" << endl;
        }

        break;

    case 1: // Takeoff

        if( (drone->*takeoff_func)() ) {

            mission_status =  STAND_BY;
            cout << "Command sent: Takeoff" << endl;
        }

        break;

    case 2: // Landing

        if( (drone->*landing_func)() ) {

            mission_status = LANDING;
            cout<< "Command sent: Landing" << endl;
        }

        break;

    case 3: // Release Control

        if( (drone->*release_ctrl_func)() ) {

            mission_status = RELEASE_CONTROL;
            cout<< "Command sent: Release control" << endl;
        }

        break;
    }

}




int main(int argc, char **argv) {

    ros::init(argc, argv, "dji_api");
    ros::NodeHandle nh;
    drone = new DJIDrone(nh);

    api_ctrl_sub = nh.subscribe("sdk_ctrl", 1, sdk_ctrl_callback);
    api_cmd_sub  = nh.subscribe("sdk_cmd",  1, sdk_cmd_callback);
    //api_ctrl_sub = n.subscribe("sdk_api_ctrl", 1, sdk_api_ctrl_callback);
    ros::Rate(ROS_LOOP_RATE);


    takeoff_func      = &DJIDrone::takeoff;
    landing_func      = &DJIDrone::landing;
    obtain_ctrl_func  = &DJIDrone::request_sdk_permission_control;
    release_ctrl_func = &DJIDrone::release_sdk_permission_control;
    api_ctrl_func     = &DJIDrone::attitude_control;

    frame_time = ros::Time::now();

    cout << "ver 1.0.1"<<endl;

    while(nh.ok()) {
        
        ros::spinOnce();

    }
    return 0;
}
