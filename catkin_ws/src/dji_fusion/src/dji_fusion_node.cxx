//TODO class
#include <iostream>
#include <stdio.h>
#include <ros/ros.h>

#include <std_msgs/UInt8.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseStamped.h>
#include "dji_fusion.h"

//include opencv libraries
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#define pi 3.14159265
using namespace std;
using namespace ros;
using namespace cv;

ros::Publisher target_pos_pub;
ros::Publisher vision_activation_pub;
ros::Publisher release_pub;
ros::Publisher current_pos_pub;
ros::Publisher odometry_pub;
ros::Publisher guidance_correct_pub;
ros::Publisher pid_param_pub;              // Publish the updated PID parameters set to PID controller
ros::Publisher pid_ctrl_limit_pub;         // Publish the updated velocity limit to PID controller


ros::Subscriber ultrasonic_sub;            // Subscribe the ultrasonic data (downwards) from Guidance
ros::Subscriber vision_activation_sub;     // Subscribe an activation signal message from Vision nodes
ros::Subscriber dji_guidance_odometry_sub;
ros::Subscriber dji_RM_car_odometry_sub;
ros::Subscriber marker_position_sub;
ros::Subscriber marker_pose_sub;
ros::Subscriber marker_status_sub;
ros::Subscriber vision_position_sub;
ros::Subscriber vision_ready_sub;
ros::Subscriber global_target_position_sub;


dji_fusion sensor_data;
dji_fusion_rectangle rect;
dji_fusion_step step = INIT;
geometry_msgs::Vector3 target_pos_msg;
geometry_msgs::Vector3 current_pos_msg;
geometry_msgs::Vector3 odometry_msg;
bool is_marker_valid = false;
bool is_vision_valid = false;
vector<float>marker_map[7][7];
float size,interval,row,col,fov_h,fov_v;
bool vision_taking_control;
double ultrasonic_height;

bool check_point_in_retangle(dji_fusion_point point, dji_fusion_rectangle rect);
dji_fusion_rectangle compute_camera_vision_field(dji_fusion _odom);
dji_fusion_point get_marker_odometry_correction(dji_fusion_rectangle _vision_field);  //judgement after horizon mirror
dji_position position;
dji_fusion_point aim_bias = {0,0,0};
dji_fusion_point marker_correct_position;
dji_fusion_point correct_value;



void vision_ready_callback(const std_msgs::UInt8& msg)
{
    static ros::Time first_time;
    static bool timer_reset = true ;
    static int marker_valid = 0;

    if(timer_reset)
    {
        first_time = ros::Time::now();
        timer_reset = false;
    }
    if(ros::Time::now().toSec() - first_time.toSec() < 1.500000)
    {
        if(msg.data == 1)  marker_valid ++;
        if(marker_valid > 8)
        {
            is_vision_valid = true;
            marker_valid = 0;
            timer_reset = true;
        }
    }
    else
    {
        is_vision_valid = false;
        timer_reset = true;
    }

}

void guidance_ultrasonic_callback(const sensor_msgs::LaserScan& msg)
{
    
    if(msg.intensities[0] == 1) //Ultrasonic data is reliable
        
        ultrasonic_height = msg.ranges[0] / 1000; //Convert the unit from mm to m

}



void map_init(float _marker_size, float _marker_interval, float _marker_row, float _marker_col,float _fov_h, float _fov_v)
{
    size = _marker_size;
    interval = _marker_interval;
    row = _marker_row;
    col = _marker_col;
    fov_h = _fov_h;
    fov_v = _fov_v;
}



void marker_pose_callback(const geometry_msgs::PoseStampedConstPtr& msg)
{
    sensor_data.marker_position_x = -msg->pose.position.y;  // y->x
    sensor_data.marker_position_y = msg->pose.position.x;  // x->y  coor transform
    sensor_data.marker_position_z = msg->pose.position.z;
    
}

void marker_position_callback(const geometry_msgs::Vector3ConstPtr& msg)
{
    sensor_data.marker_position_x = msg->x;
    sensor_data.marker_position_y = -msg->y;
    sensor_data.marker_position_z = msg->z;
}

void vision_position_callback(const geometry_msgs::Vector3ConstPtr& msg)
{
    //vision get position at camera coor
    sensor_data.vision_position_x = msg->x ;
    sensor_data.vision_position_y = msg->y ;
    sensor_data.vision_position_z = msg->z ;
}


void global_target_position_callback(const geometry_msgs::Vector3ConstPtr& msg)
{
    position.globa_position_x = msg->x;
    position.globa_position_y = msg->y;
    position.global_position_z = msg->z;

}



void guidance_odometry_callback(const nav_msgs::Odometry& msg)
{
    sensor_data.guidance_position_x = msg.pose.pose.position.x;
    sensor_data.guidance_position_y = -msg.pose.pose.position.y;
    sensor_data.guidance_position_z = msg.pose.pose.position.z;
    //TODO test guidance.z or ultrasonic
    
}

void marker_status_callback(const std_msgs::UInt8& msg)
{
    static ros::Time first_time;
    static bool timer_reset = true ;
    static int marker_valid = 0;
    
    if(timer_reset)
    {
        first_time = ros::Time::now();
        timer_reset = false;
    }
    if(ros::Time::now().toSec() - first_time.toSec() < 1.000000)
    {
        if(msg.data == 1)  marker_valid ++;
        if(marker_valid > 10)
        {
            is_marker_valid = true;
            marker_valid = 0;
            timer_reset = true;
        }
    }
    else
    {
        is_marker_valid = false;
        timer_reset = true;
    }
    
}

dji_fusion_rectangle compute_camera_vision_field(dji_fusion _odom)
{
    //sensor_data guidance_odom;
    //guidance_odom = _odom;

    float c_x = _odom.guidance_position_x;
    float c_y = _odom.guidance_position_y;
    float c_z = _odom.guidance_position_z;
    cout << "fov_v" << fov_v <<endl;
    cout << "fov_h" << fov_h <<endl;
    float dx, dy;
    dx = tan( (fov_v*pi/180) /(float)2)*c_z;
    dy = tan( (fov_h*pi/180) /(float)2)*c_z;
    cout << "dx" << dx <<endl;
    cout << "dy" << dy <<endl;

    dji_fusion_rectangle camera_field;
    
    camera_field.point_1.x = c_x - dx;
    camera_field.point_1.y = c_y - dy;
    
    camera_field.point_2.x = c_x + dx;
    camera_field.point_2.y = c_y - dy;
    
    camera_field.point_3.x = c_x + dx;
    camera_field.point_3.y = c_y + dy;
    
    camera_field.point_4.x = c_x - dx;
    camera_field.point_4.y = c_y + dy;
    return camera_field;
}



dji_fusion_point get_marker_odometry_correction(dji_fusion_rectangle _vision_field)  //judgement after horizon mirror
{
    //dji_fusion_retangle _vision_field;
    dji_fusion_rectangle marker_rect;
    dji_fusion_point marker_corner;


    cout << "p1x " <<_vision_field.point_1.x<<endl;
    cout << "p1y " <<_vision_field.point_1.y<<endl;
    cout << "p2x " <<_vision_field.point_2.x<<endl;
    cout << "p2y " <<_vision_field.point_2.y<<endl;
    cout << "p3x " <<_vision_field.point_3.x<<endl;
    cout << "p3y " <<_vision_field.point_3.y<<endl;
    cout << "p4x " <<_vision_field.point_4.x<<endl;
    cout << "p4y " <<_vision_field.point_4.y<<endl;
    dji_fusion_point aruco;
    for(int i=0;i<col;i++)
    {
        for(int j=0;j<row;j++)
        {
            marker_corner.x = (float)j*(size+interval);
            marker_corner.y = (float)i*(size+interval);
            marker_rect.point_1 = marker_corner;
            cout << "corner 1 x " << marker_corner.x << " corner 1 y " <<marker_corner.y <<endl;
            marker_corner.x = (float)j*(size+interval)+size;
            marker_corner.y = (float)i*(size+interval);
            marker_rect.point_2 = marker_corner;
            cout << "corner 2 x " << marker_corner.x << " corner 2 y " <<marker_corner.y <<endl;
            marker_corner.x = (float)j*(size+interval)+size;
            marker_corner.y = (float)i*(size+interval)+size;
            marker_rect.point_3 = marker_corner;
            cout << "corner 3 x " << marker_corner.x << " corner 3 y " <<marker_corner.y <<endl;
            marker_corner.x = (float)j*(size+interval);
            marker_corner.y = (float)i*(size+interval)+size;
            marker_rect.point_4 = marker_corner;
            cout << "corner 4 x " << marker_corner.x << " corner 4 y " <<marker_corner.y <<endl;
            if(check_point_in_retangle(marker_rect.point_1,_vision_field)&&
                    check_point_in_retangle(marker_rect.point_2,_vision_field)&&
                    check_point_in_retangle(marker_rect.point_3,_vision_field)&&
                    check_point_in_retangle(marker_rect.point_4,_vision_field))
                
            {
                cout << "aruco id  x= "<<j<<"  y= "<<i<<endl ;
                aruco.x = (marker_rect.point_1.x +
                           marker_rect.point_2.x +
                           marker_rect.point_3.x +
                           marker_rect.point_4.x)/(float)4;
                aruco.y = (marker_rect.point_1.y +
                           marker_rect.point_2.y +
                           marker_rect.point_3.y +
                           marker_rect.point_4.y)/(float)4;
                cout<< "aruco correct odom is x ->" << aruco.x<< " y-> " <<  aruco.y<<endl;
                return aruco;
            }
        }
    }
    
}

dji_fusion_point odom_error_calc(bool marker_reliability, dji_fusion _guidance_odom)
{
    geometry_msgs::Vector3 guidance_correct_value;
    dji_fusion_point error;
    error.x = 0;
    error.y = 0;
    error.z = 0;
    if(marker_reliability)
    {
        marker_correct_position = get_marker_odometry_correction(compute_camera_vision_field(_guidance_odom));

        marker_correct_position.x = marker_correct_position.x - sensor_data.marker_position_x;//calc aruco position to UAV
        marker_correct_position.y = marker_correct_position.y - sensor_data.marker_position_y;
        cout<< "y-> " << marker_correct_position.y <<endl;
        //marker_correct_position is a predict position of UAV
        correct_value.x = _guidance_odom.guidance_position_x - marker_correct_position.x;
        correct_value.y = _guidance_odom.guidance_position_y - marker_correct_position.y;
        //correct_value is a error value from guidance odometry to predict position
        guidance_correct_value.x = _guidance_odom.guidance_position_x - marker_correct_position.x;
        guidance_correct_value.y = _guidance_odom.guidance_position_y - marker_correct_position.y;
        //correct_value is a error value from guidance odometry to predict position in ros msg form
        //so the true odom is observer - error
        guidance_correct_pub.publish(guidance_correct_value);
        error.x = correct_value.x ;
        error.y = correct_value.y ;
        return error;
    }
    else
        return error;
    
}


bool check_point_in_retangle(dji_fusion_point point, dji_fusion_rectangle rect)
{
    return ( ( (point.x > rect.point_1.x) && (point.x < rect.point_2.x) )&&
             ( (point.x > rect.point_4.x) && (point.x < rect.point_3.x) )&&
             ( (point.y > rect.point_1.y) && (point.y < rect.point_4.y) )&&
             ( (point.y > rect.point_2.y) && (point.y < rect.point_3.y) ) );
}

void odometry_calc()
{
    dji_fusion_point error,odometry;
    error = odom_error_calc(is_marker_valid, sensor_data);
    correct_value.x;
    correct_value.y;
    dji_fusion_point weight_of_guidance,weight_of_marker;
    if(is_marker_valid) {

        weight_of_guidance.x = 0.6;
        weight_of_guidance.y = 0.6;
        weight_of_guidance.z = 1.0;

        weight_of_marker.x =0.4;
        weight_of_marker.y =0.4;
        weight_of_marker.z =0.0;
    }

    else {

        weight_of_guidance.x = 1.0;
        weight_of_guidance.y = 1.0;
        weight_of_guidance.z = 1.0;

        weight_of_marker.x =0.0;
        weight_of_marker.y =0.0;
        weight_of_marker.z =0.0;

    }

    odometry.x = sensor_data.guidance_position_x * weight_of_guidance.x +
            marker_correct_position.x * weight_of_marker.x;

    odometry.y = -sensor_data.guidance_position_y * weight_of_guidance.y +
            marker_correct_position.y * weight_of_marker.y;

    odometry.z = sensor_data.guidance_position_z * weight_of_guidance.z +
            sensor_data.guidance_position_z * weight_of_marker.z;

    odometry_msg.x =  odometry.x;
    odometry_msg.y = -odometry.y;
    odometry_msg.z =  odometry.z;
    odometry_pub.publish(odometry_msg);
}


void fusion_step()
{
    static ros::Time timer;
    static bool is_first = true;
    static std_msgs::UInt8 vision_valid_msg;
    static std_msgs::UInt8 aim_msg;
    static int counter;
    switch(step)
    {

        case INIT:
    
            if(is_vision_valid==false) {
                
                step = LOGIC_CTRL;
            }
    
            break;

        case LOGIC_CTRL:
    
            target_pos_msg.x = position.globa_position_x ;
            target_pos_msg.y = position.globa_position_y ;
            target_pos_msg.z = position.global_position_z ;
            current_pos_msg.x = odometry_msg.x ;
            current_pos_msg.y = odometry_msg.y ;
            current_pos_msg.z = odometry_msg.z ;

            if(is_vision_valid) {

                vision_valid_msg.data = 1;
                vision_activation_pub.publish(vision_valid_msg );
                step = VISION_CTRL;
            }

            else {

                vision_valid_msg.data = 0;
                vision_activation_pub.publish(vision_valid_msg );
                step = LOGIC_CTRL;
            }
    
            break;

        case VISION_CTRL:
    
       
            target_pos_msg.x = aim_bias.x;
            target_pos_msg.y = aim_bias.y;
            target_pos_msg.z = position.global_position_z ;
            //        target_pos_msg.z = aim_bias.z;
            //target_pos_pub.publish(target_pos_msg);

            //reverse controll value to adapt vision controll
            //mirror about aim_bias point
            current_pos_msg.x = (float)2*aim_bias.x - sensor_data.vision_position_x;
            current_pos_msg.y = (float)2*aim_bias.y - sensor_data.vision_position_y;
            //        current_pos_msg.z = aim_bias.z - sensor_data.vision_position_z;
            current_pos_msg.z = odometry_msg.z ;
            //current_pos_pub.publish(current_pos_msg);

            if( ((target_pos_msg.x - current_pos_msg.x) <  0.3 )&&
                ((target_pos_msg.x - current_pos_msg.x) > -0.3 )&&
                ((target_pos_msg.y - current_pos_msg.y) <  0.3 )&&
                ((target_pos_msg.y - current_pos_msg.y) > -0.3 )
              ) {

                counter++;
                if(is_first) {

                    timer = ros::Time::now();
                    is_first = false;
                }

            if(ros::Time::toSec()-timer.toSec() < 1) {

                if(counter > 30) {

                    counter = 0 ;
                    aim_msg.data = 1;
                    release_pub.publish(aim_msg);
                    ROS_INFO("Aiming Completed & Release signal sent");
                }
            }
        

    
            break;






    }

}




int main(int argc, char** argv)
{
    ros::init(argc, argv, "fusion");
    ros::NodeHandle n;
    ros::Rate loop_rate(50);
    
    string map_time;
    float camera_fov_H = 46;
    float camera_fov_V = 36;
    float marker_size = 0.390;
    float marker_interval = 0.590;
    float marker_col_y = 7;
    float marker_row_x = 7;
    string map_file = "/home/dji/ws/src/dji_fusion/map/marker.map" ;
    FileStorage fs1(map_file,FileStorage::READ) ;
    
    fs1["map_time"] >>  map_time;
    fs1["camera_FOV_H"] >>  camera_fov_H;
    fs1["camera_FOV_V"] >>  camera_fov_V;
    fs1["marker_size"] >>  marker_size;
    fs1["marker_interval"] >>  marker_interval;
    fs1["marker_col_y"] >>  marker_col_y;
    fs1["marker_row_x"] >>  marker_row_x;
    
    
    map_init(marker_size, marker_interval, marker_row_x, marker_col_y, camera_fov_H, camera_fov_V);

    cout <<"map update at " << map_time<<endl;

    pid_param_pub               = n.advertise<geometry_msgs::Vector3>("/pid_parameter",  10);
    pid_ctrl_limit_pub          = n.advertise<geometry_msgs::Vector3>("/pid_ctrl_limit", 10);
    
    target_pos_pub              = n.advertise<geometry_msgs::Vector3>("/target_position",  50);
    current_pos_pub             = n.advertise<geometry_msgs::Vector3>("/current_position", 50);
    
    odometry_pub                = n.advertise<geometry_msgs::Vector3>("/odometry",50);
    guidance_correct_pub        = n.advertise<geometry_msgs::Vector3>("/guidance_correct",50);
    vision_activation_pub       = n.advertise<std_msgs::UInt8>("/vision_activation",50);
    release_pub                 = n.advertise<std_msgs::UInt8>("/release",50);


    marker_status_sub           = n.subscribe("/marker_status",       1, marker_status_callback);
    marker_position_sub         = n.subscribe("/marker_position",     1, marker_position_callback);
    marker_pose_sub             = n.subscribe("/marker_pose",         1, marker_pose_callback);
    vision_position_sub         = n.subscribe("/vision_position",     1, vision_position_callback);
    ultrasonic_sub              = n.subscribe("/guidance/ultrasonic", 1, guidance_ultrasonic_callback);
    dji_guidance_odometry_sub   = n.subscribe("/guidance/odometry",   1, guidance_odometry_callback);
    vision_ready_sub            = n.subscribe("/vision_ready",        1, vision_ready_callback);
    global_target_position_sub  = n.subscribe("/global_target_position",     1, global_target_position_callback);

    while(ros::ok())
    {
        //get_marker_odometry_correction(compute_camera_vision_field(sensor_data));
        odometry_calc();
        target_pos_pub.publish(target_pos_msg);
        current_pos_pub.publish(current_pos_msg);
        loop_rate.sleep();
        ros::spinOnce();
    }
    
    return 0;
}