/**
******************************************************************************
* @file    obtain_3D_pose.cpp
* @author  FrankChen
* @version V0.3.0
* @date    4-May-2016
* @brief   This file provides basic funtion that solve Mathematical problems which
*          use in calibration and reconstruct 3D information.
******************************************************************************
*/

//include ros library
#include "ros/ros.h"

//include std/string libraries
#include<sstream>
#include<iostream>
#include<stdio.h>
#include<stdlib.h>
#include<vector>
//include messege libraries
#include "sensor_msgs/image_encodings.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/PointCloud.h"
#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Vector3.h"
#include <tf/transform_broadcaster.h>
#include "std_msgs/String.h"

//include eigen library
#include "eigen3/Eigen/Dense"

//include user library
#include "user.h"

//include opencv libraries
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace Eigen;
using namespace std;
using namespace cv;

vector <bit_pose> pose;
bit_pose pose1,pose2,pose3,pose4;

ros::Subscriber pose_1_sub;
ros::Subscriber pose_2_sub;
ros::Subscriber pose_3_sub;
ros::Subscriber pose_4_sub;

ros::Subscriber img_point_sub_1;
ros::Subscriber img_point_sub_2;
ros::Subscriber img_point_sub_3;
ros::Subscriber img_point_sub_4;

image_transport::Subscriber image_sub;

MatrixXf CM;
MatrixXf DM;

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    Mat image_in;
    cv_bridge::CvImage image_msg;

    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    image_in = cv_ptr -> image;


}

void board_pose_1_callback(const geometry_msgs::PoseStampedConstPtr &msg)
{
    pose1.pos.x_w= msg->pose.position.x;
    pose1.pos.y_w= msg->pose.position.y;
    pose1.pos.z_w= msg->pose.position.z;

    pose1.rvec.r1= msg->pose.position.x;
    pose1.rvec.r2= msg->pose.position.y;
    pose1.rvec.r3= msg->pose.position.z;

}
void board_pose_2_callback(const geometry_msgs::PoseStampedConstPtr &msg)
{

    pose2.pos.x_w= msg->pose.position.x;
    pose2.pos.y_w= msg->pose.position.y;
    pose2.pos.z_w= msg->pose.position.z;

    pose2.rvec.r1= msg->pose.position.x;
    pose2.rvec.r2= msg->pose.position.y;
    pose2.rvec.r3= msg->pose.position.z;


}
void board_pose_3_callback(const geometry_msgs::PoseStampedConstPtr &msg)
{

    pose3.pos.x_w= msg->pose.position.x;
    pose3.pos.y_w= msg->pose.position.y;
    pose3.pos.z_w= msg->pose.position.z;

    pose3.rvec.r1= msg->pose.position.x;
    pose3.rvec.r2= msg->pose.position.y;
    pose3.rvec.r3= msg->pose.position.z;


}
void board_pose_4_callback(const geometry_msgs::PoseStampedConstPtr &msg)
{

    pose4.pos.x_w= msg->pose.position.x;
    pose4.pos.y_w= msg->pose.position.y;
    pose4.pos.z_w= msg->pose.position.z;

    pose4.rvec.r1= msg->pose.position.x;
    pose4.rvec.r2= msg->pose.position.y;
    pose4.rvec.r3= msg->pose.position.z;

}

void image_point_1_callback(const geometry_msgs::PointConstPtr &msg)
{
    pose1.pos.u = msg->x;
    pose1.pos.v = msg->y;

}


void image_point_2_callback(const geometry_msgs::PointConstPtr &msg)
{
    pose2.pos.u = msg->x;
    pose2.pos.v = msg->y;

}

void image_point_3_callback(const geometry_msgs::PointConstPtr &msg)
{
    pose3.pos.u = msg->x;
    pose3.pos.v = msg->y;

}

void image_point_4_callback(const geometry_msgs::PointConstPtr &msg)
{
    pose4.pos.u = msg->x;
    pose4.pos.v = msg->y;

}
int eigen()
{
    float r_pos_x,r_pos_y,r_pos_z;
    float diff_21_x,diff_43_x,diff_21_y,diff_43_y,diff_21_z,diff_43_z;
    diff_21_x = pose2.pos.x_w-pose1.pos.x_w;
    diff_21_y = pose2.pos.y_w-pose1.pos.y_w;
    diff_21_z = pose2.pos.z_w-pose1.pos.z_w;

    diff_43_x = pose4.pos.x_w-pose3.pos.x_w;
    diff_43_y = pose4.pos.y_w-pose3.pos.y_w;
    diff_43_z = pose4.pos.z_w-pose3.pos.z_w;


}



int main(int argc,char **argv)
{
    ros::init(argc, argv, "obtain_3D_pose");
    ros::NodeHandle n,nh;
    image_transport::ImageTransport it(nh);

    pose_1_sub = n.subscribe("board_pose_1" , 50, board_pose_1_callback);
    pose_2_sub = n.subscribe("board_pose_2" , 50, board_pose_2_callback);
    pose_3_sub = n.subscribe("board_pose_3" , 50, board_pose_3_callback);
    pose_4_sub = n.subscribe("board_pose_4" , 50, board_pose_4_callback);
    img_point_sub_1 = n.subscribe("image_point_1" , 50, image_point_1_callback);
    img_point_sub_2 = n.subscribe("image_point_2" , 50, image_point_2_callback);
    img_point_sub_3 = n.subscribe("image_point_3" , 50, image_point_3_callback);
    img_point_sub_4 = n.subscribe("image_point_4" , 50, image_point_4_callback);

    image_sub = it.subscribe("camera/ir" , 10, imageCallback);


//    if(argc>1)
//    {
//        TheBoardConfigFile = argv[1];
//        if(argc>2)
//        {
//            TheIntrinsicFile = argv[2];
//            if(argc>3)
//            TheMarkerSize = atof(argv[3]);
//        }
//    }
//    else
//    {
//        cout<<"usage: --\"board config\" --\"Marker size\" "<<endl;
//        return 0;
//    }


    while(n.ok())
    {
        eigen();
        ros::spinOnce();
    }



    return 0;
}

