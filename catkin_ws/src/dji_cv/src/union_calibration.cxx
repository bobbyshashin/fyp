/**
******************************************************************************
* @file    union_calibration.cpp
* @author  FrankChen
* @version V1.1.0
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
bit_position point1,point2,point3,point4;
Mat rosImage;
ros::Subscriber pose_1_sub;
ros::Subscriber pose_2_sub;
ros::Subscriber pose_3_sub;
ros::Subscriber pose_4_sub;

ros::Subscriber img_point_sub_1;
ros::Subscriber img_point_sub_2;
ros::Subscriber img_point_sub_3;
ros::Subscriber img_point_sub_4;

image_transport::Subscriber image_sub;
image_transport::Publisher image_pub;
MatrixXf CM;
MatrixXf DM;

int transportMat(Mat src)
{
    rosImage = src.clone();
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    Mat image_in;
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
    image_in = cv_ptr -> image;
    transportMat(image_in);
}

void board_pose_1_callback(const geometry_msgs::PoseStampedConstPtr &msg)
{
    pose1.pos.x_w= msg->pose.position.x;
    pose1.pos.y_w= msg->pose.position.y;
    pose1.pos.z_w= msg->pose.position.z;

    pose1.rvec.r1= msg->pose.position.x;
    pose1.rvec.r2= msg->pose.position.y;
    pose1.rvec.r3= msg->pose.position.z;
    //    cout<<"Pose_1 pos-> "<<pose1.pos<<endl;
    //    cout<<"Pose_1 rvec-> "<<pose1.rvec<<endl;


}
void board_pose_2_callback(const geometry_msgs::PoseStampedConstPtr &msg)
{

    pose2.pos.x_w= msg->pose.position.x;
    pose2.pos.y_w= msg->pose.position.y;
    pose2.pos.z_w= msg->pose.position.z;

    pose2.rvec.r1= msg->pose.position.x;
    pose2.rvec.r2= msg->pose.position.y;
    pose2.rvec.r3= msg->pose.position.z;
    //    cout<<"Pose_2 pos-> "<<pose2.pos<<endl;
    //    cout<<"Pose_2 rvec-> "<<pose2.rvec<<endl;

}
void board_pose_3_callback(const geometry_msgs::PoseStampedConstPtr &msg)
{

    pose3.pos.x_w= msg->pose.position.x;
    pose3.pos.y_w= msg->pose.position.y;
    pose3.pos.z_w= msg->pose.position.z;

    pose3.rvec.r1= msg->pose.position.x;
    pose3.rvec.r2= msg->pose.position.y;
    pose3.rvec.r3= msg->pose.position.z;
    //    cout<<"Pose_3 pos-> "<<pose3.pos<<endl;
    //    cout<<"Pose_3 rvec-> "<<pose3.rvec<<endl;

}
void board_pose_4_callback(const geometry_msgs::PoseStampedConstPtr &msg)
{

    pose4.pos.x_w= msg->pose.position.x;
    pose4.pos.y_w= msg->pose.position.y;
    pose4.pos.z_w= msg->pose.position.z;

    pose4.rvec.r1= msg->pose.position.x;
    pose4.rvec.r2= msg->pose.position.y;
    pose4.rvec.r3= msg->pose.position.z;
    //    cout<<"Pose_4 pos-> "<<pose4.pos<<endl;
    //    cout<<"Pose_4 rvec-> "<<pose4.rvec<<endl;

}

void image_point_1_callback(const geometry_msgs::PointConstPtr &msg)
{
    point1.u = msg->x;
    point1.v = msg->y;
    point1.reliable = msg->z;
    cout<<"Point_1 u-> "<<point1.u<<endl;
    cout<<"Point_1 v-> "<<point1.v<<endl;
    cout<<"Point_1 -> reliable "<<point1.reliable<<endl;
}


void image_point_2_callback(const geometry_msgs::PointConstPtr &msg)
{
    point2.u = msg->x;
    point2.v = msg->y;
    point2.reliable = msg->z;
    cout<<"Point_2 u-> "<<point2.u<<endl;
    cout<<"Point_2 v-> "<<point2.v<<endl;
    cout<<"Point_2 -> reliable "<<point2.reliable<<endl;

}

void image_point_3_callback(const geometry_msgs::PointConstPtr &msg)
{
    point3.u = msg->x;
    point3.v = msg->y;
    point3.reliable = msg->z;
    cout<<"Point_3 u-> "<<point3.u<<endl;
    cout<<"Point_3 v-> "<<point3.v<<endl;
    cout<<"Point_3 -> reliable "<<point3.reliable<<endl;

}

void image_point_4_callback(const geometry_msgs::PointConstPtr &msg)
{
    point4.u = msg->x;
    point4.v = msg->y;
    point4.reliable = msg->z;
    cout<<"Point_4 u-> "<<point4.u<<endl;
    cout<<"Point_4 v-> "<<point4.v<<endl;
    cout<<"Point_4 -> reliable "<<point4.reliable<<endl;

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

int feature_extract()
{
    cv_bridge::CvImage image_msg;
    Mat roi_image;
    if((point1.reliable>=0.5)&&(point2.reliable>=0.5)&&(point3.reliable>=0.5)&&(point4.reliable>=0.5))
    {
        Rect roi_rect=Rect( (int)point1.u, (int)point1.v, (int)point4.u-(int)point1.u, (int)point4.v-(int)point1.v );
        //Rect roi_rect=Rect( (int)point1.u, (int)point1.v, (int)point4.u-(int)point1.u, (int)point4.v-(int)point1.v );
        roi_image=rosImage(roi_rect);
        //imshow("ROI",roi_image);

        image_msg.image = roi_image;
        image_msg.header.frame_id = "ROI_of_Union_calibration_Board";
        image_msg.header.stamp = ros::Time::now();
        image_msg.encoding = "mono8";
        image_pub.publish(image_msg.toImageMsg());

    }
    else
        ;
        //imshow("ROI",rosImage);
}


int main(int argc,char **argv)
{
    ros::init(argc, argv, "union_calibration");
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
    image_pub = it.advertise("aruco_board_roi",10);

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
    while(rosImage.empty())
        ros::spinOnce();

    while(n.ok()&&('q' != waitKey(30)) )
    {
        eigen();
        feature_extract();
        //imshow("Image",rosImage);
        ros::spinOnce();
    }



    return 0;
}

