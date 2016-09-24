/**
******************************************************************************
* @file    read_camera.cpp
* @author  FrankChen
* @version V2.4.0
* @date    30-May-2016
* @brief   This file provides basic funtion that grab video stream from ros topic.
******************************************************************************
*/
//include ros library
#include "ros/ros.h"

//include std/string libraries
#include<sstream>
#include<iostream>
#include<stdio.h>
#include<stdlib.h>

//include messege libraries
#include "sensor_msgs/image_encodings.h"
#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"


//include opencv libraries
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/calib3d/calib3d.hpp"

using namespace cv;
using namespace std;



image_transport::Publisher  image_pub;
image_transport::Subscriber image_sub;

Mat CM = Mat(3, 3, CV_32FC1);
Mat D;
Mat map1,map2;
Mat image;
bool is_first=true;

int CAMERA_INDEX = 0;
const char*  TheIntrinsicFile = "calibration.yml";
bool   read_ros = false;
bool   is_device = false;
bool   enable_topic_transport = false;
bool   enable_undistort = false ;
int    width;
int    height;
double exposure;
double brightness;
double contrast;
double saturation;
double hue;
double gain;

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    Mat image_in;
    Mat undistort_view;
    cv_bridge::CvImage image_msg;


    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
    image_in = cv_ptr -> image;
    resize(cv_ptr -> image,image_in,Size(640,480),0,0,CV_INTER_LINEAR);
    image=image_in.clone();
    if(is_first)
    {
        initUndistortRectifyMap(CM,D,Mat(),Mat(),image_in.size(),CV_32FC1,map1,map2);
        is_first = false;
    }

    if(enable_undistort)
    {
        remap(image_in, undistort_view, map1, map2, INTER_LINEAR);
        image_in = undistort_view.clone();
    }

    image_msg.image = image_in;
    image_msg.header.stamp = ros::Time::now();
    image_msg.encoding = "mono8";

    imshow("IR_MONO",image_in);
    image_pub.publish(image_msg.toImageMsg());


}




int main(int argc,char **argv)
{
    ros::init(argc, argv, "read_camera");
    ros::NodeHandle n,nh;
    image_transport::ImageTransport it(nh);

//  cv_bridge::CvImagePtr imagePtr;
    cv_bridge::CvImage image_out_msg;

    image_pub = it.advertise("/camera/ir", 10);
    image_sub = it.subscribe("/camera/image_raw" , 10, imageCallback);


    if(argc>1)
    {
        CAMERA_INDEX = atof(argv[1]);
        if(argc>2)
        {
            TheIntrinsicFile = argv[2];
            if(argc>3)
            {
                if(strcmp( argv[3], "true" ) == 0)
                {
                  enable_undistort = true;
                  //cout << "true"<<endl;
                }
                if(argc>4)
                {
                    if(strcmp( argv[4], "true" ) == 0)
                        enable_topic_transport = true;
                }
                else
                {
                    cout<<"usage: --\"camera index\" --\"The IntrinsicFile\" --\"enable_undistort(true/false) \" --\"enable IR receive from ROS(true/false) \""<<endl;
                    return 0;

                }
            }
            else
            {
                cout<<"usage: --\"camera index\" --\"The IntrinsicFile\" --\"enable_undistort(true/false) \" --\"enable IR receive from ROS(true/false) \""<<endl;
                return 0;
            }

        }
        else
        {
            cout<<"usage: --\"camera index\" --\"The IntrinsicFile\" --\"enable_undistort(true/false) \" --\"enable IR receive from ROS(true/false) \""<<endl;
            return 0;
        }
    }
    else
    {
        cout<<"usage: --\"camera index\" --\"The IntrinsicFile\" --\"enable_undistort(true/false) \" --\"enable IR receive from ROS(true/false) \""<<endl;
        return 0;
    }

   ros::Rate loopRate(30);
   VideoCapture capture;

   if(!enable_topic_transport)
    {
        capture.open(CAMERA_INDEX);
        cout<<"open camera device "<<CAMERA_INDEX<<endl;

        if( !capture.isOpened())
            return fprintf( stderr, "Could not initialize video capture\n"), -2;
        capture >> image;

        if(enable_undistort)
        {
            FileStorage fs2(TheIntrinsicFile,FileStorage::READ);
            fs2["camera_matrix"]>>CM;
            fs2["distortion_coefficients"]>>D;
            fs2.release();
            cout<<"CM: "<<CM<<endl<<"D: "<<D<<endl;
            initUndistortRectifyMap(CM,D,Mat(),Mat(),image.size(),CV_32FC1,map1,map2);
        }
    }

    while(nh.ok() && waitKey(30) != 'q' )
    {

        if(!enable_topic_transport)
        {
           capture >> image;
           if(enable_undistort)
           {
               Mat undistort_view;
               imshow("image_distort",image);
               remap(image, undistort_view, map1, map2, INTER_LINEAR);
               image = undistort_view.clone();
           }
           imshow("image_read",image);
           image_out_msg.image = image;
           image_out_msg.header.stamp = ros::Time::now();
           image_out_msg.encoding = "mono8";

           image_pub.publish(image_out_msg.toImageMsg());

        }
        else
        {
        }
       ros::spinOnce();
       loopRate.sleep();

    }


}
























