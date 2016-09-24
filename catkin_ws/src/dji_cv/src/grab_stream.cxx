/**
******************************************************************************
* @file    grab_stream.cpp
* @author  FrankChen
* @version V1.0.1
* @date    28-May-2016
* @brief   This file provides basic funtion that grab video stream from video device/file or ros message.
*          and provide preview window that from input stream and output stream.
*          This programme also support ros launch file parameter set.
* @TODO    add Gige Vision grab feature
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

void usage()
{
   printf( "This is a video stream grab programm. \n"
           "Usage: Grab Stream\n"
           "       -s  <source device/file>        #Video Device Index or video file path \"default <-1 or path/video.avi>\"\n"
           "       -i  <camera instrinsic file>    #The Camera Intrinsic File  \"default  <path/calibration.yml>\"\n"
           "       -ri <ROS topic in>              #ROS topic input  \"default <camera/in>\"\n"
           "       -ro <ROS topic out>             #ROS topic output \"default <camera/out>\"\n"
           "       -rp <ROS Launch para>           #Get parameter from launch file \"default <false>\"\n"
           "       Expert option. [Warn]: set these parameter may cause image quality bad!\n"
           "       -w  <width>                     #video width  \"default <640>\"\n"
           "       -h  <height>                    #video height \"default <480>\"\n"
           "       [-e <exposure>]  \n"
           "       [-b <brightness>]  \n"
           "       [-c <contrast>]  \n"
           "       [-s <saturation>] \n"
           "       [-h <hue>] \n"
           "       [-g <gain>] \n"

           );

}


void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    Mat image_in;
    cv_bridge::CvImage image_msg;

    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BAYER_GRBG8);
    image_in = cv_ptr -> image;
    image_msg.image = image_in;
    image_msg.header.stamp = ros::Time::now();
    image_msg.encoding = "bgr8";

    image_pub.publish(image_msg.toImageMsg());


}





int main(int argc,char **argv)
{
    ros::init(argc, argv, "grab_stream");
    ros::NodeHandle n,nh;
    image_transport::ImageTransport it(nh);

    int CAMERA_INDEX = -1;
    const char*  inputfile = "video.avi";
    const char*  TheIntrinsicFile = "calibration.yml";
    string ros_in_topic = "/camera/in",ros_out_topic = "/camera/out";
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

if(argc <= 1)
{
    usage();
    return 0;
}
/**************************************************************************
 * get parameter from command line.
 *
 * ***********************************************************************/
    for(int i = 1; i < argc; i++)
    {
        const char* s = argv[i];


        if(strcmp( s, "-s" ) == 0)
        {
             char* ss = argv[++i];
             if( isdigit(ss[0]) )
             {
                sscanf( argv[i], "%d", &CAMERA_INDEX );
                is_device = true;
             }
             else
                 inputfile = argv[i];
        }
        else if(strcmp( s, "-i" ) == 0)
        {
            TheIntrinsicFile = argv[++i];
            enable_undistort = true;
        }
        else if(strcmp( s, "-ri" ) == 0)
        {
            ros_in_topic = argv[++i];
            enable_topic_transport = true;
        }
        else if(strcmp( s, "-ro" ) == 0)
        {
            ros_out_topic = argv[++i];
        }
        else if(strcmp( s, "-rp" ) == 0)
        {
            if(strcmp( argv[++i], "true" ))
                read_ros = true;
        }
        else if(strcmp( s, "-w" ) == 0)
        {
            sscanf( argv[++i], "%d", &width );
        }
        else if(strcmp( s, "-h" ) == 0)
        {
            sscanf( argv[++i], "%d", &height );
        }
        else if(strcmp( s, "-e" ) == 0)
        {
            sscanf( argv[++i], "%lf", &exposure );
        }
        else if(strcmp( s, "-b" ) == 0)
        {
            sscanf( argv[++i], "%lf", &brightness );
        }
        else if(strcmp( s, "-c" ) == 0)
        {
            sscanf( argv[++i], "%lf", &contrast );
        }
        else if(strcmp( s, "-s" ) == 0)
        {
            sscanf( argv[++i], "%lf", &saturation );
        }
        else if(strcmp( s, "-h" ) == 0)
        {
            sscanf( argv[++i], "%lf", &hue );
        }
        else if(strcmp( s, "-g" ) == 0)
        {
            sscanf( argv[++i], "%lf", &gain );
        }
        else if(strcmp( s, "-help" ) == 0)
        {
            usage();
            return 0;
        }
        else
        {
            printf("Unknow parameter.\n");
            usage();
        }
    }
    if(read_ros)
    {
        nh.param("width",      width,      int(640));
        nh.param("height",     height,     int(480));
        nh.param("exposure",   exposure,   double(0.2));//0.2
        nh.param("brightness", brightness, double(0.5));//0.2
        nh.param("contrast",   contrast,   double(0.6));//0.6
        nh.param("saturation", saturation, double(0.5));//0.2
        nh.param("hue",        hue,        double(0.5));//0.15
        nh.param("gain",       gain,       double(0.5));//0.2
    }

    cv_bridge::CvImagePtr imagePtr;
    cv_bridge::CvImage image_out_msg;


    image_pub = it.advertise(ros_out_topic, 10);
    image_sub = it.subscribe(ros_in_topic , 10, imageCallback);




    VideoCapture capture;
    if( is_device )
    {
        capture.open(CAMERA_INDEX);
        cout<<"open camera device "<<CAMERA_INDEX<<endl;

    }
    else
    {
        capture.open(inputfile);
        cout<<"open file "<<inputfile<<endl;

    }

    if( !capture.isOpened())
        return fprintf( stderr, "Could not initialize video capture\n"), -2;

//    cap.set(CV_CAP_PROP_FOURCC ,CV_FOURCC('M', 'J', 'P', 'G') );
//    cap.set(CV_CAP_PROP_FRAME_WIDTH ,width);
//    cap.set(CV_CAP_PROP_FRAME_HEIGHT ,height);
//    cap.set(CV_CAP_PROP_EXPOSURE ,exposure);
//    cap.set(CV_CAP_PROP_BRIGHTNESS ,brightness);
//    cap.set(CV_CAP_PROP_CONTRAST,contrast);
//    cap.set(CV_CAP_PROP_SATURATION,saturation);
//    cap.set(CV_CAP_PROP_HUE,hue);
//    cap.set(CV_CAP_PROP_GAIN,gain);
//    cap.set(CV_CAP_PROP_FPS, 25);



//    ros::Subscribler image_sub  = n.advertise<geometry_msgs::Vector3>("board_pos",20);
//    ros::Publisher camera_pub = n.advertise<geometry_msgs::PoseStamped>("board_pose",20);
//    ros::Publisher aux_pub    = n.advertise<geometry_msgs::Vector3>("board_rot",20);

//********************read intrinsic file******************************
    Mat CM = Mat(3, 3, CV_32FC1);
    Mat D;


    //************************undistort*********************************
    Mat map1,map2;

    Mat image;

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

    ros::Rate loopRate(30);

    while(nh.ok() && waitKey(30) != 'c' )
    {

       if(!enable_topic_transport)
       {
           capture >> image;
           //imshow("image_show",image);
           image_out_msg.image = image;
           image_out_msg.header.stamp = ros::Time::now();
           image_out_msg.encoding = "bgr8";

           image_pub.publish(image_out_msg.toImageMsg());

       }

       ros::spinOnce();
       loopRate.sleep();

    }


}
























