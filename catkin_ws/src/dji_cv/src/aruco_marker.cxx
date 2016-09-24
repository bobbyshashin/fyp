/**
******************************************************************************
* @file    aruco_marker.cxx
* @author  FrankChen
* @version V0.1.0
* @date    27-Jul-2016
* @brief   This file provides basic funtion that detect aruco marker 4 DJI RM SummerCamp and publish pose and rotation data to ros topic.
******************************************************************************
*/

//include ros library
#include "ros/ros.h"

//include std/string libraries
#include<sstream>
#include<iostream>
#include<stdio.h>
#include<stdlib.h>
#include<string>
//include messege libraries
#include "sensor_msgs/image_encodings.h"
#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Vector3.h"
#include "tf/transform_broadcaster.h"
//#include "Matrix3x3.h"
#include "std_msgs/String.h"
#include "std_msgs/UInt8.h"
#include "std_msgs/Char.h"

//include opencv libraries
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

//include aruco libraries
#include "aruco/aruco.h"
#include "aruco/highlyreliablemarkers.h"
#include "aruco/cvdrawingutils.h"

//include eigen library
#include "eigen3/Eigen/Dense"

using namespace Eigen;
using namespace cv;
using namespace std;
using namespace aruco;

ros::Publisher position_pub;
ros::Publisher pose_pub;
ros::Publisher rotation_pub;
ros::Publisher marker_status_pub;
image_transport::Subscriber image_sub;
long frame_count=0;

Mat image;
Mat gray_img,blur_img,corner_img,nor_img,scal_img;

Mat rvec,tvec;

string TheIntrinsicFile;
string TheDictionaryFile;
int ThePyrDownLevel;
vector< Marker > Markers;
MarkerDetector MDetector;
float TheMarkerSize = 0.15;

pair< double, double > AvrgTime(0, 0); // determines the average time required for detection
double ThresParam1, ThresParam2;
int iThresParam1, iThresParam2;
int thresholdValue;
int waitTime = 0;
bool lockedCorners = false;
void cvTackBarEvents(int pos, void *);
std_msgs::String str_msg;
std_msgs::UInt8 marker_status_msg;
geometry_msgs::Vector3 pos_msg;
geometry_msgs::PoseStamped pose_msg;
geometry_msgs::Vector3 rot_msg;

CameraParameters TheCameraParameters;

Mat Rvec[4],Tvec[4];
Mat RM[4],TM[4];
string camera_file,marker_file,image_topic;
void marker_position_iosd(Mat &Image, Marker &m, const CameraParameters &CP, float x, float y, float z)
{

    float size = m.ssize * 3;
    Mat objectPoints(4, 3, CV_32FC1);
    objectPoints.at< float >(0, 0) = 0;
    objectPoints.at< float >(0, 1) = 0;
    objectPoints.at< float >(0, 2) = 0;
    objectPoints.at< float >(1, 0) = size;
    objectPoints.at< float >(1, 1) = 0;
    objectPoints.at< float >(1, 2) = 0;
    objectPoints.at< float >(2, 0) = 0;
    objectPoints.at< float >(2, 1) = size;
    objectPoints.at< float >(2, 2) = 0;
    objectPoints.at< float >(3, 0) = 0;
    objectPoints.at< float >(3, 1) = 0;
    objectPoints.at< float >(3, 2) = size;

    vector< Point2f > imagePoints;

    projectPoints(objectPoints, m.Rvec, m.Tvec, CP.CameraMatrix, CP.Distorsion, imagePoints);
    char buf[50];
    sprintf (buf, "%f", x);
    string str_x(buf);
    sprintf (buf, "%f", y);
    string str_y(buf);
//    stringstream sx,sy;
//    sx << x;
//    sy << y;
//    string str_x = sx.str();
//    string str_y = sy.str();
    string sx = "position x-> "+str_x;
    putText(Image, sx, Point2f(imagePoints[0].x, imagePoints[0].y), FONT_HERSHEY_SIMPLEX, 0.6,  Scalar(0, 255, 255, 255), 2);
    string sy = "position y-> "+str_y;
    putText(Image, sy, Point2f(imagePoints[0].x, imagePoints[0].y+20), FONT_HERSHEY_SIMPLEX, 0.6,  Scalar(0, 255, 255, 255), 2);


    // draw lines of different colours
    //    line(Image, imagePoints[0], imagePoints[1], Scalar(0, 0, 255, 255), 1, CV_AA);
    //    line(Image, imagePoints[0], imagePoints[2], Scalar(0, 255, 0, 255), 1, CV_AA);
    //    line(Image, imagePoints[0], imagePoints[3], Scalar(255, 0, 0, 255), 1, CV_AA);
    //    putText(Image, "x", imagePoints[1], FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0, 0, 255, 255), 2);
    //    putText(Image, "y", imagePoints[2], FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0, 255, 0, 255), 2);
    //    putText(Image, "z", imagePoints[3], FONT_HERSHEY_SIMPLEX, 0.6, Scalar(255, 0, 0, 255), 2);
}


int saveMat(Mat src)
{
    image = src.clone();
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    //Mat cv_bridge_data ;
    // Mat cv_bridge_data(640, 480, CV_8UC3);

    // memcpy(&cv_bridge_data.data,msg->data,sizeof(msg->data));
    //cv_bridge_data.data = msg->data ;
    float tick = getTickCount();

    Mat image_org,image_callback,image_in,erodeImage,dilateImage,thresholdImage;
    vector<Mat> rgb_image;
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
    image_org = cv_ptr -> image.clone();
    resize(image_org,image_callback,Size(640,480),0,0,INTER_LINEAR);
    image_in = image_callback.clone();
    gray_img = image_in.clone();
    saveMat(image_in);
    split(image_in,rgb_image);

    Mat Erode_element = getStructuringElement(MORPH_RECT, Size(11, 11));
    erode(image_callback, erodeImage, Erode_element);
    //imshow("erode",erodeImage);

    Mat dilate_element = getStructuringElement(MORPH_RECT, Size(9, 9));
    dilate(erodeImage, dilateImage, dilate_element);
    //imshow("dilate",dilateImage);

    threshold(dilateImage,thresholdImage,thresholdValue,255,CV_THRESH_BINARY);
    //imshow("threshold",thresholdImage);
    cvtColor(image_in,gray_img,CV_RGB2GRAY);

    //gray_img=image_callback.clone();
    //threshold(rgb_image.at(2),thresholdImage,thresholdValue,255,CV_THRESH_BINARY);
    //imshow("G",rgb_image.at(2));
    // MDetector.setThresholdParams(iThresParam1, iThresParam2);
    cout<<"p1 "<<iThresParam1<<" p2 "<<iThresParam2<<endl;
    //MDetector.detect(rgb_image.at(2), Markers, TheCameraParameters, TheMarkerSize);
    MDetector.detect(image_in, Markers, TheCameraParameters, TheMarkerSize);


    for (unsigned int i = 0; i < Markers.size(); i++) {
        cout << Markers[i] << endl;
        Markers[i].draw(image_in, Scalar(0, 0, 255), 1);
    }

    for(int i=0;i<Markers.size();i++)
    {
        RM[i]=Markers[i].Rvec;
        TM[i]=Markers[i].Tvec;
    }

    if (TheCameraParameters.isValid())
        for (unsigned int i = 0; i < Markers.size(); i++) {
            CvDrawingUtils::draw3dCube(image_in, Markers[i], TheCameraParameters);
            CvDrawingUtils::draw3dAxis(image_in, Markers[i], TheCameraParameters);
            marker_position_iosd(image_in, Markers[0], TheCameraParameters,
                    TM[i].at<float>(0,0), TM[i].at<float>(1,0), TM[i].at<float>(2,0));

        }





    for(int i=0;i<Markers.size();i++)
    {
        //      pos_msg.x = TM[i].at<float>(0,0);
        //      pos_msg.y = TM[i].at<float>(1,0);
        //      pos_msg.z = TM[i].at<float>(2,0);


        //      pos_msg.x = -TM[i].at<float>(1,0);
        //      pos_msg.y =  TM[i].at<float>(0,0);
        //      pos_msg.z =  TM[i].at<float>(2,0);

        pos_msg.x =  TM[i].at<float>(0,0);
        pos_msg.y =  TM[i].at<float>(1,0);
        pos_msg.z =  TM[i].at<float>(2,0);

        rot_msg.x = RM[i].at<float>(0,0);
        rot_msg.y = RM[i].at<float>(1,0);
        rot_msg.z = RM[i].at<float>(2,0);
        float mod_rotation;
        mod_rotation =sqrt(pow(RM[i].at<float>(0,0) , 2) +
                           pow(RM[i].at<float>(1,0) , 2) +
                           pow(RM[i].at<float>(2,0) , 2) );

        cout << "mod of rot is" << mod_rotation <<endl;

        tf::Quaternion q;

        q.setRotation(tf::Vector3(RM[i].at<float>(0,0),
                                  RM[i].at<float>(1,0),
                                  RM[i].at<float>(2,0)),
                      tfScalar(mod_rotation) );
        pose_msg.header.frame_id   = "tag";
        pose_msg.pose.orientation.x = q.getX();
        pose_msg.pose.orientation.y = q.getY();
        pose_msg.pose.orientation.z = q.getZ();
        pose_msg.pose.orientation.w = q.getW();

        pose_msg.pose.position.x = TM[i].at<float>(0,0);
        pose_msg.pose.position.y = TM[i].at<float>(1,0);
        pose_msg.pose.position.z = TM[i].at<float>(2,0);

        position_pub.publish(pos_msg);
        rotation_pub.publish(rot_msg);
        pose_pub.publish(pose_msg);
    }
    if(Markers.size() ==0)
    {
        marker_status_msg.data = 0;
    }
    else
    {
        marker_status_msg.data = 1;
    }
    marker_status_pub.publish(marker_status_msg);
    imshow("image_show", image_in);
    //imshow("thres", MDetector.getThresholdedImage());

    cout << "Time detection=" <<  1000*( (double)getTickCount() - tick) / getTickFrequency() << " milliseconds" << endl;


}

void cvTackBarEvents(int pos, void *)
{


}

void init()
{


}

int main(int argc,char **argv)
{
    ros::init(argc, argv, "aruco_detect");
    ros::NodeHandle n,nh;

    image_transport::ImageTransport it(nh);

    nh.param("/aruco/camera_file", camera_file, string("~/ws/src/dji_cv/yml/camera.yml") );
    nh.param("/aruco/marker_file", marker_file, string("~/ws/src/dji_cv/yml/marker.yml") );
    nh.param("/aruco/image_topic", image_topic, string("/camera/camera_rgb8") );

    //position_pub = n.advertise<geometry_msgs::Vector3>("target_position",50);
    position_pub = n.advertise<geometry_msgs::Vector3>("/marker_position",50);
    pose_pub = n.advertise<geometry_msgs::PoseStamped>("/marker_pose",50);
    rotation_pub = n.advertise<geometry_msgs::Vector3>("/marker_rotation",50);
    marker_status_pub = n.advertise<std_msgs::UInt8>("/marker_status",50);
    image_sub = it.subscribe(image_topic , 2, imageCallback);

    cout<<"camera_file is "<<camera_file<<endl;
    Dictionary D;
    if (D.fromFile(marker_file) == false) {
        cerr << "Could not open dictionary" << endl;
        return -1;
    };

    if (D.size() == 0) {
        cerr << "Invalid dictionary" << endl;
        return -1;
    };


    HighlyReliableMarkers::loadDictionary(D);

    TheCameraParameters.readFromXMLFile(camera_file);
    //TheCameraParameters.resize(Size(640,480));

    // Configure other parameters
    if (ThePyrDownLevel > 0)
        MDetector.pyrDown(ThePyrDownLevel);

    MDetector.enableLockedCornersMethod(lockedCorners);
    MDetector.setMakerDetectorFunction(aruco::HighlyReliableMarkers::detect);
    MDetector.setThresholdParams(21, 7);
    MDetector.setCornerRefinementMethod(aruco::MarkerDetector::LINES);
    MDetector.setWarpSize((D[0].n() + 2) * 8);
    MDetector.setMinMaxSize(0.005, 0.5);

    namedWindow("image_show", 1);
    //namedWindow("erode", 1);
    //namedWindow("dilate", 1);
    //namedWindow("threshold", 1);

    MDetector.getThresholdParams(ThresParam1, ThresParam2);
    iThresParam1 = ThresParam1;
    iThresParam2 = ThresParam2;
    createTrackbar("ThresParam1", "image_show", &iThresParam1, 100, cvTackBarEvents);
    createTrackbar("ThresParam2", "image_show", &iThresParam2, 100, cvTackBarEvents);
    createTrackbar("threshold_value", "threshold", &thresholdValue, 255, cvTackBarEvents);
    ros::Rate sl(50);

    while(n.ok() && waitKey(30) != 'q')
    {
        sl.sleep();
        ros::spinOnce();
        cout<<frame_count<<endl;
        if(!image.empty())
        {
            //imshow("image",image);
        }
    }



    return 0;
}

