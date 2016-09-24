/**
******************************************************************************
* @file    aruco_board.cxx
* @author  FrankChen
* @version V3.3.5
* @date    30-May-2016
* @brief   This file provides basic funtion that detect aruco board marker and publish pose data to ros topic.
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
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Vector3.h"
#include <tf/transform_broadcaster.h>

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
#include "aruco/boarddetector.h"
#include "aruco/cvdrawingutils.h"

//include eigen library
#include "eigen3/Eigen/Dense"

using namespace Eigen;
using namespace cv;
using namespace std;
using namespace aruco;

ros::Publisher position_pub;
ros::Publisher posi_pub;
ros::Publisher board_status_pub;
//vector <ros::Publisher> pose_pub;

ros::Publisher pose_pub_1;
ros::Publisher pose_pub_2;
ros::Publisher pose_pub_3;
ros::Publisher pose_pub_4;

ros::Publisher imgpoint_pub_1;
ros::Publisher imgpoint_pub_2;
ros::Publisher imgpoint_pub_3;
ros::Publisher imgpoint_pub_4;

ros::Publisher rotation_pub;
image_transport::Subscriber image_sub;

long frame_count=0;
unsigned char board_status=0x00;
Mat image;
Mat gray_img,blur_img,corner_img,nor_img,scal_img;

Mat img_point(6,2,CV_8UC1);
Mat obj_point(6,3,CV_8UC1);
Mat rvec,tvec;

vector< Marker > Markers;
MarkerDetector MDetector;
float TheMarkerSize = 0.15;

std_msgs::String str_msg;
std_msgs::Char board_status_msg;
geometry_msgs::Vector3 pos_msg;
geometry_msgs::PoseStamped pose_msg;
geometry_msgs::Vector3 ang_msg;
geometry_msgs::Point point_msg;
geometry_msgs::Pose ppp;
tf::TransformBroadcaster br;
CameraParameters TheCameraParameters;
BoardConfiguration TheBoardConfig[4];
BoardDetector TheBoardDetector;
Board TheBoardDetected[4];

Mat Rvec[4],Tvec[4];
Mat RM[4],TM[4];

string TheBoardConfigFile,TheIntrinsicFile;
string  boardcfg[4];

int saveMat(Mat src)
{
    image = src.clone();
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    vector< geometry_msgs::Pose > pose_t_msg;
    Mat image_callback,image_in;
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
    image_callback = cv_ptr -> image.clone();
    image_in = cv_ptr -> image.clone();
    saveMat(image_in);
    //cvtColor(image_callback,gray_img,CV_RGB2GRAY);
    gray_img=image_callback.clone();
    // Detection of the marker
    MDetector.detect(gray_img, Markers, TheCameraParameters, TheMarkerSize);

    // Detection of the board

    float probDetect[4]={0,0,0,0};
    bool is_calc_reliable;
    is_calc_reliable = true;
    for(int i=0;i<4;i++)
    {
        probDetect[i] = TheBoardDetector.detect(Markers, TheBoardConfig[i], TheBoardDetected[i], TheCameraParameters, TheMarkerSize);
        if((probDetect[i]) == 0) { is_calc_reliable = false;}
        cout<<"prob ["<<i<<"] = "<<probDetect[i]<<endl;
    }
    unsigned char ch = 0x01;
    for(int i=0;i<4;i++)
    {
        if(probDetect[i]>=0.5)
        {
            board_status |= ch;
            ch=ch<<1;
            printf("ch : %d \n",ch);
            printf("board_status : %d \n",board_status);
        }
    }
    board_status_msg.data = board_status;
    board_status_pub.publish(board_status_msg);
    for (unsigned int i = 0; i < Markers.size(); i++) {
        //cout << Markers[i] << endl;
        //Markers[i].draw(image_callback, Scalar(0, 0, 255), 2);
    }

    // draw a 3d cube in each marker if there is 3d info
    if (TheCameraParameters.isValid() && TheMarkerSize != -1)
    {
        for (unsigned int i = 0; i < Markers.size(); i++)
        {
            //CvDrawingUtils::draw3dCube(image_callback, Markers[i], TheCameraParameters);
            //CvDrawingUtils::draw3dAxis(image_callback, Markers[i], TheCameraParameters);
        }
        for (int i=0;i<4;i++)
        {
            if(probDetect[i]>0)
                CvDrawingUtils::draw3dAxis(image_callback, TheBoardDetected[i], TheCameraParameters);

        }

        for(int i=0;i<4;i++)
            cout << "Board " << i << endl <<"Rvec "<<TheBoardDetected[i].Rvec << endl << "Tvec" << TheBoardDetected[i].Tvec << endl;
        //scout << "TheBoard"<<i<<TheBoardDetected[i].Tvec << endl;


        for(int i=0;i<4;i++)
        {
            RM[i]=TheBoardDetected[i].Rvec;
            TM[i]=TheBoardDetected[i].Tvec;
        }


        float prob=0;
        for(int i=0;i<4;i++)
        {
            prob=prob+probDetect[i];
        }
        for(int i=0;i<4;i++)
        {
            pos_msg.x = TM[i].at<float>(0,0);
            pos_msg.y = TM[i].at<float>(1,0);
            pos_msg.z = TM[i].at<float>(2,0);

            ang_msg.x = RM[i].at<float>(0,0);
            ang_msg.y = RM[i].at<float>(1,0);
            ang_msg.z = RM[i].at<float>(2,0);

            pose_msg.header.frame_id = "Aruco_Pose";
            pose_msg.header.stamp = ros::Time::now();

            pose_msg.pose.position.x = pos_msg.x;
            pose_msg.pose.position.y = pos_msg.y;
            pose_msg.pose.position.z = pos_msg.z;

            pose_msg.pose.orientation.x = ang_msg.x;
            pose_msg.pose.orientation.y = ang_msg.y;
            pose_msg.pose.orientation.z = ang_msg.z;
            pose_msg.pose.orientation.w = probDetect[i];


            if(i==0) pose_pub_1.publish(pose_msg);
            if(i==1) pose_pub_2.publish(pose_msg);
            if(i==2) pose_pub_3.publish(pose_msg);
            if(i==3) pose_pub_4.publish(pose_msg);

            position_pub.publish(pos_msg);
            rotation_pub.publish(ang_msg);


        }
        frame_count++;

    }

    vector <Point2f> imagePoint_0,imagePoint_1,imagePoint_2,imagePoint_3;



    Mat objectPoint_1(1, 3, CV_32FC1);
    Mat objectPoint_2(1, 3, CV_32FC1);
    Mat objectPoint_3(1, 3, CV_32FC1);
    Mat objectPoint_4(1, 3, CV_32FC1);

    objectPoint_1.at< float >(0, 0) = 0;
    objectPoint_1.at< float >(0, 1) = 0;
    objectPoint_1.at< float >(0, 2) = 0;

    objectPoint_2.at< float >(0, 0) = 0;
    objectPoint_2.at< float >(0, 1) = 0;
    objectPoint_2.at< float >(0, 2) = 0;

    objectPoint_3.at< float >(0, 0) = 0;
    objectPoint_3.at< float >(0, 1) = 0;
    objectPoint_3.at< float >(0, 2) = 0;

    objectPoint_4.at< float >(0, 0) = 0;
    objectPoint_4.at< float >(0, 1) = 0;
    objectPoint_4.at< float >(0, 2) = 0;


    projectPoints(objectPoint_1, TheBoardDetected[0].Rvec, TheBoardDetected[0].Tvec,
            TheCameraParameters.CameraMatrix, TheCameraParameters.Distorsion, imagePoint_0);
    projectPoints(objectPoint_2, TheBoardDetected[1].Rvec, TheBoardDetected[1].Tvec,
            TheCameraParameters.CameraMatrix, TheCameraParameters.Distorsion, imagePoint_1);
    projectPoints(objectPoint_3, TheBoardDetected[2].Rvec, TheBoardDetected[2].Tvec,
            TheCameraParameters.CameraMatrix, TheCameraParameters.Distorsion, imagePoint_2);
    projectPoints(objectPoint_4, TheBoardDetected[3].Rvec, TheBoardDetected[3].Tvec,
            TheCameraParameters.CameraMatrix, TheCameraParameters.Distorsion, imagePoint_3);

    cout<<"Points: X: "<<endl<<imagePoint_0[0].x<<" , Y: "<<imagePoint_0[0].y<<endl;

    point_msg.x = imagePoint_0[0].x;
    point_msg.y = imagePoint_0[0].y;
    point_msg.z = probDetect[0];
    imgpoint_pub_1.publish(point_msg);

    point_msg.x = imagePoint_1[0].x;
    point_msg.y = imagePoint_1[0].y;
    point_msg.z = probDetect[1];
    imgpoint_pub_2.publish(point_msg);

    point_msg.x = imagePoint_2[0].x;
    point_msg.y = imagePoint_2[0].y;
    point_msg.z = probDetect[2];
    imgpoint_pub_3.publish(point_msg);

    point_msg.x = imagePoint_3[0].x;
    point_msg.y = imagePoint_3[0].y;
    point_msg.z = probDetect[3];
    imgpoint_pub_4.publish(point_msg);

    // draw lines of different colours
    cv::line(image_in, imagePoint_0[0], imagePoint_1[0], Scalar(0, 255, 0, 160), 2, CV_AA);
    cv::line(image_in, imagePoint_1[0], imagePoint_2[0], Scalar(0, 255, 0, 160), 2, CV_AA);
    cv::line(image_in, imagePoint_2[0], imagePoint_3[0], Scalar(0, 255, 0, 160), 2, CV_AA);
    cv::line(image_in, imagePoint_3[0], imagePoint_0[0], Scalar(0, 255, 0, 160), 2, CV_AA);

    putText(image_in, "P_1", imagePoint_0[0], FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255, 150), 2);
    putText(image_in, "P_2", imagePoint_1[0], FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255, 150), 2);
    putText(image_in, "P_3", imagePoint_2[0], FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255, 150), 2);
    putText(image_in, "P_4", imagePoint_3[0], FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255, 150), 2);



    imshow("Retange_draw",image_in);
    imshow("Aruco_detect",image_callback);

}



int main(int argc,char **argv)
{
    ros::init(argc, argv, "structured_light");
    ros::NodeHandle n,nh;

    Mat image_tmp;

    image_transport::ImageTransport it(nh);

    position_pub = n.advertise<geometry_msgs::Vector3>("board_position",50);

    pose_pub_1 = n.advertise<geometry_msgs::PoseStamped>("board_pose_1",50);
    pose_pub_2 = n.advertise<geometry_msgs::PoseStamped>("board_pose_2",50);
    pose_pub_3 = n.advertise<geometry_msgs::PoseStamped>("board_pose_3",50);
    pose_pub_4 = n.advertise<geometry_msgs::PoseStamped>("byoard_pose_4",50);


    imgpoint_pub_1 = n.advertise<geometry_msgs::Point>("image_point_1",50);
    imgpoint_pub_2 = n.advertise<geometry_msgs::Point>("image_point_2",50);
    imgpoint_pub_3 = n.advertise<geometry_msgs::Point>("image_point_3",50);
    imgpoint_pub_4 = n.advertise<geometry_msgs::Point>("image_point_4",50);

    rotation_pub = n.advertise<geometry_msgs::Vector3>("board_rotation",50);
    posi_pub = n.advertise<geometry_msgs::Pose>("board_posi",50);
    board_status_pub = n.advertise<std_msgs::Char>("board_status",50);

    image_sub = it.subscribe("camera/ir" , 10, imageCallback);


    if(argc>1)
    {
        TheBoardConfigFile = argv[1];
        if(argc>2)
        {
            TheIntrinsicFile = argv[2];
            if(argc>3)
                TheMarkerSize = atof(argv[3]);
        }
    }
    else
    {
        cout<<"usage: --\"board config\" --\"Marker size\" "<<endl;
        return 0;
    }





    FileStorage fs1(TheBoardConfigFile,FileStorage::READ) ;

    fs1["board_config_file_1"] >> boardcfg[0] ;
    fs1["board_config_file_2"] >> boardcfg[1] ;
    fs1["board_config_file_3"] >> boardcfg[2] ;
    fs1["board_config_file_4"] >> boardcfg[3] ;
    //    fs1["board_config_file_5"] >> boardcfg[4] ;
    //    fs1["board_config_file_6"] >> boardcfg[5] ;
    //    fs1["board_config_file_7"] >> boardcfg[6] ;
    //    fs1["board_config_file_8"] >> boardcfg[7] ;
    //    fs1["board_config_file_9"] >> boardcfg[8] ;

    /**************************************************/
    for(int i=0;i<4;i++) cout<<boardcfg[i] << endl;

    /**************************************************/
    fs1.release();

    for(int i=0;i<4;i++) TheBoardConfig[i].readFromFile ( boardcfg[i] ) ;

    TheCameraParameters.readFromXMLFile(TheIntrinsicFile);
    //TheCameraParameters.resize(image_callback.size());

    MDetector.setThresholdParams(7, 7);
    MDetector.setThresholdParamRange(2, 0);

    namedWindow("image_share", 1);

    while(n.ok() && waitKey(30) != 'q')
    {
        ros::spinOnce();
        cout<<frame_count<<endl;
        if(!image.empty())
        {
            imshow("image_share",image);
        }
    }



    return 0;
}

