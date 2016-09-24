/**
******************************************************************************
* @file    feature_extract.cpp
* @author  FrankChen
* @version V1.5.2
* @date    31-May-2016
* @brief   This file provides basic funtion that extract feature from image.
******************************************************************************
*/
//include ros library
#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Vector3.h"
#include "std_msgs/Char.h"
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

ros::Subscriber imgpoint_sub_1;
ros::Subscriber imgpoint_sub_2;
ros::Subscriber imgpoint_sub_3;
ros::Subscriber imgpoint_sub_4;

ros::Subscriber board_status_sub;

Point3f image_point_1,image_point_2,image_point_3,image_point_4;
vector<Point2f> board_point;
Mat rosImage;
int thresholdValue=45;
int maxCornerNumber = 60;
double qualityLevel = 0.01;
double minDistance = 5;
int blockSize = 5;
double k = 0.04;
unsigned char board_status=0x00;
int detectVal=10;


image_transport::Publisher  image_pub;
image_transport::Subscriber image_sub;

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
    //cv_ptr->header.stamp;
    transportMat(image_in);
}

void image_point_1_callback(const geometry_msgs::PointConstPtr& msg)
{
    image_point_1.x = msg->x;
    image_point_1.y = msg->y;
    image_point_1.z = msg->z;

}

void image_point_2_callback(const geometry_msgs::PointConstPtr& msg)
{
    image_point_2.x = msg->x;
    image_point_2.y = msg->y;
    image_point_2.z = msg->z;

}

void image_point_3_callback(const geometry_msgs::PointConstPtr& msg)
{
    image_point_3.x = msg->x;
    image_point_3.y = msg->y;
    image_point_3.z = msg->z;

}

void image_point_4_callback(const geometry_msgs::PointConstPtr& msg)
{
    image_point_4.x = msg->x;
    image_point_4.y = msg->y;
    image_point_4.z = msg->z;

}

void board_status_callback(const std_msgs::CharConstPtr& msg)
{


}

void Shi_Tomas(Mat src, Mat gray, vector< Point2f > &_corners)
{
    RNG g_rng(12345);
    Mat copy = src.clone();
    //Mat gray;
    //gray = _gray.clone();
    goodFeaturesToTrack( copy,
                         _corners,
                         maxCornerNumber,
                         qualityLevel,
                         minDistance,
                         Mat(),
                         blockSize,
                         false,
                         k );


    cout<<"The Point that we detected: "<<_corners.size()<<endl;

    int r = 3;
    for( int i = 0; i < _corners.size(); i++ )
    {

        circle( gray, _corners[i], r, Scalar(g_rng.uniform(0,255), g_rng.uniform(0,255),
                                             g_rng.uniform(0,255)), -1, 8, 0 );
    }

    imshow( "Shi-Tomas", gray );

}

void parameter(int , void*)
{


}

void salt(Mat& image, int n)
{
    for(int k=0; k<n; k++)
    {
        int i = rand()%image.cols;
        int j = rand()%image.rows;

        if(image.channels() == 1)
        {
            image.at<uchar>(j,i) = 255;
        }
        else
        {
            image.at<Vec3b>(j,i)[0] = 255;
            image.at<Vec3b>(j,i)[1] = 255;
            image.at<Vec3b>(j,i)[2] = 255;
        }
    }
}

double generateGaussianNoise()
{
    static bool hasSpare = false;
    static double rand1, rand2;

    if(hasSpare)
    {
        hasSpare = false;
        return sqrt(rand1) * sin(rand2);
    }

    hasSpare = true;

    rand1 = rand() / ((double) RAND_MAX);
    if(rand1 < 1e-100) rand1 = 1e-100;
    rand1 = -2 * log(rand1);
    rand2 = (rand() / ((double) RAND_MAX)) * 2 *CV_PI;

    return sqrt(rand1) * cos(rand2);
}


void AddGaussianNoise(Mat& I)
{
    // accept only char type matrices
    CV_Assert(I.depth() != sizeof(uchar));

    int channels = I.channels();

    int nRows = I.rows;
    int nCols = I.cols * channels;

    if(I.isContinuous()){
        nCols *= nRows;
        nRows = 1;
    }

    int i,j;
    uchar* p;
    for(i = 0; i < nRows; ++i){
        p = I.ptr<uchar>(i);
        for(j = 0; j < nCols; ++j){
            double val = p[j] + generateGaussianNoise() * 16;
            if(val < 0)
                val = 0;
            if(val > 255)
                val = 255;

            p[j] = (uchar)val;

        }
    }

}

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "feature_extract");
    ros::NodeHandle n,nh;
    image_transport::ImageTransport it(nh);

    image_sub = it.subscribe("/camera/ir" , 10, imageCallback);
    //image_sub = it.subscribe("/aruco_board_roi" , 10, imageCallback);
    imgpoint_sub_1 = n.subscribe("/image_point_1" , 50 , image_point_1_callback);
    imgpoint_sub_2 = n.subscribe("/image_point_2" , 50 , image_point_2_callback);
    imgpoint_sub_3 = n.subscribe("/image_point_3" , 50 , image_point_3_callback);
    imgpoint_sub_4 = n.subscribe("/image_point_4" , 50 , image_point_4_callback);
    board_status_sub = n.subscribe("/board_status" , 50 , board_status_callback);

    Mat show;
    Mat hsvimage;

    Mat ROI;
    Mat gaussImage,grayImage,cannyImage,
            erodeImage,dilateImage,thresholdImage,show_medianBlur,show_gaussBlur
            ;
    string imagename = argv[1];
    Mat image = imread(imagename);
    namedWindow("threshold");
    namedWindow("dilate");
    namedWindow("image");
    namedWindow("Shi-Tomas");
    createTrackbar("MaxCorner","image",&maxCornerNumber,200,parameter);
    createTrackbar("Threshold","image",&thresholdValue,255,parameter);
    createTrackbar("Detect Threshold","image",&detectVal,1000,parameter);

//    while(rosImage.empty())
//        ros::spinOnce();
    while(n.ok()&&('q' != waitKey(30)) )
    {

        show = image.clone();
//        AddGaussianNoise(show);
//        salt(show,300);
//        medianBlur(show,show_medianBlur,3);
//        imshow("image_medianBlur",show_medianBlur);

        //Gaussian Blur structrued_light
//        GaussianBlur(show_medianBlur,show_gaussBlur,Size(7,7),0,0);
//        imshow("image_gaussBlur",show_gaussBlur);
        //get gray image
        cvtColor(image,grayImage,CV_BGR2GRAY);
        cvtColor(image,hsvimage,CV_BGR2HSV);

  //      grayImage = rosImage.clone();
        //bilateralFilter(rosImage,grayImage,2,2,2);
        //imshow("bilatera",grayImage);
        //
        //adaptiveThreshold(grayImage,thresholdImage,255,CV_ADAPTIVE_THRESH_GAUSSIAN_C,CV_THRESH_BINARY_INV,25,3);
//        threshold(grayImage,thresholdImage,thresholdValue,255,CV_THRESH_BINARY);
        //threshold(grayImage,thresholdImage,thresholdValue,255,CV_THRESH_OTSU);
        //imshow("threshold",thresholdImage);
        //GaussianBlur(thresholdImage,thresholdImage,Size(3,3),0,0);
        vector<Mat> hsv_ch;
        split(hsvimage,hsv_ch);
        imshow("H",hsv_ch.at(0));
        imshow("S",hsv_ch.at(1));
        imshow("V",hsv_ch.at(2));
        Mat Erode_element = getStructuringElement(MORPH_RECT, Size(11, 11));
        erode(grayImage, erodeImage, Erode_element);
        imshow("erode",erodeImage);

        Mat dilate_element = getStructuringElement(MORPH_RECT, Size(9, 9));
        dilate(erodeImage, dilateImage, dilate_element);
        imshow("dilate",dilateImage);

        threshold(dilateImage,thresholdImage,thresholdValue,255,CV_THRESH_BINARY);
        imshow("threshold",thresholdImage);
        //Shi-Tomasi corner
        vector< Point2f > corners;
        Shi_Tomas(thresholdImage, show, corners);
        cout<<corners.size()<<endl;
        int cornersSize = corners.size();
        //
        if(cornersSize>0)
        {
        float x_means,y_means,x_sum,y_sum;
        for(int i=0;i<cornersSize;i++)
        {
            x_sum+=corners[i].x;
            y_sum+=corners[i].y;
            cout<<"Point "<<i<<"x: "<<corners[i].x<<"Point "<<i<<"y: "<<corners[i].y<<endl;
        }
        x_means = x_sum/cornersSize;
        y_means = y_sum/cornersSize;
        x_sum=0;
        y_sum=0;
        cout<<"X Means-> "<<x_means<<endl<<"Y Means-> "<<y_means<<endl;
        float x_means_square,y_means_square;
        for(int i=0;i<cornersSize;i++)
        {
            x_means_square +=pow((corners[i].x-x_means),2);
            y_means_square +=pow((corners[i].y-y_means),2);
        }
        x_means_square = x_means_square/(cornersSize*cornersSize);
        y_means_square = y_means_square/(cornersSize*cornersSize);
        cout<<"X square Means-> "<<x_means_square<<endl<<"Y square Means-> "<<y_means_square<<endl;
        float yx = y_means_square/x_means_square;
        cout<<"y/x = "<<yx<<endl;
        x_means_square=0;
        y_means_square=0;
        //        Shi_Tomas(dilateImage, show, corners,"Shi-Tomas-dilate");
        //        Shi_Tomas(erodeImage, show, corners,"Shi-Tomas-erode");
        if(yx>((float)detectVal/(float)1000))
            putText(show,"Obstacle has been Detected!!!",Point(10,30), FONT_HERSHEY_SIMPLEX, 1, Scalar(255, 0, 0, 150), 2);
        else
            putText(show,"Sensor find no Obstacle",Point(10,30), FONT_HERSHEY_SIMPLEX, 1, Scalar(255, 0, 0, 150), 2);
        }
        else
            putText(show,"No Obstacle extract!!!",Point(10,30), FONT_HERSHEY_SIMPLEX, 1, Scalar(255, 0, 0, 150), 2);
        imshow("image",show);

        ros::spinOnce();
    }

    return 0;

}
















