#include <iostream>
#include <cmath>
#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Int16.h>
#include <cv_bridge/cv_bridge.h>
#include <nav_msgs/Odometry.h>
#include <aruco/aruco.h>
#include <aruco/cvdrawingutils.h>
#include <opencv2/opencv.hpp>
#include <Eigen/Eigen>
//#include <Eigen/SVD>

using namespace cv;
using namespace aruco;
using namespace Eigen;

float MarkerSize = 0.1;
aruco::CameraParameters CamParam;
MarkerDetector MDetector;
vector<Marker> Markers;
int query_id = 10;
ros::Publisher pub_ar_odom;

void query_id_callback(const std_msgs::Int16::ConstPtr& id_msg)
{
    query_id = id_msg->data;
}

void img_callback(const sensor_msgs::ImageConstPtr &img_msg)
{
    cv_bridge::CvImagePtr bridge_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);
    cv::Mat frame = bridge_ptr->image;
    MDetector.detect(frame, Markers);
    
    for (unsigned int i = 0; i < Markers.size(); i++)
    {

        //cout << Markers[i] << endl;
        Markers[i].draw(frame, Scalar(0,0,255), 2);
        aruco::CvDrawingUtils::draw3dAxis(frame, Markers[i], CamParam);
        Markers[i].calculateExtrinsics(MarkerSize, CamParam);
        
        if(query_id != Markers[i].id)
            continue;
        cv::Mat rvec = Markers[i].Rvec;
        cv::Mat tvec = Markers[i].Tvec;
        cv::Mat R;
        cv::Rodrigues(rvec, R);
        Matrix3d R_eigen;
        for(int j=0;j<3;j++)
            for(int k=0;k<3;k++)
            {
                R_eigen(j,k) = R.at<float>(j,k);
            }
        Quaterniond Q;
        Q = R_eigen;
        nav_msgs::Odometry odom_marker;
        odom_marker.header.stamp = img_msg->header.stamp;
        odom_marker.header.frame_id = "camera"; //TODO: ???
        odom_marker.pose.pose.position.x = tvec.at<float>(0,0);
        odom_marker.pose.pose.position.y = tvec.at<float>(1,0);
        odom_marker.pose.pose.position.z = tvec.at<float>(2,0);
        cout << "tvec:  " << tvec.at<float>(0,0) << endl << tvec.at<float>(1,0) << endl << tvec.at<float>(2,0) << endl << "====" << endl;
        odom_marker.pose.pose.orientation.w = Q.w();
        odom_marker.pose.pose.orientation.x = Q.x();
        odom_marker.pose.pose.orientation.y = Q.y();
        odom_marker.pose.pose.orientation.z = Q.z();
        pub_ar_odom.publish(odom_marker);
    }
    cv::imshow("usb_image", frame);
    cv::waitKey(5);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "marker_detector");
    ros::NodeHandle nh("~");
    string cam_cal;
    nh.getParam("cam_cal_file", cam_cal);
    CamParam.readFromXMLFile(cam_cal);
    
    pub_ar_odom = nh.advertise<nav_msgs::Odometry>("/detected_markers", 10);
    ros::Subscriber sub_img = nh.subscribe("/mv_25001511/image_raw", 1, img_callback);
    cout << "hi" << endl;
    //ros::Subscriber sub_query_id = nh.subscribe("/query_id", 1, query_id_callback);
    

    
    
    
    //cv::namedWindow("dji_image", 1);
    ros::spin();
}
