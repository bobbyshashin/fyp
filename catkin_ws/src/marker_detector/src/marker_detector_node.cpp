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
int query_id = 0;
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
                R_eigen(j,k) = R.at<double>(j,k);
            }
        Quaterniond Q;
        Q = R_eigen;
        nav_msgs::Odometry odom_marker;
        odom_marker.header.stamp = img_msg->header.stamp;
        odom_marker.header.frame_id = "world"; //TODO: ???
        odom_marker.pose.pose.position.x = tvec.at<double>(0,0);
        odom_marker.pose.pose.position.y = tvec.at<double>(1,0);
        odom_marker.pose.pose.position.z = tvec.at<double>(2,0);
        odom_marker.pose.pose.orientation.w = Q.w();
        odom_marker.pose.pose.orientation.x = Q.x();
        odom_marker.pose.pose.orientation.y = Q.y();
        odom_marker.pose.pose.orientation.z = Q.z();
        pub_ar_odom.publish(odom_marker);
    }
    cv::imshow("usb_image", frame);
    cv::waitKey(10);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "marker_detector");
    ros::NodeHandle n("~");

    ros::Subscriber sub_img = n.subscribe("/usb_cam/image_raw", 100, img_callback);
    ros::Subscriber sub_query_id = n.subscribe("/query_id", 100, query_id_callback);
    string cam_cal;

    n.getParam("cam_cal_file", cam_cal);
    CamParam.readFromXMLFile(cam_cal);
    cv::namedWindow("usb_image", 1);
    ros::spin();
}
