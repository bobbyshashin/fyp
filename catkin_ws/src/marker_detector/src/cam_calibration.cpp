#include <iostream>
#include <cmath>
#include <ros/ros.h>
#include <ros/console.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <highgui.h>
#include <cv_bridge/cv_bridge.h>
int i = 0;
using namespace cv;

void img_callback(const sensor_msgs::ImageConstPtr &img_msg)
{
    cv_bridge::CvImagePtr bridge_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);
    cv::Mat frame = bridge_ptr->image;
    cv::imshow("usb_image", frame);
    char ch = cv::waitKey(25);
    if(ch=='s'){
        std::ostringstream name;
        name <<  i << ".jpg";
        cv::imwrite(name.str(), frame);
	//cout << "Save !!!" << endl;
        i++;
    }
    
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "cam_calibration");
    ros::NodeHandle nh("~");

    ros::Subscriber sub_img = nh.subscribe("/mv_25001511/image_raw", 1, img_callback);

    ros::spin();
}
