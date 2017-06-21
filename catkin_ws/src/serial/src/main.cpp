#include "main.h"
#include <ros/ros.h>
#include <Eigen/Eigen>
#include <math.h>
//#include <tf/transform_broadcaster.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>
//#define FRAME_CNT 5000
#include <string>
#include <iostream>
#include <stdio.h>
#include <string>
#include <cctype>
#define RX_TIMEOUT 30
#define TX_TIMEOUT 30
#define BUF_LEN 1
//This node only send single byte.
using namespace Eigen;
using namespace std;
string name;
string target_status;

void ugv_callback(const nav_msgs::Odometry::ConstPtr &msg
)
{
    float x_vel = msg->twist.twist.linear.x;
    float y_vel = msg->twist.twist.linear.y;
    float x_ang = msg->twist.twist.angular.x;
    float y_ang = msg->twist.twist.angular.y;
    Quaterniond ori(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,msg->pose.pose.orientation.y,msg->pose.pose.orientation.z);
    Matrix3d Rgi = ori.toRotationMatrix();
    float phi = asin(Rgi(2,1));
    float yaw = acos(Rgi(1,1)/cos(phi));
    unsigned char start_mark = 0xa5;
    unsigned char end_mark = 0xa6;

    //unsigned char data[100] = "1.233"; //TODO
    //int data_len = sizeof(data)/sizeof(data[0]);
    //data[0] = 0xa5;
    x_vel = roundf(x_vel*100) / 100;
    y_vel = roundf(y_vel*100) / 100;
    x_ang = roundf(x_ang*100) / 100;
    y_ang = roundf(y_ang*100) / 100;
    yaw = roundf(yaw*100) / 100;
    string x_vel_str = to_string(x_vel);
    string y_vel_str = to_string(y_vel);
    string x_ang_str = to_string(x_ang);
    string y_ang_str = to_string(y_ang);
    string yaw_str = to_string(yaw);
    string data_str = start_mark + '0' + 'a' + x_vel_str + 'b' + y_vel_str + 'c' + x_ang + 'd' + y_ang + 'e' + yaw + end_mark;
    int data_len = data_str.length();
    unsigned char data[100] = data_str;
    uint32_t send_total_len = 1;
    send_total_len += write_serial(data,data_len, RX_TIMEOUT);
}

int main(int argc, char* argv[])
{


    ros::init(argc, argv, "serial_node");
    ros::NodeHandle n("~");
    name = ros::this_node::getName();
   
    ros::Subscriber s1 = n.subscribe("/n3_sdk/odometry", 1, ugv_callback);
    
    std::string port;
    if (n.hasParam("port"))
      n.getParam("port", port);
    else
      {
        ROS_ERROR("%s: must provide a port", name.c_str());
        return -1;
      }
    const char* port_name = port.c_str();



    if(connect_serial(port_name) == -1)
	{
		printf("serial open error!\n");
		return -1;
	}

    //ros::Rate r(20);

    while(n.ok()){

        ros::spinOnce();
        //r.sleep();

    }
    
    disconnect_serial();
    return 0;

}
