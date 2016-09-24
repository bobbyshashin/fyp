#include <stdio.h>
#include <cstdlib>
#include <iostream>
#include <fstream>
#include <sstream>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include "geometry_msgs/PoseStamped.h"
#include "std_msgs/UInt8.h"
#include "nav_msgs/Odometry.h"
#include "dji_sdk/AttitudeQuaternion.h"

using namespace std;
using namespace ros;

string str_fixed  = "fixed";
string str_world  = "world";
string str_uav    = "uav";
string str_camera = "camera";
string str_tag    = "tag";


//#define USE_ATTITUDE
#define USE_ODOMETRY

ros::Subscriber tf_world_origin;
ros::Subscriber tf_uav_pose;
ros::Subscriber tf_camera_pose;
ros::Subscriber tf_imu_origin;
ros::Subscriber tf_tag_pose;

void tf_world_origin_Callback(const geometry_msgs::PoseStampedConstPtr& msg)
{
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    tf::Quaternion q;
    transform.setOrigin( tf::Vector3(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z) );
    transform.setRotation( tf::Quaternion(msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w) );
    cout<<"world_set!"<<endl;
    //q.setRPY(0, 0, 0);
    msg->header.frame_id;
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), str_fixed, str_world));
}


void tf_uav_pose_Callback(const geometry_msgs::PoseStampedConstPtr& msg)
{
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    tf::Quaternion q;
    transform.setOrigin( tf::Vector3(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z) );
    transform.setRotation( tf::Quaternion(msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w) );
    cout<<"uav_set!"<<endl;
    //q.setRPY(0, 0, 0);

    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), str_world, str_uav));
}

void tf_camera_pose_Callback(const geometry_msgs::PoseStampedConstPtr& msg)
{
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    tf::Quaternion q;
    transform.setOrigin( tf::Vector3(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z) );
    transform.setRotation( tf::Quaternion(msg->pose.orientation.x, msg->pose.orientation.y,
                                          msg->pose.orientation.z, msg->pose.orientation.w) );
    cout<<"camera_set!"<<endl;
    //q.setRPY(0, 0, 0);

    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), str_uav, str_camera));
}

void tf_tag_pose_Callback(const geometry_msgs::PoseStampedConstPtr& msg)
{
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    tf::Quaternion q;
    transform.setOrigin( tf::Vector3(msg->pose.position.x,
                                     msg->pose.position.y,
                                     msg->pose.position.z) );
    transform.setRotation( tf::Quaternion(msg->pose.orientation.y,
                                          msg->pose.orientation.x,
                                          msg->pose.orientation.z,
                                          msg->pose.orientation.w) );

    cout<<"tag_set!"<<endl;
    //q.setRPY(0, 0, 0);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), str_camera, str_tag));
}


#ifdef USE_ATTITUDE

void tf_imu_origin_Callback(const dji_sdk::AttitudeQuaternionConstPtr& msg)
{
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    tf::Quaternion q;
    transform.setOrigin( tf::Vector3(0, 0, 0.5) );
    transform.setRotation( tf::Quaternion(msg->q1, -msg->q2, -msg->q3, msg->q0) );
    cout<<"dji_odometry_set!"<<endl;
    //q.setRPY(0, 0, 0);

    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), str_world, str_uav));
}
#endif

#ifdef USE_ODOMETRY

void tf_imu_origin_Callback(const nav_msgs::OdometryConstPtr& msg)
{
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    //tf::Quaternion q;
    transform.setOrigin( tf::Vector3(-msg->pose.pose.position.x,
                                     msg->pose.pose.position.y,
                                     msg->pose.pose.position.z) );
    transform.setRotation( tf::Quaternion(-msg->pose.pose.orientation.x,
                                          msg->pose.pose.orientation.y,
                                          msg->pose.pose.orientation.z,
                                          msg->pose.pose.orientation.w) );
    cout<<"guidance_set!"<<endl;
    //q.setRPY(0, 0, 0);

    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), str_world, str_uav));
}
#endif

int main(int argc, char** argv)
{
    ros::init(argc, argv, "dji_tf");
    ros::NodeHandle nh;
    string imu_or_guidance;

    nh.param("/dji_tf/odometry_source", imu_or_guidance, string("guidance") );
    cout<<"odometry from "<< imu_or_guidance<< endl;

    tf_world_origin   = nh.subscribe("/world_origin", 50, tf_world_origin_Callback );
    tf_uav_pose       = nh.subscribe("/uav_pose",     50, tf_uav_pose_Callback     );
    tf_camera_pose    = nh.subscribe("/camera_pose",  50, tf_camera_pose_Callback  );
#ifdef USE_ATTITUDE
    tf_imu_origin     = nh.subscribe("/dji_sdk/attitude_quaternion",  50, tf_imu_origin_Callback  );
#endif

#ifdef USE_ODOMETRY
    tf_imu_origin     = nh.subscribe("/guidance/odometry",  50, tf_imu_origin_Callback  );
#endif
    tf_tag_pose       = nh.subscribe("/marker_pose",  50, tf_tag_pose_Callback  );
    ros::Rate sl(50);


    while(nh.ok())
    {
        sl.sleep();
        ros::spinOnce();

    }

    return 0;
}

