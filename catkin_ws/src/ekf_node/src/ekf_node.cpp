#include <iostream>
#include <cmath>
#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Range.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Point.h>
#include <fstream>
#include <std_msgs/Float32.h>
#include "ekf.h"

EKF ekf;
ros::Publisher odom_uav_pub;
ros::Publisher odom_ugv_pub;
ros::Publisher test_pub;
ros::Publisher marker_position_pub;
float uavID = 10;
float uavID2 = 40;
float origin = 30;
float ugvID = 20;
Matrix3d Rgi = Matrix3d::Identity(3,3);
Matrix3d Rgc = Matrix3d::Identity(3,3); //From car to global
Vector3d Tic = Vector3d::Zero(3);
Vector3d Tgo(0,0,0);
Vector3d Tgt;
Vector3d Tgt2;
Matrix3d Ric;
int fuck = 121;
  
 //TODO: These two need to be initialized
//Notes: In Aruco, it is supposed that what we get about transformation is from the marker coordinate system to the camera system. 



Matrix3d Rggg;// = Matrix3d::Identity(3,3); //Rggg is rotation from global to local for UAV
Matrix3d Rggg_car;// = Matrix3d::Identity(3,3);

bool flag = false;
bool flag_ugv = false;
bool flag_target[2] = {false, false};

void uav_vel_callback(const geometry_msgs::Vector3Stamped &msg)
{
  //cout << "uav prop start" << endl;
  ros::Time Time_uav = msg.header.stamp;
  Vector3d u = VectorXd::Zero(3);
  u(0) = msg.vector.x;
  u(1) = msg.vector.y;
  u(2) = msg.vector.z;
  /*
  Vector3d u1 = Rgi.transpose()*u;
  geometry_msgs::Vector3 bodyv;
  bodyv.x = u1(0);
  bodyv.y = u1(1);
  bodyv.z = u1(2);
  test_pub.publish(bodyv);
  */
  u = Rggg.transpose() * u;
  if(ekf.isInit() == false){
    VectorXd mean_init = Eigen::VectorXd::Zero(6);
    ekf.SetInit(mean_init, Time_uav);
    ekf.SetParam(0.01,0.01);
    return;
  }
  //cout << "uav prop done" << endl;
  ekf.UavPropagation(u, Time_uav, Rgi, Rgc);
  //cout << "else" << endl;
  VectorXd mean = ekf.GetState();
  nav_msgs::Odometry pos_ekf;
  pos_ekf.header.frame_id = "map";
  pos_ekf.pose.pose.position.x = mean(0);
  pos_ekf.pose.pose.position.y = mean(1);
  pos_ekf.pose.pose.position.z = mean(2);
  pos_ekf.header.stamp = Time_uav;
  odom_uav_pub.publish(pos_ekf);
  
}
/*
void uav_vel_callback(const nav_msgs::Odometry &msg)
{
  //cout << "uav vel callback done!" << endl;
  ros::Time Time_uav = msg.header.stamp;
  Vector3d u = VectorXd::Zero(3);
  u(0) = msg.twist.twist.linear.x;
  u(1) = msg.twist.twist.linear.y;
  u(2) = msg.twist.twist.linear.z;
  if(ekf.isInit() == false){
    VectorXd mean_init = Eigen::VectorXd::Zero(6);
    ekf.SetInit(mean_init, Time_uav);
    ekf.SetParam(0.01,0.01);
    return;
  }
  ekf.UavPropagation(u, Time_uav, Rgi, Rgc);
  VectorXd mean = ekf.GetState();
  nav_msgs::Odometry pos_ekf;
  pos_ekf.pose.pose.position.x = mean(0);
  pos_ekf.pose.pose.position.y = mean(1);
  pos_ekf.pose.pose.position.z = mean(2);
  pos_ekf.header.stamp = Time_uav;
  odom_uav_pub.publish(pos_ekf);
}
*/
void ugv_vel_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
  
  ros::Time Time_ugv = msg->header.stamp;
  Vector3d u = VectorXd::Zero(3);
  u(0) = msg->twist.twist.linear.x;
  u(1) = msg->twist.twist.linear.y;
  u(2) = msg->twist.twist.linear.z;
  if(ekf.isInit() == false){
    VectorXd mean_init = Eigen::VectorXd::Zero(6);
    ekf.SetInit(mean_init, Time_ugv);
    ekf.SetParam(0.01,0.01);
    return;
  }
  if(!flag_ugv) return;
  u = Rggg_car.transpose() * u;
  geometry_msgs::Vector3 u1;
  u1.x = u(0);
  u1.y = u(1);
  u1.z = u(2);
  test_pub.publish(u1);
  ekf.UgvPropagation(u, Time_ugv, Rgi, Rgc);
  VectorXd mean = ekf.GetState();
  nav_msgs::Odometry pos_ekf;
  pos_ekf.header.frame_id = "map";
  pos_ekf.pose.pose.position.x = mean(3);
  pos_ekf.pose.pose.position.y = mean(4);
  pos_ekf.pose.pose.position.z = mean(5);
  pos_ekf.twist.twist.linear.x = u(0);
  pos_ekf.twist.twist.linear.y = u(1);
  pos_ekf.twist.twist.linear.z = u(2);
  pos_ekf.header.stamp = Time_ugv;
  odom_ugv_pub.publish(pos_ekf);
}

void uav_odom_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
  float id = msg->twist.twist.linear.x;
  if(((id-uavID) < 0.1) && ((id-uavID) > -0.1) && flag_target[0]){
      //cout << "UAV update start" << endl;
      Vector3d Tct,Tgi;
  
      Tct(0) = msg->pose.pose.position.x;
      Tct(1) = msg->pose.pose.position.y;
      Tct(2) = msg->pose.pose.position.z;
      ros::Time Time_update = msg->header.stamp;

  
      Tgi = Tgt -Rggg.transpose() * Rgi * (Tic + Ric * Tct);
      VectorXd odom_x = Eigen::VectorXd::Zero(6);
  
      odom_x.segment<3>(0) = Tgi;  
      ekf.UavOdomUpdate(odom_x, Time_update);
      }
   
   else if(((id-uavID2) < 0.1) && ((id-uavID2) > -0.1) && flag_target[1]){//marker #40 is detected(after the first time)


      Vector3d Tct,Tgi;
  
      Tct(0) = msg->pose.pose.position.x;
      Tct(1) = msg->pose.pose.position.y;
      Tct(2) = msg->pose.pose.position.z;
      ros::Time Time_update = msg->header.stamp;

  
      Tgi = Tgt2 -Rggg.transpose() * Rgi * (Tic + Ric * Tct);
      VectorXd odom_x = Eigen::VectorXd::Zero(6);
  
      odom_x.segment<3>(0) = Tgi;  
      ekf.UavOdomUpdate(odom_x, Time_update);

   }
   
   else if(((id-origin) < 0.1) && ((id-origin) > -0.1)){
      Vector3d Tco, Tgi;
      Tco(0) = msg->pose.pose.position.x;
      Tco(1) = msg->pose.pose.position.y;
      Tco(2) = msg->pose.pose.position.z;
      ros::Time Time_update = msg->header.stamp;
      Tgi = Tgo - Rggg.transpose() * Rgi * (Tic + Ric * Tco);
      VectorXd odom_x = Eigen::VectorXd::Zero(6);
      odom_x.segment<3>(0) = Tgi;
      ekf.UavOdomUpdate(odom_x, Time_update);
      }
   else
      return;
}

void ugv_odom_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
  //cout << "=== UGV Code found ===" << endl;
  float id = msg->twist.twist.linear.x;
  if(((id-ugvID) > 0.1) || ((id-ugvID) < -0.1))
      return;
  //cout << "UGV updated" << endl;
  Vector3d Tcu,Tgu,Tgi;
  VectorXd mean = ekf.GetState();
  Tgi = mean.segment<3>(0);
  Tcu(0) = msg->pose.pose.position.x;
  Tcu(1) = msg->pose.pose.position.y;
  Tcu(2) = msg->pose.pose.position.z;
  ros::Time Time_update = msg->header.stamp;

  Tgu = Tgi + Rggg_car.transpose() *Rgi * (Tic + Ric * Tcu);
  VectorXd odom_x = VectorXd::Zero(6);
  odom_x.segment<3>(3) = Tgu;
  ekf.UgvOdomUpdate(odom_x, Time_update);


}

void uav_rot_callback(const nav_msgs::Odometry &msg)
{
  Quaterniond ori(msg.pose.pose.orientation.w, msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z);
  Rgi = ori.toRotationMatrix();
  /*
  double phi = msg.x;
  double psi = msg.y;
  double theta = msg.z;
  
  Rgi << cos(psi)*cos(theta)-sin(phi)*sin(psi)*sin(theta), -cos(phi)*sin(psi), cos(psi)*sin(theta)+cos(theta)*sin(phi)*sin(psi), cos(theta)*sin(psi)+cos(psi)*sin(phi)*sin(theta), cos(phi)*cos(psi), sin(psi)*sin(theta)-cos(psi)*cos(theta)*sin(phi), -cos(phi)*sin(theta), sin(phi), cos(phi)*cos(theta);
*/
  
}

void ugv_rot_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
  Quaterniond quater;
  quater.w() = msg->pose.pose.orientation.w;
  quater.x() = msg->pose.pose.orientation.x;
  quater.y() = msg->pose.pose.orientation.y;
  quater.z() = msg->pose.pose.orientation.z;
  Rgc = quater.toRotationMatrix();
}

void target_position_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
  Tgt(0) = msg->pose.pose.position.x;
  Tgt(1) = msg->pose.pose.position.y;
  Tgt(2) = msg->pose.pose.position.z;
}

void initial_angle_callback(const std_msgs::Float32 &msg)
{
  double initial = msg.data;
  Rggg << cos(initial), -sin(initial), 0,
          sin(initial),  cos(initial), 0,
          0           ,             0, 1;
  flag = true;
  //cout << "uav initial: " << initial << endl;
}

void initial_angle_ugv_callback(const geometry_msgs::Vector3 &msg)
{
  if(flag_ugv) return;
  /*
  Quaterniond quater;
  quater.w() = msg->pose.pose.orientation.w;
  quater.x() = msg->pose.pose.orientation.x;
  quater.y() = msg->pose.pose.orientation.y;
  quater.z() = msg->pose.pose.orientation.z;
  flag_ugv = true;
  Rggg_car = quater.toRotationMatrix();*/
  double initial = msg.y;
  Rggg_car << cos(initial), -sin(initial), 0,
              sin(initial),  cos(initial), 0,
              0,                        0, 1;
  flag_ugv = true;
  cout << "ugv initial: " <<  initial << endl;
}

void marker_detector_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
    float target_id = msg->twist.twist.linear.x;
    if(!flag_target[0] && ((target_id - uavID) < 0.1) && ((target_id - uavID) > -0.1)){ 
       
        VectorXd ekfState = ekf.GetState();
        Vector3d Tgi_temp = ekfState.segment<3>(0);
        Vector3d Tct;
        Tct(0) = msg->pose.pose.position.x;
        Tct(1) = msg->pose.pose.position.y;
        Tct(2) = msg->pose.pose.position.z;
        Tgt = Tgi_temp + Rggg.transpose() *Rgi*(Tic + Ric * Tct);
        cout << "Target1 detected:  " << Tgt << endl;
        //cout << "Target local: " << Tct << endl;
        //cout << "UAV pos: " << Tgi_temp << endl; 
	flag_target[0] = true;
        geometry_msgs::Vector3 marker_msg;
        marker_msg.x = Tgt(0);
        marker_msg.y = Tgt(1);
        marker_msg.z = target_id;
        for(int i=0;i<10;i++)
            marker_position_pub.publish(marker_msg);
    }
    else if(!flag_target[1] && ((target_id - uavID2) < 0.1) && ((target_id - uavID2) > -0.1)){
        VectorXd ekfState = ekf.GetState();
        Vector3d Tgi_temp = ekfState.segment<3>(0);
        Vector3d Tct;
        Tct(0) = msg->pose.pose.position.x;
        Tct(1) = msg->pose.pose.position.y;
        Tct(2) = msg->pose.pose.position.z;
        Tgt2 = Tgi_temp + Rggg.transpose() *Rgi*(Tic + Ric * Tct);
        cout << "Target2 detected:  " << Tgt2 << endl;
        //cout << "Target local: " << Tct << endl;
        //cout << "UAV pos: " << Tgi_temp << endl; 
        flag_target[1] = true;
        geometry_msgs::Vector3 marker_msg;
        marker_msg.x = Tgt2(0);
        marker_msg.y = Tgt2(1);
        marker_msg.z = target_id;
        for(int i=0;i<10;i++)
            marker_position_pub.publish(marker_msg);
    }
    else
        return;
    
}
void test_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
  Vector3d tmp;
  tmp(0) = msg->pose.pose.position.x;
  tmp(1) = msg->pose.pose.position.y;
  tmp(2) = msg->pose.pose.position.z;
  tmp = Rggg_car.transpose() * tmp;
  geometry_msgs::Vector3 tmp_geo;
  tmp_geo.x = tmp(0);
  tmp_geo.y = tmp(1);
  tmp_geo.z = tmp(2);
  //test_pub.publish(tmp_geo);
}
int main(int argc, char **argv)
{
  Ric << 0, 1, 0,
      -1, 0, 0,
       0, 0, 1;
  ros::init(argc, argv, "ekf");
  ros::NodeHandle n("~");
  //TODO: reimplement ekf.SetParam
  ros::Subscriber s1 = n.subscribe("/guidance/velocity", 1, uav_vel_callback); //Or /guidance/velocity
  ros::Subscriber s2 = n.subscribe("/n3_sdk/odometry", 1, ugv_vel_callback);
  ros::Subscriber s3 = n.subscribe("/detected_markers", 1, uav_odom_callback);
  ros::Subscriber s4 = n.subscribe("/detected_markers", 1, ugv_odom_callback);
  ros::Subscriber s5 = n.subscribe("/dji_sdk/odometry", 1, uav_rot_callback);
  ros::Subscriber s6 = n.subscribe("/n3_sdk/odometry", 1, ugv_rot_callback);
  //ros::Subscriber s7 = n.subscribe("target_position", 1, target_position_callback);
  ros::Subscriber s8 = n.subscribe("/initial_angle", 1, initial_angle_callback);
  ros::Subscriber s9 = n.subscribe("/n3_sdk/orientation", 1, initial_angle_ugv_callback);
  ros::Subscriber s10 = n.subscribe("/detected_markers", 1, marker_detector_callback);
  ros::Subscriber s11 = n.subscribe("/n3_sdk/odometry", 1, test_callback); 
  odom_ugv_pub = n.advertise<nav_msgs::Odometry>("ekf_odom_ugv", 1); 
  odom_uav_pub = n.advertise<nav_msgs::Odometry>("ekf_odom_uav", 1);
  test_pub = n.advertise<geometry_msgs::Vector3> ("test_pub", 10);
  marker_position_pub = n.advertise<geometry_msgs::Vector3>("/marker_position", 10);
  ros::spin();
}
