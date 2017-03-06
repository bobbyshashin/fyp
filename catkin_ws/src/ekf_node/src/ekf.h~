#ifndef EKF_H
#define EKF_H

#include <iostream>
#include <ros/ros.h>
#include <ros/console.h>
#include <Eigen/Eigen>
#include <fstream>
#include <math.h>


#define PI M_PI

using namespace std;
using namespace Eigen;
struct State
{
		ros::Time stamp;
		VectorXd  u_uav;
		VectorXd  u_ugv;
		VectorXd  mean;
		MatrixXd  var;
};
class EKF
{

	private:
		vector<State> StateHist;

		bool init;
		double gravity;
		Vector3d g;
		MatrixXd Q, R, W, C_uav, C_ugv, I6, I3;//, I15;
		MatrixXd A, F, U, V;

 	public:
		EKF();
		~EKF();

		bool isInit();
		VectorXd GetState();
		ros::Time GetTime();
		void SetParam(double var_n_uav, double var_n_ugv);
		void SetInit(VectorXd Z, ros::Time stamp);
		void UavPropagation(VectorXd u, ros::Time stamp, Matrix3d R_a_g, Matrix3d R_c_g);
		void UgvPropagation(VectorXd u, ros::Time stamp, Matrix3d R_a_g, Matrix3d R_c_g);
		void UavOdomUpdate(VectorXd z, ros::Time time);
    void UgvOdomUpdate(VectorXd z, ros::Time time);

};

#endif
