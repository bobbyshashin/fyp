#include "ekf.h"

EKF::EKF()
{
	init = false;

	gravity = 9.7;
  	g << 0,
  	     0,
  	     gravity;

  	I6 = MatrixXd::Identity(6,6);
		I3 = MatrixXd::Identity(3,3);
	//I15 = MatrixXd::Identity(15,15);
}

EKF::~EKF() { }

bool EKF::isInit()
{
		return init;
}

VectorXd EKF::GetState()
{
	return StateHist[StateHist.size()-1].mean;
}

ros::Time EKF::GetTime()
{
	return StateHist[StateHist.size()-1].stamp;
}

void EKF::SetParam(double var_n_uav, double var_n_ugv)
{
	Q.setZero(6, 6);
	R.setZero(6, 6);
	W = I6;
	C_uav.setZero(6,6);
	C_ugv.setZero(6,6);
	C_uav.block(0,0,3,3) = I3;
	C_ugv.block(3,3,3,3) = I3;
	A.setZero(6, 6);
        F.setZero(6, 6);
        U.setZero(6, 6);
	V.setZero(6, 6);

  for(int i=0;i<3; i++){
      R(i,i) = var_n_uav;}
  for(int i=3;i<6;i++){
      R(i,i) = var_n_ugv;}
}

void EKF::SetInit(VectorXd Z, ros::Time stamp)
{
  StateHist.clear();
  VectorXd mean_init = Eigen::VectorXd::Zero(6);
  MatrixXd var_init  = Eigen::MatrixXd::Zero(6,6);

  //mean_init.segment<6>(0) = Z;
  State state_init;
  state_init.mean = mean_init;
  state_init.var  = var_init;
  state_init.stamp = stamp;
  state_init.u_uav = VectorXd::Zero(3); 
  state_init.u_ugv = VectorXd::Zero(3);
  StateHist.push_back(state_init);
  init = true;

}

void EKF::UavPropagation(VectorXd u, ros::Time stamp, Matrix3d R_a_g, Matrix3d R_c_g)
{
  State state_last = StateHist[StateHist.size()-1];
  VectorXd mean_last = state_last.mean;
  MatrixXd var_last = state_last.var;
  
  A = MatrixXd::Zero(6,6);
  U.block(0,0,3,3) = -R_a_g; //TODO: Need to be defined later, subscribe from SDK
  U.block(3,3,3,3) = -R_c_g; //TODO: Same with above

  double dT = (stamp - state_last.stamp).toSec();
  VectorXd xdot = VectorXd::Zero(6);
  xdot.segment<3>(0) = u;
  xdot.segment<3>(3) = state_last.u_ugv;

  F = I6;
  V = dT * U;
  State state_new;
  state_new.mean = mean_last +dT * (xdot);
  state_new.var = F * var_last * F.transpose() + V * Q * V.transpose();
  state_new.stamp = stamp;
  state_new.u_uav = u;
  state_new.u_ugv = state_last.u_ugv;
  StateHist.push_back(state_new);
}

void EKF::UgvPropagation(VectorXd u, ros::Time stamp, Matrix3d R_a_g, Matrix3d R_c_g)
{
  State state_last = StateHist[StateHist.size()-1];
  VectorXd mean_last = state_last.mean;
  MatrixXd var_last = state_last.var;

  A = MatrixXd::Zero(6,6);
  U.block(0,0,3,3) = -R_a_g; //TODO: Need to be defined later, subscribe from SDK
  U.block(3,3,3,3) = -R_c_g; //TODO: Same with above
  //TODO: Notice question here: If we update seperatedly, do we need to keep U unchanged partly?
  double dT = (stamp - state_last.stamp).toSec();
  VectorXd xdot = Eigen::VectorXd::Zero(6);
  xdot.segment<3>(3) = u;
  xdot.segment<3>(0) = state_last.u_uav;

  F = I6;
  V = dT * U;
  State state_new;
  state_new.mean = mean_last +dT * xdot;
  state_new.var = F * var_last * F.transpose() + V * Q * V.transpose();
  state_new.stamp = stamp;
  state_new.u_ugv = u;
  state_new.u_ugv = state_last.u_uav;
  StateHist.push_back(state_new);
}

void EKF::UavOdomUpdate(VectorXd z, ros::Time time)
{
  State state_last;
  vector< State >::iterator itr;
  vector< State >::iterator itr2;
  vector< State > state3;

  state_last = StateHist[StateHist.size()-1];
  /*
  for(itr = StateHist.begin(); itr != StateHist.end(); itr ++){
      if(itr->stamp > stamp || itr->stamp == stamp) {
          state_last = *(itr-1);
          state3.assign(itr,StateHist.end());
          break;
      }
  }
  */
  StateHist.clear();
  Eigen::MatrixXd K = Eigen::MatrixXd::Zero(6,6);
  Eigen::MatrixXd K1 = Eigen::MatrixXd::Zero(6,6);
  Eigen::MatrixXd K2 = Eigen::MatrixXd::Zero(6,6);
  Eigen::MatrixXd K3 = Eigen::MatrixXd::Zero(6,6);

  K1 = state_last.var * (C_uav.transpose());
  K2 = C_uav*state_last.var*(C_uav.transpose())+W*R*(W.transpose()) ;
  K3 = K2.inverse();
  K = K1*K3;

  State state_new;
  state_new.mean = state_last.mean + K*(z-C_uav*state_last.mean);
  state_new.var  = state_last.var - K*C_uav*state_last.var;
  state_new.stamp = state_last.stamp;

  StateHist.push_back(state_new);
  /*
  for(itr2 = state3.begin(); itr2 != state3.end(); itr2 ++){
      VectorXd u_again = itr2->u;
      ros::Time stamp_again = itr2->stamp;
      EKF::ImuPropagation(u_again, stamp_again);
  
  }*/
}
void EKF::UgvOdomUpdate(VectorXd z, ros::Time time)
{
	State state_last;
  vector< State >::iterator itr;
  vector< State >::iterator itr2;
  vector< State > state3;

  state_last = StateHist[StateHist.size()-1];
	StateHist.clear();
  Eigen::MatrixXd K = Eigen::MatrixXd::Zero(6,6);
  Eigen::MatrixXd K1 = Eigen::MatrixXd::Zero(6,6);
  Eigen::MatrixXd K2 = Eigen::MatrixXd::Zero(6,6);
  Eigen::MatrixXd K3 = Eigen::MatrixXd::Zero(6,6);

  K1 = state_last.var * (C_ugv.transpose());
  K2 = C_ugv*state_last.var*(C_ugv.transpose())+W*R*(W.transpose()) ;
  K3 = K2.inverse();
  K = K1*K3;

  State state_new;
  state_new.mean = state_last.mean + K*(z-C_ugv*state_last.mean);
  state_new.var  = state_last.var - K*C_ugv*state_last.var;
  state_new.stamp = state_last.stamp;

  StateHist.push_back(state_new);
}
