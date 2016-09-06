#ifndef _ESTI_H
#define _ESTI_H

#include <iostream>
#include <stdio.h>

#include <Eigen/Dense>

#include "ekf.h"

using namespace std;
using namespace Eigen;

namespace estimate
{

void F_func(MatrixXd& F,MatrixXd& G,VectorXd& state, const VectorXd& imu,const float dt);
void H_func(MatrixXd& H,VectorXd& error,const VectorXd& state, const VectorXd& vicon);
void C_func(const VectorXd& error,VectorXd& state);

class ekf_estimate
{
private:
    // q_vb | p_v | bias_v_b | v_v |
    VectorXd x;
    void init_estimate();

public:
    ekf_estimate();
    ~ekf_estimate();

    EKF* ekf;

    bool init_state(const VectorXd vicon_meas);
    void readin_vicon(const VectorXd vicon_meas);
    void readin_imu(const VectorXd imu_meas, const float dt);
    void correct();

    const Quaterniond get_orientation_q_vb();
    const Vector3d get_bias_v();
    const Vector3d get_velocity_v();
    const Vector3d get_position_v();

    bool is_init;

    VectorXd imu_meas;
    VectorXd vicon_meas;

};

}
#endif
