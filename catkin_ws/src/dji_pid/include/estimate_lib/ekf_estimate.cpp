#include "ekf_estimate.h"
#include "math_uilts.h"

using namespace estimate;

void estimate::F_func(MatrixXd& F,MatrixXd& G,VectorXd& state, const VectorXd& imu,const float dt)
{
    // read in imu data
    Quaterniond q_eb( imu.segment<4>(0));
    Vector3d w_b(     imu.segment<3>(0+4));
    Vector3d v_e(     imu.segment<3>(0+4+3));
    Quaterniond q_ve( imu.segment<4>(0+4+3+3));

    // propagation
    Matrix3d R_eb = q_eb.toRotationMatrix();
    Matrix3d R_ve = q_ve.toRotationMatrix();

    Vector3d v_v_post = state.segment<3>(0+4+3+3);
    Vector3d v_v = R_ve*(v_e - R_eb* Vector3d(state.segment<3>(0+4+3)));
    Vector3d v_v_calc = 0.5f*( v_v + v_v_post);


    // update Jacobian Matrix
    Matrix3d R_vb = Quaterniond(state.segment<4>(0)).toRotationMatrix();
    F<<
                                       -crossMat(w_b), Matrix3d::Zero(), Matrix3d::Zero(),
     R_vb*crossMat(Vector3d(state.segment<3>(0+4+3))), Matrix3d::Zero(),       -R_ve*R_eb,
                                     Matrix3d::Zero(), Matrix3d::Zero(), Matrix3d::Zero();

    G<<
        -Matrix3d::Identity(), Matrix3d::Zero(),     Matrix3d::Zero(),
             Matrix3d::Zero(),            -R_ve,           -R_ve*R_eb,
             Matrix3d::Zero(), Matrix3d::Zero(), Matrix3d::Identity();

    // update states
    // q_vb
    state.segment<4>(0) = (q_ve*q_eb).coeffs();
    // p_v
    state.segment<3>(0+4) += v_v_calc* dt;
    // bias_v_b
    state.segment<3>(0+4+3) = state.segment<3>(0+4+3);
    // v_v
    state.segment<3>(0+4+3+3) = v_v;

}

void estimate::H_func( MatrixXd& H,VectorXd& error,const VectorXd& state, const VectorXd& vicon)
{
    //read in vicon data
    Quaterniond q_vb_meas( vicon.segment<4>(0));
    Vector3d p_v_meas(     vicon.segment<3>(0+4));

    // orietation error
    Quaterniond q_bv_meas = q_vb_meas.conjugate();
    Quaterniond delta_q = q_bv_meas * Quaterniond(state.segment<4>(0));

    // calculate error-state
    error.segment<3>(0) = -delta_q.vec();// /delta_q.w();
    error.segment<3>(0+3) = p_v_meas - state.segment<3>(0+4);

    // update Jacobian Matrix
    H<<
            Matrix3d::Identity(),     Matrix3d::Zero(), Matrix3d::Zero(),
                Matrix3d::Zero(), Matrix3d::Identity(), Matrix3d::Zero();

}

void estimate::C_func(const VectorXd& error, VectorXd& state)
{
    // get delta state
    Vector3d delta_theta(  error.segment<3>(0) );
    Vector3d delta_p_v(    error.segment<3>(0+3) );
    Vector3d delta_bias_v( error.segment<3>(0+3+3) );

    Quaterniond q_vb = Quaterniond(state.segment<4>(0));
    quaternion_correct(q_vb, delta_theta);

    // state correct
    state.segment<4>(0) = q_vb.coeffs();
    state.segment<3>(0+4) += delta_p_v;
    state.segment<3>(0+4+3) += delta_bias_v;

}



/********************************************************************************/

ekf_estimate::ekf_estimate()
{
    x = VectorXd::Zero(13);

    init_estimate();
}

ekf_estimate::~ekf_estimate()
{

}

void ekf_estimate::init_estimate()
{
    ekf = new EKF(9,6,
                  &estimate::F_func,&estimate::H_func,&estimate::C_func,
                  0.1,0.1);
    cout<<"init done."<<endl;
}

bool ekf_estimate::init_state(const VectorXd vicon_meas)
{
    x.segment<4>(0) = vicon_meas.segment<4>(0);
    x.segment<3>(0+4) = vicon_meas.segment<3>(0+4);
    x.segment<3>(0+4+3) =  Vector3d::Zero();
    x.segment<3>(0+4+3+3) =  Vector3d::Zero();

    return true;
}

void ekf_estimate::readin_vicon(const VectorXd vicon_meas)
{
    ekf->update(x,vicon_meas);

    ekf->correct(x);
}

void ekf_estimate::readin_imu(const VectorXd imu_meas,const float dt)
{
    ekf->predict(x,imu_meas,dt);

}

const Quaterniond ekf_estimate::get_orientation_q_vb()
{
    return Quaterniond( x.segment<4>(0) );
}

const Vector3d ekf_estimate::get_position_v()
{
    return Vector3d( x.segment<3>(0+4) );
}

const Vector3d ekf_estimate::get_bias_v()
{
    return Vector3d( x.segment<3>(0+4+3) );
}

const Vector3d ekf_estimate::get_velocity_v()
{
    return Vector3d( x.segment<3>(0+4+3+3) );
}


