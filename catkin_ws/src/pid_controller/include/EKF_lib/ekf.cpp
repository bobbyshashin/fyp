#include "ekf.h"

EKF::EKF(int _n, int _m, float _q, float _r)
{
    init_filter( _n, _m);

    setQ(_q);
    setR(_r);
}

void EKF::init_filter(int _n, int _m)
{
    n = _n;
    m = _m;

    x = VectorXd::Zero(n+1+3);

    error_in = VectorXd::Zero(m);
    error_out = VectorXd::Zero(n);

    P_mat = MatrixXd::Identity(n,n);

    F_mat = MatrixXd::Identity(n,n);
    Phi_mat = MatrixXd::Identity(n,n);
    A_mat = MatrixXd::Zero(n,n);
    //TODO: G and Q maybe not such size
    Q_mat = MatrixXd::Identity(n,n);
    G_mat = MatrixXd::Identity(n,n);

    H_mat = MatrixXd::Zero(m,n);
    R_mat = MatrixXd::Identity(m,m);
    Kg = MatrixXd::Zero(n,m);

    is_init = false;
}

void EKF::setQ(MatrixXd _Q)
{
    Q_mat = _Q;
}

void EKF::setR(MatrixXd _R)
{
    R_mat = _R;
}

void EKF::setQ(float _q)
{
    Q_mat = MatrixXd::Identity(n,n);

    Q_mat = Q_mat* _q;
}

void EKF::setR(float _r)
{
    R_mat = MatrixXd::Identity(m,m);

    R_mat = R_mat* _r;
}

const Quaterniond EKF::get_orientation_q_vb()
{
    return Quaterniond( x.segment<4>(0) );
}

const Vector3d EKF::get_position_v()
{
    return Vector3d( x.segment<3>(0+4) );
}

const Vector3d EKF::get_bias_v()
{
    return Vector3d( x.segment<3>(0+4+3) );
}

const Vector3d EKF::get_velocity_v()
{
    return Vector3d( x.segment<3>(0+4+3+3) );
}

void EKF::init_ekf_state(Quaterniond q_vb_init, Vector3d p_v_init)
{
    x.segment<4>(0) = q_vb_init.coeffs();
    x.segment<3>(0+4) = p_v_init;
    x.segment<3>(0+4+3) =  Vector3d::Zero();
    x.segment<3>(0+4+3+3) =  Vector3d::Zero();

    is_init = true;
}

void EKF::predict(const Quaterniond q_eb, const Vector3d w_b, const Vector3d v_e, const float dt)
{
    if(is_init == true)
    {
        Matrix3d R_eb = q_eb.toRotationMatrix();

        Vector3d v_v_post = this->get_velocity_v();
        Vector3d v_v = R_ve*(v_e - R_eb* this->get_bias_v());

        Vector3d v_v_calc = 0.5f*( v_v + v_v_post);

        //TODO: here may be not vary true???
        // q_vb
        x.segment<4>(0) = (q_ve*q_eb).coeffs();
        // p_v
        x.segment<3>(0+4) += v_v_calc* dt;
        // bias_v_b
        x.segment<3>(0+4+3) = this->get_bias_v();
        // v_v
        x.segment<3>(0+4+3+3) = v_v;


        Matrix3d R_vb = get_orientation_q_vb().toRotationMatrix();
        F_mat<<
                             -crossMat(w_b), Matrix3d::Zero(), Matrix3d::Zero(),
                R_vb*crossMat(get_bias_v()), Matrix3d::Zero(),       -R_ve*R_eb,
                           Matrix3d::Zero(), Matrix3d::Zero(), Matrix3d::Zero();

        G_mat<<
                -Matrix3d::Identity(), Matrix3d::Zero(),     Matrix3d::Zero(),
                     Matrix3d::Zero(),            -R_ve,           -R_ve*R_eb,
                     Matrix3d::Zero(), Matrix3d::Zero(), Matrix3d::Identity();

        Phi_mat = MatrixXd::Identity(n,n) + F_mat*dt;

        //TODO: for nonlinear model, the predict update maybe not above
        P_mat = Phi_mat * P_mat * Phi_mat.transpose() + G_mat*Q_mat*G_mat.transpose()*dt;
    }
}

void EKF::measrue_update(const Quaterniond q_vb_meas, const Vector3d p_v_meas)
{
    if(is_init == true)
    {

        // orietation error
        Quaterniond q_bv_meas = q_vb_meas.conjugate();
        Quaterniond delta_q = q_bv_meas * this->get_orientation_q_vb();

        error_in.segment<3>(0) = -delta_q.vec();// /delta_q.w();
        error_in.segment<3>(0+3) = p_v_meas - this->get_position_v();

        H_mat<<
                Matrix3d::Identity(),     Matrix3d::Zero(), Matrix3d::Zero(),
                    Matrix3d::Zero(), Matrix3d::Identity(), Matrix3d::Zero();

        Kg = P_mat * H_mat.transpose() * (H_mat * P_mat * H_mat.transpose() + R_mat).inverse();

        error_out =  Kg * error_in;

        P_mat = (MatrixXd::Identity(Kg.rows(),H_mat.cols()) - Kg * H_mat) *P_mat;
    }
}

void EKF::correct()
{
    Vector3d delta_theta(  error_out.segment<3>(0) );
    Vector3d delta_p_v(    error_out.segment<3>(0+3) );
    Vector3d delta_bias_v( error_out.segment<3>(0+3+3) );

    Quaterniond q_vb = this->get_orientation_q_vb();
    quaternion_correct(q_vb, delta_theta);

    x.segment<4>(0) = q_vb.coeffs();
    x.segment<3>(0+4) += delta_p_v;
    x.segment<3>(0+4+3) += delta_bias_v;

}
