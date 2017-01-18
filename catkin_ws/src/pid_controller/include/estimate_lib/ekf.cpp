#include "ekf.h"

EKF::EKF(int _n, int _m,
         void (*_f_func)(MatrixXd&,MatrixXd&,VectorXd&,const VectorXd&, const float dt),
         void (*_h_func)(MatrixXd&,VectorXd&,const VectorXd&, const VectorXd&),
         void (*_c_func)(const VectorXd &, VectorXd &),
         float _q, float _r)
{
    init_filter( _n, _m);

    f_func = _f_func;
    h_func = _h_func;
    c_func = _c_func;

    setQ(_q);
    setR(_r);
}

void EKF::init_filter(int _n, int _m)
{
    n = _n;
    m = _m;

    error_in = VectorXd::Zero(m);
    error_out = VectorXd::Zero(n);

    P_mat = MatrixXd::Identity(n,n);

    F_mat = MatrixXd::Identity(n,n);
    Phi_mat = MatrixXd::Identity(n,n);
    //TODO: G and Q maybe not such size
    Q_mat = MatrixXd::Identity(n,n);
    G_mat = MatrixXd::Identity(n,n);

    H_mat = MatrixXd::Zero(m,n);
    R_mat = MatrixXd::Identity(m,m);
    Kg = MatrixXd::Zero(n,m);

}

EKF::~EKF(){}


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

void EKF::predict(VectorXd& state,const VectorXd& imu, const float dt)
{
    f_func(F_mat, G_mat, state, imu, dt);

    Phi_mat = MatrixXd::Identity(n,n) + F_mat*dt;

    //TODO: for nonlinear model, the predict update maybe not above
    P_mat = Phi_mat * P_mat * Phi_mat.transpose() + G_mat*Q_mat*G_mat.transpose()*dt;

}

void EKF::update(const VectorXd& state,const VectorXd& vicon)
{
    h_func(H_mat, error_in, state, vicon);

    Kg = P_mat * H_mat.transpose() * (H_mat * P_mat * H_mat.transpose() + R_mat).inverse();

    error_out =  Kg * error_in;

    P_mat = (MatrixXd::Identity(Kg.rows(),H_mat.cols()) - Kg * H_mat) *P_mat;
}

void EKF::correct(VectorXd& state)
{
    c_func(error_out,state);
}
