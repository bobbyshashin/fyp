#ifndef _UTILS_H_
#define _UTILS_H_

#include <Eigen/Dense>
//#include </usr/local/include/eigen3/Eigen/Dense>

using namespace Eigen;

Matrix3d crossMat(const Vector3d& v);
Matrix4d Omega(const Vector3d& v);
Quaterniond quaternion_correct(Quaterniond q, Vector3d d_theta);

#endif
