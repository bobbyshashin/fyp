#include "math_uilts.h"

Matrix3d crossMat(const Vector3d& v)
{
    Matrix3d m;
    m <<     0, -v(2),  v(1),
          v(2),     0, -v(0),
         -v(1),  v(0),     0;
    return m;
}

//
// Omega( v ) = -[v] v
//               -v  0
//
Matrix4d Omega(const Vector3d& v)
{
    Matrix4d m;
    m <<     0,  v(2), -v(1),  v(0),
         -v(2),     0,  v(0),  v(1),
          v(1), -v(0),     0,  v(2),
         -v(0), -v(1), -v(2),     0;
    return m;
}

Quaterniond quaternion_correct(Quaterniond q, Vector3d d_theta)
{
    Quaterniond dq(
                   1,
                   0.5f* d_theta(0),
                   0.5f* d_theta(1),
                   0.5f* d_theta(2)
                   );
    dq.w() = 1 - dq.vec().transpose() * dq.vec();
    q = (q * dq).normalized();
    return q;
}
