/**
   @author Kunio Kojima
*/

#include "Interpolator.h"

using namespace cnoid;

// スプライン補間
void cnoid::setCubicSplineInterpolation(std::vector<Vector3d>& a, const Vector3d& x0, const Vector3d& dx0, const Vector3d& x1, const Vector3d& dx1, const double tau)
{
    while(a.size() < 3) a.push_back(Vector3(0,0,0));

    a[0] = x0;
    a[1] = dx0;
    MatrixXd A(2,3);
    Matrix2d T;
    T <<
        pow(tau, 2), pow(tau, 3),
        2 * tau, 3 * pow(tau, 2);
    MatrixXd P(2,3);
    P.row(0) = x1.transpose() - x0.transpose()  - dx0.transpose() * tau;
    P.row(1) = dx1.transpose() - dx0.transpose();
    A = T.inverse() * P;
    a[2] = A.block(0,0, 1,3).transpose();
    a[3] = A.block(1,0, 1,3).transpose();
}

// void cnoid::minjerkInterpolation(Vector3d& a0, Vector3d& a1, Vector3d& a2, Vector3d& a3, Vector3d& a4, Vector3d& a5,
//                           const Vector3d& x0, const Vector3d& dx0, const Vector3d& ddx0, const Vector3d& x1, const Vector3d& dx1, const Vector3d& ddx1, const double& T)
// {
//     a0 = x0;
//     a1 = dx0;
//     a2 = ddx0/2;

//     const double T2 = pow(T,2), T3 = pow(T,3), T4 = pow(T,4), T5 = pow(T,5);

//     a3 = ddx1/(2*T)  - 3*ddx0/(2*T)  - 4*dx1/T2 - 6*dx0/T2 + 10*x1/T3 - 10*x0/T3;
//     a4 = -ddx1/T2    + 3*ddx0/(2*T2) + 7*dx1/T3 + 8*dx0/T3 - 15*x1/T4 + 15*x0/T4;
//     a5 = ddx1/(2*T3) - ddx0/(2*T3)   - 3*dx1/T4 - 3*dx0/T4 + 6*x1/T5  - 6*x0/T5;
// }
