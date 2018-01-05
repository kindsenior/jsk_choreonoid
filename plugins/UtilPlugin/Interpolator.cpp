/**
   @author Kunio Kojima
*/

#include "Interpolator.h"
// #include <iostream>

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

void CubicSplineInterpolator::calcCoefficients(const VectorXd& x0, const VectorXd& dx0, const VectorXd& x1, const VectorXd& dx1, const double duration)
{
    const int numDimensions = x0.size();
    while(coefficientVec.size() < polynominalDegree) coefficientVec.push_back(VectorXd::Zero(numDimensions));

    coefficientVec[0] = x0;
    coefficientVec[1] = dx0;
    MatrixXd A(2,numDimensions);
    Matrix2d T;
    T <<
        pow(duration, 2), pow(duration, 3),
        2 * duration, 3 * pow(duration, 2);
    MatrixXd P(2,numDimensions);
    P.row(0) = x1.transpose() - x0.transpose()  - dx0.transpose() * duration;
    P.row(1) = dx1.transpose() - dx0.transpose();
    A = T.inverse() * P;
    coefficientVec[2] = A.block(0,0, 1,numDimensions).transpose();
    coefficientVec[3] = A.block(1,0, 1,numDimensions).transpose();
}

VectorXd CubicSplineInterpolator::x(double t){
    return coefficientVec[0] + coefficientVec[1] * t + coefficientVec[2] * pow(t,2) + coefficientVec[3] * pow(t,3);
}

VectorXd CubicSplineInterpolator::dx(double t){
    return coefficientVec[1] + 2 * coefficientVec[2] * t + 3 * coefficientVec[3] * pow(t,2);
}

void AccelerationInterpolator::calcCoefficients(const VectorXd& x0, const VectorXd& dx0, const VectorXd& ddx0, const VectorXd& x1, const VectorXd& dx1, const VectorXd& ddx1,
                                                const double duration, const double constPhaseRatio, std::vector<double> phaseRatioVec)
{
    const int numDimensions = x0.size();

    if(phaseRatioVec.size() != numPhases){
        double Tconst = constPhaseRatio*duration, Tvariable = (1-constPhaseRatio*2)/3 * duration;
        phaseRatioVec.resize(numPhases);
        for(int i=0; i<numPhases; ++i) durationVec[i] = i%2 == 0 ? Tvariable : Tconst;
    }else{
        double sumPhaseRatio = 0;
        for(int i=0; i<numPhases; ++i) sumPhaseRatio += phaseRatioVec[i];
        for(int i=0; i<numPhases; ++i) durationVec[i] = phaseRatioVec[i]/sumPhaseRatio*duration;
    }
    {
        double tmp = 0;
        for(int i=0; i<numPhases; ++i){
            phaseInitTimeVec[i] = tmp;
            tmp += durationVec[i];
        }
    }

    std::vector<Matrix2d> Avec(numPhases), Bvec(numPhases);
    for(int i=0; i<numPhases; ++i) Avec[i] << 1,durationVec[i], 0,1;
    Bvec[0] << pow(durationVec[0],2)/6,0, durationVec[0]/2,0;
    Bvec[1] << pow(durationVec[1],2)/2,0, durationVec[1],0;
    Bvec[2] << pow(durationVec[2],2)/3,pow(durationVec[2],2)/6, durationVec[2]/2,durationVec[2]/2;
    Bvec[3] << 0,pow(durationVec[3],2)/2, 0,durationVec[3];
    Bvec[4] << 0,pow(durationVec[4],2)/3, 0,durationVec[4]/2;

    Matrix2d extendedA = Bvec[4] + Avec[4]*Bvec[3] + Avec[4]*Avec[3]*Bvec[2] + Avec[4]*Avec[3]*Avec[2]*Bvec[1] + Avec[4]*Avec[3]*Avec[2]*Avec[1]*Bvec[0];
    MatrixXd C0(2,3),C1(2,3);
    C0.block(0,0, 2,2) = Avec[0];               C0(0,2) = pow(durationVec[0],2)/3; C0(1,2) = durationVec[0]/2;
    C1.block(0,0, 2,2) = -Matrix2d::Identity(); C1(0,2) = pow(durationVec[4],2)/6; C1(1,2) = durationVec[4]/2;

    MatrixXd constAccMat(numDimensions,2);// eg) ((ddx1, ddx3), (ddy1, ddy3), (ddz1, ddz3)) when 3dof(x,y,z)
    MatrixXd initX(3,numDimensions);// eg) ((x0,y0,z0),(dx0,dy0,dz0),(ddx0,ddy0,ddz0))
    initX << x0.transpose(), dx0.transpose(), ddx0.transpose();// vstack of vectors
    MatrixXd terminalX(3,numDimensions);
    terminalX << x1.transpose(), dx1.transpose(), ddx1.transpose();// vstack of vectors
    constAccMat = - (extendedA.inverse() * (Avec[4]*Avec[3]*Avec[2]*Avec[1]*C0*initX + C1*terminalX)).transpose();
    // for(int i=0; i<numPhases; ++i){
    //     // std::cout << "A" << i << ":" << std::endl << Avec[i] << std::endl;
    //     std::cout << "B" << i << ":" << std::endl << Bvec[i] << std::endl;
    // }
    // std::cout << "A4*A3*A2*A1:" << std::endl << Avec[4]*Avec[3]*Avec[2]*Avec[1] << std::endl;
    // std::cout << "(ddz0, ddz1): " << constAccMat.row(2) << std::endl;

    // eg) coefficientMatVec[i] = ((a_0x,a_1x,a_2x,a_3x),(a_0y,a_1y,a_2y,a_3y),(a_0z,a_1z,a_2z,a_3z))
    coefficientMatVec[0] = (MatrixXd(numDimensions,polynominalDegree) << x0,dx0,                          ddx0/2,               (constAccMat.col(0)-ddx0              )/(6*durationVec[0])).finished();
    coefficientMatVec[1] = (MatrixXd(numDimensions,polynominalDegree) << MatrixXd::Zero(numDimensions,2), constAccMat.col(0)/2, MatrixXd::Zero(numDimensions,1)).finished();
    coefficientMatVec[2] = (MatrixXd(numDimensions,polynominalDegree) << MatrixXd::Zero(numDimensions,2), constAccMat.col(0)/2, (constAccMat.col(1)-constAccMat.col(0))/(6*durationVec[2])).finished();
    coefficientMatVec[3] = (MatrixXd(numDimensions,polynominalDegree) << MatrixXd::Zero(numDimensions,2), constAccMat.col(1)/2, MatrixXd::Zero(numDimensions,1)).finished();
    coefficientMatVec[4] = (MatrixXd(numDimensions,polynominalDegree) << MatrixXd::Zero(numDimensions,2), constAccMat.col(1)/2, (ddx1              -constAccMat.col(1))/(6*durationVec[4])).finished();
    coefficientMatVec[1].block(0,0,numDimensions,2) = ( C0*initX + Bvec[0]*constAccMat.transpose() ).transpose();
    for(int i=2; i<numPhases; ++i){
        coefficientMatVec[i].block(0,0,numDimensions,2) = ( Avec[i-1]*coefficientMatVec[i-1].block(0,0,numDimensions,2).transpose() + Bvec[i-1]*constAccMat.transpose() ).transpose();
    }
    // for(int i=0; i<numPhases; ++i){
    //     std::cout << "a0,a1,a2,a3 " << phaseInitTimeVec[i] << "->" << phaseInitTimeVec[i]+durationVec[i] << " : " << coefficientMatVec[i].row(2) << std::endl;
    // }

}

VectorXd AccelerationInterpolator::x(double t){
    for(int i=0; i<numPhases; ++i){
        if(t > phaseInitTimeVec[i]+durationVec[i]) continue;
        MatrixXd coefficientMat = coefficientMatVec[i];
        double T = t - phaseInitTimeVec[i];
        return coefficientMat.col(0) + coefficientMat.col(1)*T + coefficientMat.col(2)*pow(T,2) + coefficientMat.col(3)*pow(T,3);
    }
}

VectorXd AccelerationInterpolator::dx(double t){
    for(int i=0; i<numPhases; ++i){
        if(t > phaseInitTimeVec[i]+durationVec[i]) continue;
        MatrixXd coefficientMat = coefficientMatVec[i];
        double T = t - phaseInitTimeVec[i];
        return coefficientMat.col(1) + 2*coefficientMat.col(2)*T + 3*coefficientMat.col(3)*pow(T,2);
    }
}
