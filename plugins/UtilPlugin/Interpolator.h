/**
   @author Kunio Kojima
*/
#pragma once

#include <vector>
#include <cnoid/src/Util/EigenTypes.h>

namespace cnoid {

void setCubicSplineInterpolation(std::vector<Vector3d>& a, const Vector3d& x0, const Vector3d& dx0, const Vector3d& x1, const Vector3d& dx1, const double tau);

class Interpolator
{
protected:
    std::vector<VectorXd> coefficientVec;

public:
    Interpolator(){}
};

class CubicSplineInterpolator : public Interpolator
{
protected:
    int polynominalDegree;

public:
    CubicSplineInterpolator() : Interpolator(){
        polynominalDegree = 4;
    }

    void calcCoefficients(const VectorXd& x0, const VectorXd& dx0, const VectorXd& x1, const VectorXd& dx1, const double duration);

    VectorXd x(double t);
    VectorXd dx(double t);
};

class AccelerationInterpolator : public Interpolator
{
protected:
    int numPhases;
    int polynominalDegree;
    std::vector<MatrixXd> coefficientMatVec;
    std::vector<double> durationVec;
    std::vector<double> phaseInitTimeVec;

public:

    AccelerationInterpolator() : Interpolator(){
        numPhases = 5;
        polynominalDegree = 4;
        coefficientMatVec.resize(numPhases);
        durationVec.resize(numPhases);
        phaseInitTimeVec.resize(numPhases);
    }

    void calcCoefficients(const VectorXd& x0, const VectorXd& dx0, const VectorXd& ddx0, const VectorXd& x1, const VectorXd& dx1, const VectorXd& ddx1,
                          const double duration, const double constPhaseRatio=0.2, const std::vector<double> phaseRatioVec={});

    VectorXd x(double t);
    VectorXd dx(double t);
};

}
