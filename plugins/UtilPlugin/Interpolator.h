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

public:
    std::vector<VectorXd> coefficientVec;

    Interpolator(){}
};

class CubicSplineInterpolator : public Interpolator
{
public:
    int polynominalDegree;

    CubicSplineInterpolator() : Interpolator(){
        polynominalDegree = 4;
    }

    void calcCoefficients(const VectorXd& x0, const VectorXd& dx0, const VectorXd& x1, const VectorXd& dx1, const double duration);

    VectorXd x(double t);
    VectorXd dx(double t);
};

}
