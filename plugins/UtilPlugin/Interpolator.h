/**
   @author Kunio Kojima
*/
#pragma once

#include <vector>
#include <cnoid/src/Util/EigenTypes.h>

namespace cnoid {

void setCubicSplineInterpolation(std::vector<Vector3d>& a, const Vector3d& x0, const Vector3d& dx0, const Vector3d& x1, const Vector3d& dx1, const double tau);

}
