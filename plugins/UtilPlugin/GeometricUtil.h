/**
   @author Kunio Kojima
*/
#pragma once

#include <vector>

#include <cnoid/src/Util/EigenTypes.h>

namespace cnoid{

bool compVector(const Vector3d& vec1, const Vector3d& vec2);

Vector3d getLongestVector(std::vector<Vector3d> pointVec, const std::vector<Vector3d>& exceptionalPointVec = std::vector<Vector3d>(), const double distanceThresh = 0.05);

Vector3d getFarthestVector(std::vector<Vector3d> pointVec, const Vector3d& referencePoint);

void reduceConvexHullToQuadrangle(std::vector<Vector3d>& quadrangleVec, const std::vector<Vector3d>& convexHullVec );

}
