/**
   @author Kunio Kojima
*/
#include "GeometricUtil.h"

using namespace cnoid;

// bool operator>(const Vector3d& vec1, const Vector3d& vec2) {return vec1.norm() > vec2.norm();}
// bool operator<(const Vector3d& vec1, const Vector3d& vec2) {return vec1.norm() < vec2.norm();}
bool cnoid::compVector(const Vector3d& vec1, const Vector3d& vec2) {return vec1.norm() > vec2.norm();}

Vector3d cnoid::getLongestVector(std::vector<Vector3d> pointVec, const std::vector<Vector3d>& exceptionalPointVec, const double distanceThresh)
{
    // remove neighbor points of exceptionalPointVec
    if(exceptionalPointVec.size() > 0){
        for(auto exceptionalPoint : exceptionalPointVec){
            auto iter = pointVec.begin();
            while(iter != pointVec.end()){
                if(((*iter) - exceptionalPoint).norm() < distanceThresh) iter = pointVec.erase(iter);
                else ++iter;
            }
        }
    }

    // std::sort(pointVec.begin(), pointVec.end());
    // std::sort(pointVec.begin(), pointVec.end(), std::greater<Vector3d>());
    std::sort(pointVec.begin(), pointVec.end(), compVector);

    return pointVec[0];
}

Vector3d cnoid::getFarthestVector(std::vector<Vector3d> pointVec, const Vector3d& referencePoint)
{
    for(auto& point : pointVec) point -= referencePoint;
    return referencePoint + getLongestVector(pointVec);
}

void cnoid::reduceConvexHullToQuadrangle(std::vector<Vector3d>& quadrangleVec, const std::vector<Vector3d>& convexHullVec)
{
    quadrangleVec.push_back(getLongestVector(convexHullVec));
    std::vector<Vector3d> v;
    v.push_back(quadrangleVec[0]);
    quadrangleVec.push_back(getLongestVector(convexHullVec, v));

    quadrangleVec.push_back(getFarthestVector(convexHullVec, quadrangleVec[0]));
    quadrangleVec.push_back(getFarthestVector(convexHullVec, quadrangleVec[1]));
}
