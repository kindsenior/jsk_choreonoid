/**
   @author Kunio Kojima
*/
#pragma once

#include <iostream>
#include <vector>

#include <hrpUtil/EigenTypes.h>

namespace hrp {

class ContactConstraintParam
{
public:
    int contactState;
    int numEquals;
    int numInequals;
    std::string linkName;
    Vector3 p;
    Matrix33 R;

    dmatrix inequalMat;

    std::vector<Vector3> edgeVec;
    double mu;

    virtual void calcInequalMatrix()=0;

    ContactConstraintParam(const std::string linkName_)
    {
        linkName = linkName_;
    };
};

class StaticContactConstraintParam : public ContactConstraintParam
{
public:
    // std::vector<Vector3> edgeVec;
    // double mu;

    void calcInequalMatrix();

    StaticContactConstraintParam(const std::string linkName_, const double mu_, const std::vector<Vector3>& edgeVec_)
        : ContactConstraintParam(linkName_)
    {
        numEquals = 0;
        mu = mu_;
        edgeVec = edgeVec_;
        numInequals = 4 + edgeVec.size();// 静止摩擦制約4式
    };
};

}
