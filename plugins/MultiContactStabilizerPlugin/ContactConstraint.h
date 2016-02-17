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
protected:
    int unitInputDim;

public:
    int contactState;
    int numEquals;
    int numInequals;
    std::string linkName;
    Vector3 p;
    Matrix33 R;

    dmatrix equalMat;
    dvector equalVec;
    dmatrix inequalMat;
    dvector inequalMinVec;
    dvector inequalMaxVec;
    dvector minVec;
    dvector maxVec;

    std::vector<Vector3> edgeVec;
    double mu;

    virtual void calcEqualMatrix(){equalMat.resize(0,0);};
    virtual void calcEqualVector(){equalVec.resize(0);};
    virtual void calcInequalMatrix(){inequalMat.resize(0,0);};
    virtual void calcInequalMinimumVector(){inequalMinVec.resize(0);};
    virtual void calcInequalMaximumVector(){inequalMaxVec.resize(0);}
    virtual void calcMinimumVector(){minVec.resize(0);};
    virtual void calcMaximumVector(){maxVec.resize(0);};

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
    void calcInequalMinimumVector();
    void calcInequalMaximumVector();
    void calcMinimumVector();
    void calcMaximumVector();

    StaticContactConstraintParam(const std::string linkName_, const double mu_, const std::vector<Vector3>& edgeVec_)
        : ContactConstraintParam(linkName_)
    {
        unitInputDim = 6;
        numEquals = 0;
        mu = mu_;
        edgeVec = edgeVec_;
        numInequals = 4 + edgeVec.size();// 静止摩擦制約4式
    };
};

}
