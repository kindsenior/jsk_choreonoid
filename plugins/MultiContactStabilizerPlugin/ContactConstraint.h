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

class SimpleContactConstraintParam : public ContactConstraintParam
{
public:
    std::vector<Vector3> edgeVec;

    virtual void calcInequalMatrix();
    virtual void calcInequalMinimumVector();
    virtual void calcInequalMaximumVector();
    virtual void calcMinimumVector();
    virtual void calcMaximumVector();

    SimpleContactConstraintParam(const std::string linkName_, const std::vector<Vector3>& edgeVec_)
        : ContactConstraintParam(linkName_)
    {
        unitInputDim = 6;
        numEquals = 0;
        edgeVec = edgeVec_;
        numInequals = edgeVec.size();// 静止摩擦制約4式
    };
};

class StaticContactConstraintParam : public SimpleContactConstraintParam
{
public:
    double muTrans, muRot;

    void calcInequalMatrix();

    StaticContactConstraintParam(const std::string linkName_, const double mu_, const std::vector<Vector3>& edgeVec_)
        : SimpleContactConstraintParam(linkName_, edgeVec_)
    {
        muTrans = mu_;
        numInequals = 4 + edgeVec.size();// 静止摩擦制約4式
    };
};

}
