/**
   @author Kunio Kojima
*/
#pragma once

#include <iostream>
#include <vector>

#include <boost/math/special_functions/sign.hpp>

#include <hrpUtil/EigenTypes.h>

namespace hrp {

class ContactConstraintParam
{
protected:
    int inputForceDim;

public:
    int inputDim;
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

    dmatrix inputForceConvertMat;
    dmatrix inputWeightConvertMat;

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
protected:
    void initialize()
    {
        inputForceDim = 6;// 6軸力
        inputDim = inputForceDim;// 6軸力
        numEquals = 0;
        numInequals = edgeVec.size();
        inputForceConvertMat = dmatrix::Identity(inputForceDim,inputDim);
        inputWeightConvertMat = dmatrix::Identity(inputForceDim,inputDim);
    }

public:
    virtual void calcInequalMatrix();
    virtual void calcInequalMinimumVector();
    virtual void calcInequalMaximumVector();
    virtual void calcMinimumVector();
    virtual void calcMaximumVector();

    SimpleContactConstraintParam(const std::string linkName_, const std::vector<Vector3>& edgeVec_)
        : ContactConstraintParam(linkName_)
    {
        initialize();
    };
};

class StaticContactConstraintParam : public SimpleContactConstraintParam
{
public:
    double muTrans, muRot;

    void calcInequalMatrix();

    StaticContactConstraintParam(const std::string linkName_, const std::vector<Vector3>& edgeVec_, const double mu_)
        : SimpleContactConstraintParam(linkName_, edgeVec_)
    {
        muTrans = mu_;
        numInequals = 4 + edgeVec.size();// 静止摩擦制約4式
    };
};

class SlideContactConstraintParam : public SimpleContactConstraintParam
{
protected:
    void initialize()
    {
        inputForceDim = 6;// 6軸力
        inputDim = inputForceDim;// 6軸力
        numEquals = 0;
        numInequals = edgeVec.size();
        inputForceConvertMat = dmatrix::Identity(inputForceDim,inputDim);
        inputWeightConvertMat = dmatrix::Identity(inputForceDim,inputDim);
    }

public:
    double muTrans, muRot;
    Vector3 direction;

    void calcInequalMatrix();
    void calcInequalMaximumVector();

    SlideContactConstraintParam(const std::string linkName_, const std::vector<Vector3>& edgeVec_, const double mu_, const Vector3& direction_)
        : SimpleContactConstraintParam(linkName_, edgeVec_)
    {
        muTrans = mu_;
        direction = direction_;
        numInequals = 2 + edgeVec.size();// 動摩擦制約2式
    };
};

}
