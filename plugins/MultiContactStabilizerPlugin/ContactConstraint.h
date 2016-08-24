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
    void calcEdgeFromVertex()
    {
        edgeVec.clear();
        for(std::vector<Vector2>::iterator iter0 = vertexVec.begin(); iter0 != --vertexVec.end(); ++iter0){
            std::vector<Vector2>::iterator iter1 = iter0;
            for(++iter1; iter1 != vertexVec.end(); ++iter1){
                double a = -((*iter0)[1] - (*iter1)[1]);// -ydiff
                double b = (*iter0)[0] - (*iter1)[0];// xdiff
                double c = -(*iter0)[0]*a - (*iter0)[1]*b;
                bool pluseFlg = true, minusFlg = true;
                for(std::vector<Vector2>::iterator iter2 = vertexVec.begin(); iter2 != vertexVec.end(); ++iter2){
                    if(*iter2 == *iter0 || *iter2 == *iter1) continue;
                    pluseFlg &= (*iter2)[0]*a + (*iter2)[1]*b + c > 0;
                    minusFlg &= (*iter2)[0]*a + (*iter2)[1]*b + c < 0;
                }
                if(pluseFlg || minusFlg){
                    Vector3 edge;
                    if(c > 0){
                        edge << a,b,c;
                    }else{
                        edge << -a,-b,-c;
                    }
                    edgeVec.push_back(edge);
                }
            }
        }
    }

public:
    int inputDim;
    int numEquals;
    int numInequals;
    std::string linkName;
    Vector3 p;
    Matrix33 R;
    std::vector<Vector3> edgeVec;
    std::vector<Vector2> vertexVec;

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

    ContactConstraintParam(const std::string linkName_, const std::vector<Vector3> edgeVec_)
    {
        linkName = linkName_;
        edgeVec = edgeVec_;
    };

    ContactConstraintParam(const std::string linkName_, const std::vector<Vector2> vertexVec_)
    {
        linkName = linkName_;
        vertexVec = vertexVec_;
        calcEdgeFromVertex();
    }
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

    SimpleContactConstraintParam(const std::string linkName_, const std::vector<Vector3> edgeVec_)
        : ContactConstraintParam(linkName_, edgeVec_)
    {
        initialize();
    };

    SimpleContactConstraintParam(const std::string linkName_, const std::vector<Vector2> vertexVec_)
        : ContactConstraintParam(linkName_, vertexVec_)
    {
        initialize();
    };
};

class StaticContactConstraintParam : public SimpleContactConstraintParam
{
protected:
    void initialize(const double mu_)
    {
        muTrans = mu_;
        numInequals = 4 + edgeVec.size();// 静止摩擦制約4式
    }

public:
    double muTrans, muRot;

    void calcInequalMatrix();

    StaticContactConstraintParam(const std::string linkName_, const std::vector<Vector3> edgeVec_, const double mu_)
        : ContactConstraintParam(linkName_, edgeVec_),
          SimpleContactConstraintParam(linkName_, edgeVec_)
    {
        initialize(mu_);
    };

    StaticContactConstraintParam(const std::string linkName_, const std::vector<Vector2> vertexVec_, const double mu_)
        : ContactConstraintParam(linkName_, vertexVec_),
          SimpleContactConstraintParam(linkName_, vertexVec_)
    {
        initialize(mu_);
    }
};

class SlideContactConstraintParam : public SimpleContactConstraintParam
{
public:
    double muTrans, muRot;
    Vector3 direction;

    void calcInequalMatrix();
    void calcInequalMaximumVector();

    SlideContactConstraintParam(const std::string linkName_, const std::vector<Vector3> edgeVec_, const double mu_, const Vector3& direction_)
        : ContactConstraintParam(linkName_, edgeVec_),
          SimpleContactConstraintParam(linkName_, edgeVec_)
    {
        muTrans = mu_;
        direction = direction_;
        numInequals = 2 + edgeVec.size();// 動摩擦制約2式
    };

    SlideContactConstraintParam(const std::string linkName_, const std::vector<Vector2> vertexVec_, const double mu_, const Vector3& direction_)
        : ContactConstraintParam(linkName_, vertexVec_),
          SimpleContactConstraintParam(linkName_, vertexVec_)
    {
        *this = SlideContactConstraintParam(linkName_, edgeVec, mu_, direction_);
    }
};

}
