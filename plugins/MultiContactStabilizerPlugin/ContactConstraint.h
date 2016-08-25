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
    Vector3 p,v,w;
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

class SimpleContactConstraintParam : virtual public ContactConstraintParam
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

class DistributedForceContactConstraintParam : public SimpleContactConstraintParam
{
protected:
    Vector2 startVertex;
    Vector2 diagonalVec;
    int nonDistributedDim;
    int distributedNum;
    int rows_,cols_;

public:
    std::vector<Vector3> forcePointVec;// in local coordinate

    virtual void calcMinimumVector();

    DistributedForceContactConstraintParam(const std::string linkName_, const std::vector<Vector2> vertexVec_, int rows, int cols)
        : ContactConstraintParam(linkName_, vertexVec_),
          SimpleContactConstraintParam(linkName_, vertexVec_)
    {
        inputForceDim = 6;// 6軸
        distributedNum = (rows+1)*(cols+1);// distributed fz
        nonDistributedDim = inputForceDim - 3; // 3 (fx,fy,tauz)
        inputDim = nonDistributedDim + distributedNum;// input dimension
        rows_ = rows;
        cols_ = cols;

        // forcePointVec
        startVertex = vertexVec.front();
        diagonalVec = Vector2::Zero();
        for(std::vector<Vector2>::iterator iter = vertexVec.begin(); iter != vertexVec.end(); ++iter){
            if((*iter - startVertex).norm() > diagonalVec.norm()) diagonalVec = *iter - startVertex;
        }
        double xStep = diagonalVec[0]/rows, yStep = diagonalVec[1]/cols;
        for(int i=0; i<rows+1; ++i){
            for(int j=0; j<cols+1; ++j){
                Vector3 forcePoint;
                forcePoint << startVertex[0]+xStep*i, startVertex[1]+yStep*j, 0;
                forcePointVec.push_back(forcePoint);
            }
        }

        // inputForceConvertMat
        inputForceConvertMat = dmatrix::Zero(inputForceDim, inputDim);
        inputForceConvertMat.block(0,0, inputForceDim,nonDistributedDim) <<
            1,0,0,// fx
            0,1,0,// fy
            0,0,0,// fz   distributed
            0,0,0,// taux distributed
            0,0,0,// tauy distributed
            0,0,1;// tauz
        inputForceConvertMat.block(2,nonDistributedDim, 1,distributedNum) = dmatrix::Ones(1,distributedNum);
        int idx = 0;
        for(std::vector<Vector3>::iterator iter = forcePointVec.begin(); iter != forcePointVec.end(); ++iter){
            inputForceConvertMat.block(3,nonDistributedDim+idx, 2,1) << (*iter)[1], -(*iter)[0];
            ++idx;
        }

        // inputWeightConvertMat
        inputWeightConvertMat = inputForceConvertMat;
        inputWeightConvertMat.block(3,0, 2,inputDim) = dmatrix::Zero(2,inputDim);
    };
};

class DistributedForceStaticContactConstraintParam : public StaticContactConstraintParam,
                                                     public DistributedForceContactConstraintParam
{
public:
    DistributedForceStaticContactConstraintParam(const std::string linkName_, const std::vector<Vector2> vertexVec_, const int rows, const int cols, const double mu_)
        : ContactConstraintParam(linkName_, vertexVec_),
          StaticContactConstraintParam(linkName_, vertexVec_, mu_),
          DistributedForceContactConstraintParam(linkName_, vertexVec_, rows, cols)
    {
        numInequals = 4 + edgeVec.size();// overwrite DFCCP
    }

    void calcInequalMatrix(){StaticContactConstraintParam::calcInequalMatrix();}
    void calcInequalMinimumVector(){StaticContactConstraintParam::calcInequalMinimumVector();}
    void calcInequalMaximumVector(){StaticContactConstraintParam::calcInequalMaximumVector();}
    void calcMinimumVector(){DistributedForceContactConstraintParam::calcMinimumVector();}
    void calcMaximumVector(){StaticContactConstraintParam::calcMaximumVector();}
};

}
