/**
   @author Kunio Kojima
*/

#include "ContactConstraint.h"

using namespace hrp;
using namespace std;
using namespace boost;

// calcInequalMatrix
void SimpleContactConstraintParam::calcInequalMatrix()
{
    inequalMat = dmatrix::Zero(numInequals,inputDim);
    for(std::vector<Vector3>::iterator iter = edgeVec.begin(); iter != edgeVec.end(); ++iter){
        int idx = std::distance(edgeVec.begin(), iter);
        dmatrix tmpMat(1,inputForceDim);// 1x6
        tmpMat << 0,0,(*iter)[2], (*iter)[1],-(*iter)[0],0;
        inequalMat.block(idx,0, 1,inputDim) << tmpMat*inputForceConvertMat;
    }
}

void StaticContactConstraintParam::calcInequalMatrix()
{
    SimpleContactConstraintParam::calcInequalMatrix();
    // only translation friction constraint
    // dmatrix tmpMat(4,inputForceDim);
    // tmpMat <<
    //     -1,0,muTrans, 0,0,0,
    //     0,-1,muTrans, 0,0,0,
    //     1,0,muTrans,  0,0,0,
    //     0,1,muTrans,  0,0,0;
    // inequalMat.block(edgeVec.size(),0, 4,inputDim) = tmpMat*inputForceConvertMat;
    dmatrix tmpMat(6,inputForceDim);// 4+2=6
    tmpMat <<
        -1,0,muTrans, 0,0,0,
        0,-1,muTrans, 0,0,0,
        1,0,muTrans,  0,0,0,
        0,1,muTrans,  0,0,0,
        0,0,muRot,    0,0,1,
        0,0,muRot,    0,0,-1;
    inequalMat.block(edgeVec.size(),0, 6,inputDim) = tmpMat*inputForceConvertMat;
    // cout << "inequalMat:" << endl << inequalMat << endl;
}

void SlideContactConstraintParam::calcInequalMatrix()
{
    SimpleContactConstraintParam::calcInequalMatrix();
    inequalMat.block(edgeVec.size(),0, 2,3) << 1,0,muTrans*math::sign(direction(0)), 0,1,muTrans*math::sign(direction(1));// signは不要?
}

void DistributedForceSlideContactConstraintParam::calcInequalMatrix()
{
    SimpleContactConstraintParam::calcInequalMatrix();
    inequalMat.block(edgeVec.size(),0, nonDistributedDim,nonDistributedDim) = dmatrix::Identity(nonDistributedDim,nonDistributedDim);

    int rowIdx = edgeVec.size();
    int colIdx = nonDistributedDim;
    for(std::vector<Vector3>::iterator rIter = forcePointVec.begin(); rIter != forcePointVec.end(); ++rIter){
        Vector3 ri = *rIter;
        Vector3 di = R.transpose()*(-v) + (R.transpose()*(-w)).cross((*rIter));// in local coordinate
        di.normalize();

        inequalMat.block(rowIdx,colIdx, 2,1) = -muTrans*di.head(2);// -mu*dxy
        inequalMat(rowIdx+2,colIdx) = -muTrans*(ri(0)*di(1) - ri(1)*di(0));
        ++colIdx;
    }
    // cout << "p:" << p.transpose() << endl;
    // cout << "v:" << v.transpose() << endl;
    // cout << "w:" << w.transpose() << endl;
    // cout << "R:" << endl << R << endl;
    // cout << "inequalMat:" << endl << inequalMat << endl;
}

// calcInequalMinimumVector
void SimpleContactConstraintParam::calcInequalMinimumVector()
{
    inequalMinVec = dvector::Zero(numInequals);
}

// calcInequalMaximumVector
void SimpleContactConstraintParam::calcInequalMaximumVector()
{
    inequalMaxVec = dvector::Constant(numInequals,INFINITY);
}

void SlideContactConstraintParam::calcInequalMaximumVector()
{
    SimpleContactConstraintParam::calcInequalMaximumVector();
    inequalMaxVec(edgeVec.size()) = 0;
    inequalMaxVec(edgeVec.size()+1) = 0;
}

void DistributedForceSlideContactConstraintParam::calcInequalMaximumVector()
{
    SimpleContactConstraintParam::calcInequalMaximumVector();
    inequalMaxVec.tail(3) = dvector::Zero(3);// translation x, translation y, rotation z
}

// clacMinimumVector
void SimpleContactConstraintParam::calcMinimumVector()
{
    minVec = dvector::Constant(inputDim,-INFINITY);
    minVec(2) = 0;
}

void DistributedForceContactConstraintParam::calcMinimumVector()
{
    minVec = dvector::Constant(inputDim,-INFINITY);
    minVec.tail(distributedNum) = dvector::Zero(distributedNum);// fx,fy,tauz, fz0,fz1,...
}

// calcMaximumVector
void SimpleContactConstraintParam::calcMaximumVector()
{
    maxVec = dvector::Constant(inputDim,INFINITY);
}
