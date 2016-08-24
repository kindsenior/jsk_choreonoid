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
    inequalMat = dmatrix::Zero(numInequals,unitInputDim);
    for(std::vector<Vector3>::iterator iter = edgeVec.begin(); iter != edgeVec.end(); ++iter){
        int idx = std::distance(edgeVec.begin(), iter);
        inequalMat.block(idx,0, 1,6) << 0,0,(*iter)[2], (*iter)[1],-(*iter)[0],0;
    }
}

void StaticContactConstraintParam::calcInequalMatrix()
{
    SimpleContactConstraintParam::calcInequalMatrix();
    inequalMat.block(edgeVec.size(),0, 4,3) << -1,0,muTrans, 0,-1,muTrans, 1,0,muTrans, 0,1,muTrans;
}

void SlideContactConstraintParam::calcInequalMatrix()
{
    SimpleContactConstraintParam::calcInequalMatrix();
    inequalMat.block(edgeVec.size(),0, 2,3) << 1,0,muTrans*math::sign(direction(0)), 0,1,muTrans*math::sign(direction(1));
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

// clacMinimumVector
void SimpleContactConstraintParam::calcMinimumVector()
{
    minVec = dvector::Constant(unitInputDim,-INFINITY);
    minVec(2) = 0;
}

// calcMaximumVector
void SimpleContactConstraintParam::calcMaximumVector()
{
    maxVec = dvector::Constant(unitInputDim,INFINITY);
}
