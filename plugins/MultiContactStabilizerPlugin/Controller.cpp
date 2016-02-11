#include "Controller.h"

using namespace cnoid;
using namespace hrp;

ModelPreviewController::ModelPreviewController()
{
}

MultiContactStabilizer::MultiContactStabilizer()
    : ModelPreviewController()
{
    unitInputDim = 6;// 接触点ごとの入力次元
    stateDim = 6;
}

void MultiContactStabilizerParam::calcSystemMatrix(dmatrix& systemMat)
{
    const double m = controller->m;
    double Fz = F(2);
    systemMat(0,1) = dt;
    systemMat(2,3) = dt;
    systemMat(4,2) = -dt*Fz/m;
    systemMat(5,0) = dt*Fz/m;
}

void MultiContactStabilizerParam::calcInputMatrix(dmatrix& inputMat)
{
    const int stateDim = controller->stateDim;

    for(std::vector<ContactConstraintParam>::iterator iter = ccParamVec.begin(); iter != ccParamVec.end(); ++iter){
        int idx = std::distance(ccParamVec.begin(), iter);
        Matrix33 R = (*iter).R;
        Vector3 p = (*iter).p;

        dmatrix R2 = dmatrix::Zero(unitInputDim,unitInputDim);
        R2.block(0,0,3,3) = R;
        R2.block(3,3,3,3) = R;
        dmatrix unitInputMat = dmatrix(stateDim,unitInputDim);
        double T2_2 = dt*dt/2;
        double beta = dt*(p(2)-CM(2));
        unitInputMat <<
            T2_2,0, 0,        0, 0,0,
            dt,  0, 0,        0, 0,0,
            0,T2_2, 0,        0, 0,0,
            0,dt,   0,        0, 0,0,
            0,-beta,dt*p(1), dt,0,0,
            beta,0,-dt*p(0), 0,dt,0;
        inputMat.block(0,unitInputDim*idx, stateDim,unitInputDim) = unitInputMat*R2;
    }
}

void MultiContactStabilizerParam::calcEqualMatrix(dmatrix& equalMat)
{
    for(std::vector<ContactConstraintParam>::iterator iter = ccParamVec.begin(); iter != ccParamVec.end(); ++iter){
        int idx = std::distance(ccParamVec.begin(), iter);
        Matrix33 R = (*iter).R;
        Vector3 p = (*iter).p;

        equalMat.block((*iter).numEquals*idx,unitInputDim*idx, 1,3) = R.block(2,0,1,3);
    }
}

void MultiContactStabilizerParam::calcEqualVector(dvector& equalVec)
{
    for(std::vector<ContactConstraintParam>::iterator iter = ccParamVec.begin(); iter != ccParamVec.end(); ++iter){
        int idx = std::distance(ccParamVec.begin(), iter);
        equalVec((*iter).numEquals*idx) = F(2);
    }
}

void MultiContactStabilizerParam::calcInequalMatrix(dmatrix& inequalMat)
{
    for(std::vector<ContactConstraintParam>::iterator iter = ccParamVec.begin(); iter != ccParamVec.end(); ++iter){
        int idx = std::distance(ccParamVec.begin(), iter);
        Matrix33 R = (*iter).R;
        Vector3 p = (*iter).p;

        std::vector<Vector3> edgeVec = (*iter).edgeVec;
        for(std::vector<Vector3>::iterator edgeIter = edgeVec.begin(); edgeIter != edgeVec.end(); ++edgeIter){
            int j = std::distance(edgeVec.begin(), edgeIter);
            Vector3 vec = (*edgeIter);
            inequalMat.block((*iter).numInequals*idx+j,unitInputDim*idx, 1,6) << 0,0,vec[2], vec[1],-vec[0],0;
        }
        inequalMat.block((*iter).numInequals*idx+edgeVec.size(),unitInputDim*idx, 4,3) << -1,0,(*iter).mu, 0,-1,(*iter).mu, 1,0,(*iter).mu, 0,1,(*iter).mu;
    }
}

void MultiContactStabilizerParam::calcMinimumVector(dvector& minVec)
{
    for(std::vector<ContactConstraintParam>::iterator iter = ccParamVec.begin(); iter != ccParamVec.end(); ++iter){
        int idx = std::distance(ccParamVec.begin(), iter);

        minVec.block(unitInputDim*idx+2,0,1,1) = dmatrix::Zero(1,1);
    }
}

void MultiContactStabilizerParam::convertToMPCParam(ModelPreviewControllerParam& mpcParam)
{
    const int numContact = ccParamVec.size();
    const int stateDim = controller->stateDim;
    const int inputDim = unitInputDim*numContact;

    // equal matrix
    mpcParam.equalMat = dmatrix::Zero(numEquals, 6*numContact);
    calcEqualMatrix(mpcParam.equalMat);
    // equal vector
    mpcParam.equalVec = dvector::Zero(numEquals);
    calcEqualVector(mpcParam.equalVec);

    // inequal matrix
    mpcParam.inequalMat = dmatrix::Zero(numInequals, 6*numContact);
    calcInequalMatrix(mpcParam.inequalMat);

    mpcParam.inequalMinVec = dvector::Zero(numInequals);
    mpcParam.inequalMaxVec = dvector::Constant(numInequals,INFINITY);

    // max min
    mpcParam.minVec     = dvector::Constant(inputDim,-INFINITY);
    calcMinimumVector(mpcParam.minVec);
    mpcParam.maxVec     = dvector::Constant(inputDim,INFINITY);

    // input matrix
    mpcParam.inputMat = dmatrix::Zero(stateDim,inputDim);
    calcInputMatrix(mpcParam.inputMat);
    // system matrix
    mpcParam.systemMat = dmatrix::Identity(stateDim,stateDim);
    calcSystemMatrix(mpcParam.systemMat);
}
