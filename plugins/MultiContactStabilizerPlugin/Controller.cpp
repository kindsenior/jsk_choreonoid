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

void MultiContactStabilizer::calcSystemMatrix(dmatrix& systemMat, MultiContactStabilizerParam& mcsParam)
{
    double Fz = mcsParam.F.z();
    systemMat(0,1) = dt;
    systemMat(2,3) = dt;
    systemMat(4,2) = -dt*Fz/m;
    systemMat(5,0) = dt*Fz/m;
}

void MultiContactStabilizer::calcInputMatrix(dmatrix& inputMat, MultiContactStabilizerParam& mcsParam)
{
    std::vector<ContactConstraintParam> ccParamVec = mcsParam.ccParamVec;

    for(std::vector<ContactConstraintParam>::iterator iter = ccParamVec.begin(); iter != ccParamVec.end(); ++iter){
        int idx = std::distance(ccParamVec.begin(), iter);
        Matrix33 R = (*iter).R;
        Vector3 p = (*iter).p;

        dmatrix R2 = dmatrix::Zero(unitInputDim,unitInputDim);
        R2.block(0,0,3,3) = R;
        R2.block(3,3,3,3) = R;
        dmatrix unitInputMat = dmatrix(stateDim,unitInputDim);
        double T2_2 = dt*dt/2;
        double beta = dt*(p.z()-mcsParam.CM.z());
        unitInputMat <<
            T2_2,0, 0,        0, 0,0,
            dt,  0, 0,        0, 0,0,
            0,T2_2, 0,        0, 0,0,
            0,dt,   0,        0, 0,0,
            0,-beta,dt*p.y(), dt,0,0,
            beta,0,-dt*p.x(), 0,dt,0;
        inputMat.block(0,unitInputDim*idx, stateDim,unitInputDim) = unitInputMat*R2;
    }
}

void MultiContactStabilizer::calcEqualMatrix(dmatrix& equalMat, MultiContactStabilizerParam& mcsParam)
{
    std::vector<ContactConstraintParam> ccParamVec = mcsParam.ccParamVec;

    for(std::vector<ContactConstraintParam>::iterator iter = ccParamVec.begin(); iter != ccParamVec.end(); ++iter){
        int idx = std::distance(ccParamVec.begin(), iter);
        Matrix33 R = (*iter).R;
        Vector3 p = (*iter).p;

        equalMat.block((*iter).numEquals*idx,unitInputDim*idx, 1,3) = R.block(2,0,1,3);
    }
}

void MultiContactStabilizer::calcEqualVector(dvector& equalVec, MultiContactStabilizerParam& mcsParam)
{
    std::vector<ContactConstraintParam> ccParamVec = mcsParam.ccParamVec;

    for(std::vector<ContactConstraintParam>::iterator iter = ccParamVec.begin(); iter != ccParamVec.end(); ++iter){
        int idx = std::distance(ccParamVec.begin(), iter);
        equalVec((*iter).numEquals*idx) = mcsParam.F.z();
    }
}

void MultiContactStabilizer::calcInequalMatrix(dmatrix& inequalMat, MultiContactStabilizerParam& mcsParam)
{
    std::vector<ContactConstraintParam> ccParamVec = mcsParam.ccParamVec;

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

void MultiContactStabilizer::calcMinimumVector(dvector& minVec, MultiContactStabilizerParam& mcsParam)
{
    std::vector<ContactConstraintParam> ccParamVec = mcsParam.ccParamVec;

    for(std::vector<ContactConstraintParam>::iterator iter = ccParamVec.begin(); iter != ccParamVec.end(); ++iter){
        int idx = std::distance(ccParamVec.begin(), iter);

        minVec.block(unitInputDim*idx+2,0,1,1) = dmatrix::Zero(1,1);
    }
}

