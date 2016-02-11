#include "Controller.h"

using namespace hrp;

ModelPreviewController::ModelPreviewController()
{
    isInitial = true;
}

void ModelPreviewController::calcPhiMatrix()
{
    phiMat = dmatrix(stateDim*numWindows,stateDim);
    dmatrix lastMat = dmatrix::Identity(stateDim,stateDim);
    for(std::deque<ModelPreviewControllerParam>::iterator iter = mpcParamDeque.begin(); iter != mpcParamDeque.end(); ++iter){
        int idx = std::distance(mpcParamDeque.begin(), iter);
        lastMat = (*iter).systemMat * lastMat;
        phiMat.block(stateDim*idx,0, stateDim,stateDim) = lastMat;
    }
}

void ModelPreviewController::calcPsiMatrix()
{
    psiMat = dmatrix::Zero(stateDim*numWindows,psiCols);
    int colIdx = 0;
    for(std::deque<ModelPreviewControllerParam>::iterator Biter = mpcParamDeque.begin(); Biter != mpcParamDeque.end(); ){// Biterは内側のforループでインクリメント
        dmatrix lastMat = (*Biter).inputMat;
        int cols = lastMat.cols();
        int Bidx = std::distance(mpcParamDeque.begin(), Biter);// column index
        psiMat.block(stateDim*Bidx,colIdx, stateDim,cols) = lastMat;
        for(std::deque<ModelPreviewControllerParam>::iterator Aiter = ++Biter; Aiter != mpcParamDeque.end(); ++Aiter){
            int Aidx = std::distance(mpcParamDeque.begin(), Aiter);// row index
            lastMat = (*Aiter).systemMat * lastMat;
            psiMat.block(stateDim*Aidx,colIdx, stateDim,cols) = lastMat;
        }
        colIdx += cols;
    }
}

void ModelPreviewController::calcEqualConstraints()
{
    equalMat = dmatrix::Zero(equalMatRows,psiCols);
    equalVec = dvector(equalMatRows);
    int rowIdx = 0;
    int colIdx = 0;
    for(std::deque<ModelPreviewControllerParam>::iterator iter = mpcParamDeque.begin(); iter != mpcParamDeque.end(); ++iter){
        int rows = (*iter).equalMat.rows();
        int cols = (*iter).equalMat.cols();
        equalMat.block(rowIdx,colIdx, rows,cols) = (*iter).equalMat;
        equalVec.block(rowIdx,0,      rows,1) = (*iter).equalVec;
        rowIdx += rows;
        colIdx += cols;
    }
}

void ModelPreviewController::calcInequalConstraints()
{
    inequalMat = dmatrix::Zero(inequalMatRows,psiCols);
    inequalMinVec = dvector(inequalMatRows);
    inequalMaxVec = dvector(inequalMatRows);
    int rowIdx = 0;
    int colIdx = 0;
    for(std::deque<ModelPreviewControllerParam>::iterator iter = mpcParamDeque.begin(); iter != mpcParamDeque.end(); ++iter){
        int rows = (*iter).inequalMat.rows();
        int cols = (*iter).inequalMat.cols();
        inequalMat.block(rowIdx,colIdx, rows,cols) = (*iter).inequalMat;
        inequalMinVec.block(rowIdx,0,   rows,1) = (*iter).inequalMinVec;
        inequalMaxVec.block(rowIdx,0,   rows,1) = (*iter).inequalMaxVec;
        rowIdx += rows;
        colIdx += cols;
    }
}

void ModelPreviewController::calcBoundVectors()
{
    minVec = dvector(psiCols);
    maxVec = dvector(psiCols);
    int rowIdx = 0;
    for(std::deque<ModelPreviewControllerParam>::iterator iter = mpcParamDeque.begin(); iter != mpcParamDeque.end(); ++iter){
        int rows = (*iter).minVec.rows();
        minVec.block(rowIdx,0, rows,1) = (*iter).minVec;
        maxVec.block(rowIdx,0, rows,1) = (*iter).maxVec;
        rowIdx += rows;
    }
}

void ModelPreviewController::calcRefXVector()
{
    refX = dvector(stateDim*numWindows);
    for(std::deque<ModelPreviewControllerParam>::iterator iter = mpcParamDeque.begin(); iter != mpcParamDeque.end(); ++iter){
        int idx = std::distance(mpcParamDeque.begin(), iter);
        refX.block(stateDim*idx,0, stateDim,1) = (*iter).refStateVec;
    }
}

void ModelPreviewController::calcErrorWeightMatrix()
{
    errorWeightMat = dmatrix::Zero(stateDim*numWindows,stateDim*numWindows);
    for(std::deque<ModelPreviewControllerParam>::iterator iter = mpcParamDeque.begin(); iter != mpcParamDeque.end(); ++iter){
        int idx = std::distance(mpcParamDeque.begin(), iter);
        errorWeightMat.block(stateDim*idx,stateDim*idx, stateDim,stateDim) = (*iter).errorWeightVec.asDiagonal();
    }
}

void ModelPreviewController::calcInputWeightMatrix()
{
    inputWeightMat = dmatrix::Zero(psiCols,psiCols);
    int rowIdx = 0;
    for(std::deque<ModelPreviewControllerParam>::iterator iter = mpcParamDeque.begin(); iter != mpcParamDeque.end(); ++iter){
         int rows = (*iter).inputWeightVec.rows();
        inputWeightMat.block(rowIdx,rowIdx, rows,rows) = (*iter).inputWeightVec.asDiagonal();
        rowIdx += rows;
    }
}

void ModelPreviewController::calcX0Vector()
{
    x0 = dvector(stateDim);
    if(isInitial){
        x0 = mpcParamDeque[0].refStateVec;
        isInitial = false;
    }else{
        // U->u0
        // x0,u0->x1
        x0 = dvector::Zero(stateDim);
    }
}

void ModelPreviewController::calcAugmentedMatrix()
{
    psiCols = 0;
    equalMatRows = 0;
    inequalMatRows = 0;
    for(std::deque<ModelPreviewControllerParam>::iterator iter = mpcParamDeque.begin(); iter != mpcParamDeque.end(); ++iter){
        psiCols += (*iter).inputMat.cols();
        equalMatRows += (*iter).equalMat.rows();
        inequalMatRows += (*iter).inequalMat.rows();
    }

    calcPhiMatrix();
    calcPsiMatrix();
    calcEqualConstraints();
    calcInequalConstraints();
    calcBoundVectors();
    calcRefXVector();
    calcErrorWeightMatrix();
    calcInputWeightMatrix();
    U = dvector::Zero(psiCols);
    calcX0Vector();//要改良

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

void MultiContactStabilizerParam::calcRefStateVector(dvector& refStateVec)
{
    refStateVec[0] = CM(0);
    refStateVec[1] = P(0);
    refStateVec[2] = CM(1);
    refStateVec[3] = P(1);
    refStateVec[4] = L(0);
    refStateVec[5] = L(1);
}

void MultiContactStabilizerParam::calcErrorWeightVector(dvector& errorWeightVec)
{
    errorWeightVec << 1e1,1e2, 1e1,1e2, 1e3,1e3;
}

void MultiContactStabilizerParam::calcInputWeightVector(dvector& inputWeightVec)
{
    for(std::vector<ContactConstraintParam>::iterator iter = ccParamVec.begin(); iter != ccParamVec.end(); ++iter){
        int idx = std::distance(ccParamVec.begin(), iter);
        inputWeightVec.block(unitInputDim*idx,0, unitInputDim,1) << 1e-3,1e-3,1e-3, 1e4,1e4,1e4;
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

    // ref state vector
    mpcParam.refStateVec = dvector(stateDim);
    calcRefStateVector(mpcParam.refStateVec);

    // error weight Vector
    mpcParam.errorWeightVec = dvector(stateDim);
    calcErrorWeightVector(mpcParam.errorWeightVec);

    // input weight vector
    mpcParam.inputWeightVec = dvector(inputDim);
    calcInputWeightVector(mpcParam.inputWeightVec);
}
