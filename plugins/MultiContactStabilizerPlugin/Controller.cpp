/**
   @author Kunio Kojima
*/
#include "Controller.h"

using namespace hrp;
using namespace std;

ModelPreviewController::ModelPreviewController()
{
    isInitial = true;
}

void ModelPreviewController::calcPhiMatrix()
{
    phiMat = dmatrix(stateDim*numWindows,stateDim);
    dmatrix lastMat = dmatrix::Identity(stateDim,stateDim);
    for(std::deque<ModelPreviewControllerParam*>::iterator iter = mpcParamDeque.begin(); iter != mpcParamDeque.end(); ++iter){
        int idx = std::distance(mpcParamDeque.begin(), iter);
        lastMat = (*iter)->systemMat * lastMat;
        phiMat.block(stateDim*idx,0, stateDim,stateDim) = lastMat;
    }
}

void ModelPreviewController::calcPsiMatrix()
{
    psiMat = dmatrix::Zero(stateDim*numWindows,psiCols);
    int colIdx = 0;
    for(std::deque<ModelPreviewControllerParam*>::iterator Biter = mpcParamDeque.begin(); Biter != mpcParamDeque.end(); ){// Biterは内側のforループでインクリメント
        dmatrix lastMat = (*Biter)->inputMat;
        int cols = lastMat.cols();
        int Bidx = std::distance(mpcParamDeque.begin(), Biter);// column index
        psiMat.block(stateDim*Bidx,colIdx, stateDim,cols) = lastMat;
        for(std::deque<ModelPreviewControllerParam*>::iterator Aiter = ++Biter; Aiter != mpcParamDeque.end(); ++Aiter){
            int Aidx = std::distance(mpcParamDeque.begin(), Aiter);// row index
            lastMat = (*Aiter)->systemMat * lastMat;
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
    for(std::deque<ModelPreviewControllerParam*>::iterator iter = mpcParamDeque.begin(); iter != mpcParamDeque.end(); ++iter){
        int rows = (*iter)->equalMat.rows();
        int cols = (*iter)->equalMat.cols();
        equalMat.block(rowIdx,colIdx, rows,cols) = (*iter)->equalMat;
        equalVec.block(rowIdx,0,      rows,1) = (*iter)->equalVec;
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
    for(std::deque<ModelPreviewControllerParam*>::iterator iter = mpcParamDeque.begin(); iter != mpcParamDeque.end(); ++iter){
        int rows = (*iter)->inequalMat.rows();
        int cols = (*iter)->inequalMat.cols();
        inequalMat.block(rowIdx,colIdx, rows,cols) = (*iter)->inequalMat;
        inequalMinVec.block(rowIdx,0,   rows,1) = (*iter)->inequalMinVec;
        inequalMaxVec.block(rowIdx,0,   rows,1) = (*iter)->inequalMaxVec;
        rowIdx += rows;
        colIdx += cols;
    }
}

void ModelPreviewController::calcBoundVectors()
{
    minVec = dvector(psiCols);
    maxVec = dvector(psiCols);
    int rowIdx = 0;
    for(std::deque<ModelPreviewControllerParam*>::iterator iter = mpcParamDeque.begin(); iter != mpcParamDeque.end(); ++iter){
        int rows = (*iter)->minVec.rows();
        minVec.block(rowIdx,0, rows,1) = (*iter)->minVec;
        maxVec.block(rowIdx,0, rows,1) = (*iter)->maxVec;
        rowIdx += rows;
    }
}

void ModelPreviewController::calcRefXVector()
{
    refX = dvector(stateDim*numWindows);
    for(std::deque<ModelPreviewControllerParam*>::iterator iter = mpcParamDeque.begin(); iter != mpcParamDeque.end(); ++iter){
        int idx = std::distance(mpcParamDeque.begin(), iter);
        refX.block(stateDim*idx,0, stateDim,1) = (*iter)->refStateVec;
    }
}

void ModelPreviewController::calcErrorWeightMatrix()
{
    errorWeightMat = dmatrix::Zero(stateDim*numWindows,stateDim*numWindows);
    for(std::deque<ModelPreviewControllerParam*>::iterator iter = mpcParamDeque.begin(); iter != mpcParamDeque.end(); ++iter){
        int idx = std::distance(mpcParamDeque.begin(), iter);
        errorWeightMat.block(stateDim*idx,stateDim*idx, stateDim,stateDim) = (*iter)->errorWeightVec.asDiagonal();
    }
}

void ModelPreviewController::calcInputWeightMatrix()
{
    inputWeightMat = dmatrix::Zero(psiCols,psiCols);
    int rowIdx = 0;
    for(std::deque<ModelPreviewControllerParam*>::iterator iter = mpcParamDeque.begin(); iter != mpcParamDeque.end(); ++iter){
         int rows = (*iter)->inputWeightVec.rows();
        inputWeightMat.block(rowIdx,rowIdx, rows,rows) = (*iter)->inputWeightVec.asDiagonal();
        rowIdx += rows;
    }
}

void ModelPreviewController::updateX0Vector()
{
    ModelPreviewControllerParam* mpcParam = mpcParamDeque[0];
    // U->u0
    // x0 = A0*x0 + B'0*u0
    x0 = mpcParam->systemMat*x0 + mpcParam->inputMat*U.block(0,0, mpcParam->inputMat.cols(),1);
}

void ModelPreviewController::calcAugmentedMatrix()
{
    psiCols = 0;
    equalMatRows = 0;
    inequalMatRows = 0;
    for(std::deque<ModelPreviewControllerParam*>::iterator iter = mpcParamDeque.begin(); iter != mpcParamDeque.end(); ++iter){
        psiCols += (*iter)->inputMat.cols();
        equalMatRows += (*iter)->equalMat.rows();
        inequalMatRows += (*iter)->inequalMat.rows();
    }

    if(isInitial){
        x0 = dvector(stateDim);
        x0 = mpcParamDeque[0]->refStateVec;
        isInitial = false;
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

}

MultiContactStabilizer::MultiContactStabilizer()
    : ModelPreviewController()
{
    unitInputDim = 6;// 接触点ごとの入力次元
    stateDim = 6;
}

void MultiContactStabilizer::setupQP()
{
    qpInterface = QP(psiCols, equalMatRows, inequalMatRows);

}

int MultiContactStabilizer::execQP()
{
    static int count = 0;
    if(count == 0 && COUT){
        cout << "psi:" << endl << psiMat << endl << endl;
        cout << "phi:" << endl << phiMat << endl << endl;
        cout << "refX:" << endl << refX.transpose() << endl << endl;
        cout << "x0:" << endl << x0.transpose() << endl << endl;
        cout << "W1(error):" << endl << errorWeightMat << endl << endl;
        cout << "W2(input):" << endl << inputWeightMat << endl << endl;
    }
    int ret  = qpInterface.execQP(U,
                                  psiMat.transpose()*errorWeightMat*psiMat + inputWeightMat, psiMat.transpose()*errorWeightMat*(phiMat*x0 - refX),
                                  equalMat, equalVec,
                                  inequalMat, inequalMinVec, inequalMaxVec,
                                  minVec, maxVec);
    ++count;
    return ret;
}

void MultiContactStabilizerParam::calcSystemMatrix()
{
    systemMat = dmatrix::Identity(stateDim,stateDim);
    const double m = controller->m;
    double Fz = F(2);
    systemMat(0,1) = dt;
    systemMat(2,3) = dt;
    systemMat(4,2) = -dt*Fz/m;
    systemMat(5,0) = dt*Fz/m;
}

void MultiContactStabilizerParam::calcInputMatrix()
{
    inputMat = dmatrix::Zero(stateDim,inputDim);
    for(std::vector<ContactConstraintParam>::iterator iter = ccParamVec.begin(); iter != ccParamVec.end(); ++iter){
        int idx = std::distance(ccParamVec.begin(), iter);
        Matrix33 R = (*iter).R;
        Vector3 p = (*iter).p;

        dmatrix R2 = dmatrix::Zero(unitInputDim,unitInputDim);
        R2.block(0,0,3,3) = R;
        R2.block(3,3,3,3) = R;
        dmatrix unitInputMat = dmatrix(stateDim,unitInputDim);// B_k^i
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

void MultiContactStabilizerParam::calcEqualMatrix()
{
    equalMat = dmatrix::Zero(numEquals, inputDim);
    for(std::vector<ContactConstraintParam>::iterator iter = ccParamVec.begin(); iter != ccParamVec.end(); ++iter){
        int idx = std::distance(ccParamVec.begin(), iter);
        Matrix33 R = (*iter).R;
        equalMat.block(0,unitInputDim*idx, 1,3) = R.block(2,0,1,3);
    }
}

void MultiContactStabilizerParam::calcEqualVector()
{
    equalVec = dvector::Zero(numEquals);
    equalVec(0) = F(2);
}

void MultiContactStabilizerParam::calcInequalMatrix()
{
    inequalMat = dmatrix::Zero(numInequals, inputDim);
    for(std::vector<ContactConstraintParam>::iterator iter = ccParamVec.begin(); iter != ccParamVec.end(); ++iter){
        int idx = std::distance(ccParamVec.begin(), iter);
        // Matrix33 R = (*iter).R;
        // Vector3 p = (*iter).p;

        std::vector<Vector3> edgeVec = (*iter).edgeVec;
        for(std::vector<Vector3>::iterator edgeIter = edgeVec.begin(); edgeIter != edgeVec.end(); ++edgeIter){
            int j = std::distance(edgeVec.begin(), edgeIter);
            Vector3 vec = (*edgeIter);
            inequalMat.block((*iter).numInequals*idx+j,unitInputDim*idx, 1,6) << 0,0,vec[2], vec[1],-vec[0],0;
        }
        inequalMat.block((*iter).numInequals*idx+edgeVec.size(),unitInputDim*idx, 4,3) << -1,0,(*iter).mu, 0,-1,(*iter).mu, 1,0,(*iter).mu, 0,1,(*iter).mu;
    }
}

void MultiContactStabilizerParam::calcMinimumVector()
{
    minVec = dvector::Constant(inputDim,-INFINITY);
    for(std::vector<ContactConstraintParam>::iterator iter = ccParamVec.begin(); iter != ccParamVec.end(); ++iter){
        int idx = std::distance(ccParamVec.begin(), iter);

        minVec.block(unitInputDim*idx+2,0,1,1) = dmatrix::Zero(1,1);
    }
}

void MultiContactStabilizerParam::calcRefStateVector()
{
    refStateVec = dvector(stateDim);
    refStateVec[0] = CM(0);
    refStateVec[1] = P(0);
    refStateVec[2] = CM(1);
    refStateVec[3] = P(1);
    refStateVec[4] = L(0);
    refStateVec[5] = L(1);
}

void MultiContactStabilizerParam::calcErrorWeightVector()
{
    errorWeightVec = dvector(stateDim);
    double errorCMWeight = controller->errorCMWeight, errorMomentumWeight = controller->errorMomentumWeight, errorAngularMomentumWeight = controller->errorAngularMomentumWeight;
    errorWeightVec << errorCMWeight,errorMomentumWeight, errorCMWeight,errorMomentumWeight, errorAngularMomentumWeight,errorAngularMomentumWeight;
}

void MultiContactStabilizerParam::calcInputWeightVector()
{
    inputWeightVec = dvector(inputDim);
    double inputForceWeight = controller->inputForceWeight, inputMomentWeight = controller->inputMomentWeight;
    for(std::vector<ContactConstraintParam>::iterator iter = ccParamVec.begin(); iter != ccParamVec.end(); ++iter){
        int idx = std::distance(ccParamVec.begin(), iter);
        inputWeightVec.block(unitInputDim*idx,0, unitInputDim,1) << inputForceWeight,inputForceWeight,inputForceWeight, inputMomentWeight,inputMomentWeight,inputMomentWeight;
    }
}

void MultiContactStabilizerParam::convertToMPCParam()
{
    inputDim = unitInputDim*ccParamVec.size();

    calcEqualMatrix();
    calcEqualVector();

    calcInequalMatrix();
    inequalMinVec = dvector::Zero(numInequals);
    inequalMaxVec = dvector::Constant(numInequals,INFINITY);

    calcMinimumVector();
    maxVec = dvector::Constant(inputDim,INFINITY);

    calcInputMatrix();
    calcSystemMatrix();

    calcRefStateVector();

    calcErrorWeightVector();
    calcInputWeightVector();
}
