/**
   @author Kunio Kojima
*/

#include "ModelPredictiveController.h"

using namespace hrp;
using namespace std;

ModelPredictiveController::ModelPredictiveController()
{
    isInitial = true;
    parent = NULL;
}

void ModelPredictiveController::calcPhiMatrix()
{
    phiMat = dmatrix(stateDim*numWindows_,stateDim);
    dmatrix lastMat = dmatrix::Identity(stateDim,stateDim);
    for(std::deque<ModelPredictiveControllerParam*>::iterator iter = mpcParamDeque.begin(); iter != mpcParamDeque.end(); ++iter){
        int idx = std::distance(mpcParamDeque.begin(), iter);
        lastMat = (*iter)->systemMat * lastMat;
        phiMat.block(stateDim*idx,0, stateDim,stateDim) = lastMat;
    }
}

void ModelPredictiveController::calcPsiMatrix()
{
    psiMat = dmatrix::Zero(stateDim*numWindows_,psiCols);
    int colIdx = 0;
    for(std::deque<ModelPredictiveControllerParam*>::iterator Biter = mpcParamDeque.begin(); Biter != mpcParamDeque.end(); ){// Biterは内側のforループでインクリメント
        dmatrix lastMat = (*Biter)->inputMat;
        int cols = lastMat.cols();
        int Bidx = std::distance(mpcParamDeque.begin(), Biter);// column index
        psiMat.block(stateDim*Bidx,colIdx, stateDim,cols) = lastMat;
        for(std::deque<ModelPredictiveControllerParam*>::iterator Aiter = ++Biter; Aiter != mpcParamDeque.end(); ++Aiter){
            int Aidx = std::distance(mpcParamDeque.begin(), Aiter);// row index
            lastMat = (*Aiter)->systemMat * lastMat;
            psiMat.block(stateDim*Aidx,colIdx, stateDim,cols) = lastMat;
        }
        colIdx += cols;
    }
}

void ModelPredictiveController::calcEqualConstraints()
{
    equalMat = dmatrix::Zero(equalMatRows,URows);
    equalVec = dvector(equalMatRows);
    int rowIdx = 0;
    int colIdx = 0;
    hrp::dumpMatrix(equalMat, &ModelPredictiveControllerParam::equalMat, mpcParamDeque, rowIdx, colIdx, blockFlagVec);
    hrp::dumpVector(equalVec, &ModelPredictiveControllerParam::equalVec, mpcParamDeque, rowIdx, blockFlagVec);
}

void ModelPredictiveController::calcInequalConstraints()
{
    inequalMat = dmatrix::Zero(inequalMatRows,URows);
    inequalMinVec = dvector(inequalMatRows);
    inequalMaxVec = dvector(inequalMatRows);
    int rowIdx = 0;
    int colIdx = 0;
    hrp::dumpMatrix(inequalMat, &ModelPredictiveControllerParam::inequalMat, mpcParamDeque, rowIdx, colIdx, blockFlagVec);
    hrp::dumpVector(inequalMinVec, &ModelPredictiveControllerParam::inequalMinVec, mpcParamDeque, rowIdx, blockFlagVec);
    hrp::dumpVector(inequalMaxVec, &ModelPredictiveControllerParam::inequalMaxVec, mpcParamDeque, rowIdx, blockFlagVec);
}

void ModelPredictiveController::calcBoundVectors()
{
    minVec = dvector(URows);
    maxVec = dvector(URows);
    int rowIdx = 0;
    hrp::dumpVector(minVec, &ModelPredictiveControllerParam::minVec, mpcParamDeque, rowIdx, blockFlagVec);
    hrp::dumpVector(maxVec, &ModelPredictiveControllerParam::maxVec, mpcParamDeque, rowIdx, blockFlagVec);
}

void ModelPredictiveController::calcRefXVector()
{
    refXVec = dvector(stateDim*numWindows_);
    for(std::deque<ModelPredictiveControllerParam*>::iterator iter = mpcParamDeque.begin(); iter != mpcParamDeque.end(); ++iter){
        int idx = std::distance(mpcParamDeque.begin(), iter);
        refXVec.block(stateDim*idx,0, stateDim,1) = (*iter)->refStateVec;
    }
}

void ModelPredictiveController::calcErrorWeightMatrix()
{
    errorWeightMat = dmatrix::Zero(stateDim*numWindows_,stateDim*numWindows_);
    for(std::deque<ModelPredictiveControllerParam*>::iterator iter = mpcParamDeque.begin(); iter != mpcParamDeque.end(); ++iter){
        int idx = std::distance(mpcParamDeque.begin(), iter);
        errorWeightMat.block(stateDim*idx,stateDim*idx, stateDim,stateDim) = (*iter)->errorWeightVec.asDiagonal();
    }
}

void ModelPredictiveController::calcInputWeightMatrix()
{
    inputWeightMat = dmatrix::Zero(psiCols,psiCols);
    int rowIdx = 0;
    for(std::deque<ModelPredictiveControllerParam*>::iterator iter = mpcParamDeque.begin(); iter != mpcParamDeque.end(); ++iter){
         int rows = (*iter)->inputWeightVec.rows();
         inputWeightMat.block(rowIdx,rowIdx, rows,rows) = (*iter)->inputWeightVec.asDiagonal();
        rowIdx += rows;
    }
}

void ModelPredictiveController::calcBlockMatrix()
{
    blockMat = dmatrix::Zero(psiCols,URows);
    blockMatInv = dmatrix::Zero(URows,psiCols);
    std::vector<bool>::iterator blockFlagIter = blockFlagVec.begin();
    std::deque<ModelPredictiveControllerParam*>::iterator colIter = mpcParamDeque.begin();
    int count = 1, rowIdx = 0, colIdx = 0;
    for(std::deque<ModelPredictiveControllerParam*>::iterator rowIter = mpcParamDeque.begin(); rowIter != mpcParamDeque.end(); ++rowIter, ++colIter){
        int rows,cols;
        rows = (*rowIter)->inputDim;
        if(*blockFlagIter){
            cols = (*colIter)->inputDim;
            blockMatInv.block(colIdx,rowIdx, cols,rows) = dmatrix::Identity(cols,rows);// blockMatInv
        }
        blockMat.block(rowIdx,colIdx, rows,cols) = dmatrix::Identity(rows,cols);// blockMat
        if(*(++blockFlagIter)){
            colIdx += cols;
        }
        rowIdx += rows;
    }
}

void ModelPredictiveController::updateX0Vector()
{
    ModelPredictiveControllerParam* mpcParam = mpcParamDeque[0];
    // U->u0
    // x0 = A0*x0 + B'0*u0
    x0 = mpcParam->systemMat*x0 + mpcParam->inputMat*U.block(0,0, mpcParam->inputMat.cols(),1);
}

void ModelPredictiveController::calcAugmentedMatrix()
{
    psiCols = 0;
    equalMatRows = 0;
    inequalMatRows = 0;
    URows = 0;
    std::vector<bool>::iterator blockFlagIter = blockFlagVec.begin();
    for(std::deque<ModelPredictiveControllerParam*>::iterator iter = mpcParamDeque.begin(); iter != mpcParamDeque.end(); ++iter, ++blockFlagIter){
        psiCols += (*iter)->inputMat.cols();
        if(*blockFlagIter){
            URows += (*iter)->inputDim;
            equalMatRows += (*iter)->equalMat.rows();
            inequalMatRows += (*iter)->inequalMat.rows();
        }
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
    calcBlockMatrix();
    U = dvector::Zero(URows);

}

ModelPredictiveController* ModelPredictiveController::rootController()
{
    if(parent != NULL) return parent->rootController();
    return this;
}

void ModelPredictiveController::pushAllPreMPCParamFromRoot()
{
    ModelPredictiveController* root = rootController();
    int stepNum = dt/root->dt;// root->dtで割り切れる?
    std::deque<ModelPredictiveControllerParam*>::iterator iter = root->preMpcParamDeque.begin();
    while(iter != root->preMpcParamDeque.end()){
        preMpcParamDeque.push_back(copyMpcParam(this ,*iter));
        for(int i=0; i<stepNum && iter !=root->preMpcParamDeque.end(); ++i, ++iter);
    }
}
