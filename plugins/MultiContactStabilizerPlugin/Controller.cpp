/**
   @author Kunio Kojima
*/
#include "Controller.h"

using namespace hrp;
using namespace std;

ModelPredictiveController::ModelPredictiveController()
{
    isInitial = true;
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

namespace {
    template <typename containerClass, typename paramClass> void dumpMatrix(dmatrix& retMat, dmatrix paramClass::*inMat, containerClass& container, int rowIdx, int colIdx, std::vector<bool> flagVec)
    {
        std::vector<bool>::iterator flagIter = flagVec.begin();
        for(typename containerClass::iterator iter = container.begin(); iter != container.end(); ++iter, ++flagIter){
            if(*flagIter){
                int rows = ((*iter)->*inMat).rows();
                int cols = ((*iter)->*inMat).cols();
                retMat.block(rowIdx,colIdx, rows,cols) = (*iter)->*inMat;
                rowIdx += rows;
                colIdx += cols;
            }
        }
    }
    template <typename containerClass, typename paramClass> void dumpMatrix(dmatrix& retMat, dmatrix paramClass::*inMat, containerClass& container, int rowIdx, int colIdx)
    {
        std::vector<bool> flagVec(container.size(), true);
        ::dumpMatrix(retMat, inMat, container, rowIdx,colIdx, flagVec);
    }

    template <typename containerClass, typename paramClass> void dumpVector(dvector& retVec, dvector paramClass::*inVec, containerClass& container, int rowIdx, std::vector<bool> flagVec)
    {
        std::vector<bool>::iterator flagIter = flagVec.begin();
        for(typename containerClass::iterator iter = container.begin(); iter != container.end(); ++iter, ++flagIter){
            if(*flagIter){
                int rows = ((*iter)->*inVec).rows();
                retVec.block(rowIdx,0, rows,1) = (*iter)->*inVec;
                rowIdx += rows;
            }
        }
    }
    template <typename containerClass, typename paramClass> void dumpVector(dvector& retVec, dvector paramClass::*inVec, containerClass& container, int rowIdx)
    {
        std::vector<bool> flagVec(container.size(), true);
        ::dumpVector(retVec, inVec, container, rowIdx, flagVec);
    }
}

void ModelPredictiveController::calcEqualConstraints()
{
    equalMat = dmatrix::Zero(equalMatRows,URows);
    equalVec = dvector(equalMatRows);
    int rowIdx = 0;
    int colIdx = 0;
    ::dumpMatrix(equalMat, &ModelPredictiveControllerParam::equalMat, mpcParamDeque, rowIdx, colIdx, blockFlagVec);
    ::dumpVector(equalVec, &ModelPredictiveControllerParam::equalVec, mpcParamDeque, rowIdx, blockFlagVec);
}

void ModelPredictiveController::calcInequalConstraints()
{
    inequalMat = dmatrix::Zero(inequalMatRows,URows);
    inequalMinVec = dvector(inequalMatRows);
    inequalMaxVec = dvector(inequalMatRows);
    int rowIdx = 0;
    int colIdx = 0;
    ::dumpMatrix(inequalMat, &ModelPredictiveControllerParam::inequalMat, mpcParamDeque, rowIdx, colIdx, blockFlagVec);
    ::dumpVector(inequalMinVec, &ModelPredictiveControllerParam::inequalMinVec, mpcParamDeque, rowIdx, blockFlagVec);
    ::dumpVector(inequalMaxVec, &ModelPredictiveControllerParam::inequalMaxVec, mpcParamDeque, rowIdx, blockFlagVec);
}

void ModelPredictiveController::calcBoundVectors()
{
    minVec = dvector(URows);
    maxVec = dvector(URows);
    int rowIdx = 0;
    ::dumpVector(minVec, &ModelPredictiveControllerParam::minVec, mpcParamDeque, rowIdx, blockFlagVec);
    ::dumpVector(maxVec, &ModelPredictiveControllerParam::maxVec, mpcParamDeque, rowIdx, blockFlagVec);
}

void ModelPredictiveController::calcRefXVector()
{
    refX = dvector(stateDim*numWindows_);
    for(std::deque<ModelPredictiveControllerParam*>::iterator iter = mpcParamDeque.begin(); iter != mpcParamDeque.end(); ++iter){
        int idx = std::distance(mpcParamDeque.begin(), iter);
        refX.block(stateDim*idx,0, stateDim,1) = (*iter)->refStateVec;
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

MultiContactStabilizer::MultiContactStabilizer()
    : ModelPredictiveController()
{
    unitInputDim = 6;// 接触点ごとの入力次元
    stateDim = 6;
}

void MultiContactStabilizer::setupQP()
{
    qpInterface = QP(URows, equalMatRows, inequalMatRows);
}

int MultiContactStabilizer::execQP()
{
    static int count = 0;
    if(COUT){
        cout << psiCols << " x " << URows << endl;
        cout << "psi:" << endl << psiMat << endl << endl;
        cout << "phi:" << endl << phiMat << endl << endl;
        cout << "refX:" << endl << refX.transpose() << endl << endl;
        cout << "x0:" << endl << x0.transpose() << endl << endl;
        cout << "W1(error):" << endl << errorWeightMat << endl << endl;
        cout << "W2(input):" << endl << inputWeightMat << endl << endl;
    }
    dmatrix blockMat2 = blockMatInv.transpose();
    int ret  = qpInterface.execQP(U,
                                  // blockMatInv*(psiMat.transpose()*errorWeightMat*psiMat + inputWeightMat)*blockMat, blockMatInv*psiMat.transpose()*errorWeightMat*(phiMat*x0 - refX),
                                  // blockMatInv*(psiMat.transpose()*errorWeightMat*psiMat)*blockMat + blockMatInv*inputWeightMat*blockMat2, blockMatInv*psiMat.transpose()*errorWeightMat*(phiMat*x0 - refX),
                                  // blockMat.transpose()*(psiMat.transpose()*errorWeightMat*psiMat)*blockMat + blockMat2.transpose()*inputWeightMat*blockMat2, blockMat.transpose()*psiMat.transpose()*errorWeightMat*(phiMat*x0 - refX),
                                  blockMat.transpose()*(psiMat.transpose()*errorWeightMat*psiMat + inputWeightMat)*blockMat, blockMat.transpose()*psiMat.transpose()*errorWeightMat*(phiMat*x0 - refX),// 1_6 2_7
                                  // blockMatInv*(psiMat.transpose()*errorWeightMat*psiMat + inputWeightMat)*blockMat2, blockMatInv*psiMat.transpose()*errorWeightMat*(phiMat*x0 - refX),
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
    for(std::vector<ContactConstraintParam*>::iterator iter = ccParamVec.begin(); iter != ccParamVec.end(); ++iter){
        int idx = std::distance(ccParamVec.begin(), iter);
        Matrix33 R = (*iter)->R;
        Vector3 p = (*iter)->p;

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

void MultiContactStabilizerParam::calcMatrix(void (ContactConstraintParam::*calcFunc)(void))
{
    for(std::vector<ContactConstraintParam*>::iterator iter = ccParamVec.begin(); iter != ccParamVec.end(); ++iter){
        ((*iter)->*calcFunc)();
    }
}

void MultiContactStabilizerParam::calcVector(void (ContactConstraintParam::*calcFunc)(void))
{
    for(std::vector<ContactConstraintParam*>::iterator iter = ccParamVec.begin(); iter != ccParamVec.end(); ++iter){
        ((*iter)->*calcFunc)();
    }
}

void MultiContactStabilizerParam::calcEqualConstraints()
{
    int rowIdx = 0, colIdx = 0;
    equalMat = dmatrix::Zero(numEquals, inputDim);
    equalVec.resize(numEquals);

    // common constraint in different kinds of contact
    equalVec(rowIdx) = F(2);
    for(std::vector<ContactConstraintParam*>::iterator iter = ccParamVec.begin(); iter != ccParamVec.end(); ++iter){
        Matrix33 R = (*iter)->R;
        equalMat.block(rowIdx,colIdx, 1,3) = R.block(2,0,1,3);
        colIdx += unitInputDim;
    }
    ++rowIdx;

    // each contact constraint
    colIdx = 0;
    calcMatrix(&ContactConstraintParam::calcEqualMatrix);
    calcVector(&ContactConstraintParam::calcEqualVector);
    ::dumpMatrix(equalMat, &ContactConstraintParam::equalMat, ccParamVec, rowIdx, colIdx);
    ::dumpVector(equalVec, &ContactConstraintParam::equalVec, ccParamVec, rowIdx);
}

void MultiContactStabilizerParam::calcInequalConstraints()
{
    int rowIdx = 0, colIdx = 0;
    inequalMat = dmatrix::Zero(numInequals, inputDim);
    inequalMinVec.resize(numInequals);
    inequalMaxVec.resize(numInequals);
    calcMatrix(&ContactConstraintParam::calcInequalMatrix);
    calcVector(&ContactConstraintParam::calcInequalMinimumVector);
    calcVector(&ContactConstraintParam::calcInequalMaximumVector);
    ::dumpMatrix(inequalMat, &ContactConstraintParam::inequalMat, ccParamVec, rowIdx, colIdx);
    ::dumpVector(inequalMinVec, &ContactConstraintParam::inequalMinVec, ccParamVec, rowIdx);
    ::dumpVector(inequalMaxVec, &ContactConstraintParam::inequalMaxVec, ccParamVec, rowIdx);
}

void MultiContactStabilizerParam::calcBoundVectors()
{
    int rowIdx = 0;
    minVec.resize(inputDim);
    maxVec.resize(inputDim);
    calcVector(&ContactConstraintParam::calcMinimumVector);
    calcVector(&ContactConstraintParam::calcMaximumVector);
    ::dumpVector(minVec, &ContactConstraintParam::minVec, ccParamVec, rowIdx);
    ::dumpVector(maxVec, &ContactConstraintParam::maxVec, ccParamVec, rowIdx);
}

void MultiContactStabilizerParam::calcRefStateVector()
{
    const double m = controller->m;
    refStateVec = dvector(stateDim);
    refStateVec[0] = m*CM(0);
    refStateVec[1] = P(0);
    refStateVec[2] = m*CM(1);
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
    for(std::vector<ContactConstraintParam*>::iterator iter = ccParamVec.begin(); iter != ccParamVec.end(); ++iter){
        int idx = std::distance(ccParamVec.begin(), iter);
        inputWeightVec.block(unitInputDim*idx,0, unitInputDim,1) << inputForceWeight,inputForceWeight,inputForceWeight, inputMomentWeight,inputMomentWeight,inputMomentWeight;
    }
}

void MultiContactStabilizerParam::convertToMPCParam()
{
    inputDim = unitInputDim*ccParamVec.size();
    for(std::vector<ContactConstraintParam*>::iterator iter = ccParamVec.begin(); iter != ccParamVec.end(); ++iter){
        numEquals += (*iter)->numEquals;
        numInequals += (*iter)->numInequals;
    }

    calcEqualConstraints();
    calcInequalConstraints();
    calcBoundVectors();

    calcInputMatrix();
    calcSystemMatrix();

    calcRefStateVector();

    calcErrorWeightVector();
    calcInputWeightVector();
}
