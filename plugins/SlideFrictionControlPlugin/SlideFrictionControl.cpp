/**
   @author Kunio Kojima
*/
#include "SlideFrictionControl.h"

using namespace hrp;
using namespace std;

SlideFrictionControl::SlideFrictionControl()
    : ModelPredictiveController<SlideFrictionControl, SlideFrictionControlParam>()
{
    stateDim = 7;//changed add Lz
}

void SlideFrictionControl::setupQP()
{
    qpInterface = QP(URows, equalMatRows, inequalMatRows);

    // blockMatInv*(psiMat.transpose()*errorWeightMat*psiMat + inputWeightMat)*blockMat, blockMatInv*psiMat.transpose()*errorWeightMat*(phiMat*x0 - refXVec),
    // blockMatInv*(psiMat.transpose()*errorWeightMat*psiMat)*blockMat + blockMatInv*inputWeightMat*blockMat2, blockMatInv*psiMat.transpose()*errorWeightMat*(phiMat*x0 - refXVec),
    qpInterface.HMat = blockMat.transpose()*(psiMat.transpose()*errorWeightMat*psiMat)*blockMat + blockMatInv*inputWeightMat*blockMatInv.transpose();
    qpInterface.gVec = blockMat.transpose()*psiMat.transpose()*errorWeightMat*(phiMat*x0 - refXVec);
    // blockMat.transpose()*(psiMat.transpose()*errorWeightMat*psiMat + inputWeightMat)*blockMat, blockMat.transpose()*psiMat.transpose()*errorWeightMat*(phiMat*x0 - refXVec),// 1_6 2_7
    // blockMatInv*(psiMat.transpose()*errorWeightMat*psiMat + inputWeightMat)*blockMat2, blockMatInv*psiMat.transpose()*errorWeightMat*(phiMat*x0 - refXVec),

    qpInterface.equalMat = equalMat; qpInterface.equalVec = equalVec;
    qpInterface.inequalMat = inequalMat; qpInterface.inequalMinVec = inequalMinVec; qpInterface.inequalMaxVec = inequalMaxVec;
    qpInterface.minVec = minVec; qpInterface.maxVec = maxVec;
}

int SlideFrictionControl::execQP()
{
    static int count = 0;
    // cout << setprecision(3);
    if(COUT){
        cout << psiCols << " x " << URows << endl;
        // cout << setw(3)  << "psi:" << endl << psiMat << endl << endl;
        cout << "psi:" << endl << psiMat << endl << endl;
        cout << "phi:" << endl << phiMat << endl << endl;
        cout << "refXVec:" << endl << refXVec.transpose() << endl << endl;
        cout << "x0:" << endl << x0.transpose() << endl << endl;
        cout << "W1(error):" << endl << errorWeightMat << endl << endl;
        cout << "W2(input):" << endl << inputWeightMat << endl << endl;
        cout << "block:" << endl << blockMat << endl << endl;
        cout << "blockInv:" << endl << blockMatInv << endl << endl;
    }
    // cout << setprecision(6);// default 6
    int ret  = qpInterface.execQP(U);

    ++count;
    return ret;
}

void SlideFrictionControlParam::calcSystemMatrix()
{
    double dt = controller()->dt;
    systemMat = dmatrix::Identity(stateDim,stateDim);
    const double m = controller()->m;
    double Fz = F(2);
    systemMat(0,1) = dt;
    systemMat(2,3) = dt;
    systemMat(4,2) = -dt*Fz/m;
    systemMat(5,0) = dt*Fz/m;
    //-rx*fy-(-ry*fx)=0
}

void SlideFrictionControlParam::calcInputMatrix()
{
    double dt = controller()->dt;
    inputMat = dmatrix::Zero(stateDim,inputDim);
    int inputForceDim = 6;

    int colIdx = 0;
    for(std::vector<ContactConstraintParam*>::iterator iter = ccParamVec.begin(); iter != ccParamVec.end(); ++iter){
        Matrix33 R = (*iter)->R;
        Vector3 p = (*iter)->p;

        dmatrix R2 = dmatrix::Zero(inputForceDim,inputForceDim);
        R2.block(0,0,3,3) = R;
        R2.block(3,3,3,3) = R;
        dmatrix unitInputMat = dmatrix(stateDim,inputForceDim);// B_k^i
        double T2_2 = dt*dt/2;
        double beta = dt*(p(2)-CM(2));
        dmatrix tmpMat(stateDim,inputForceDim);// statedim x inputForceDim
        tmpMat <<
            T2_2,0, 0,       0, 0,0,
            dt,  0, 0,       0, 0,0,
            0,T2_2, 0,       0, 0,0,
            0,dt,   0,       0, 0,0,
            0,-beta,dt*p(1), dt,0,0,
            beta,0,-dt*p(0), 0,dt,0,
            // 0,0,0,           0,0,dt;//changed add Lz
            -dt*(p(1)-CM(1)),dt*(p(0)-CM(0)),0, 0,0,dt;//changed add Lz, (rx-xg)*fy - (ry-yg)*fx. (xg,yg) is reference
        inputMat.block(0,colIdx, stateDim,(*iter)->inputDim) = tmpMat*R2*(*iter)->inputForceConvertMat;// product input conversion matrix (distribution->6dof force)
        colIdx += (*iter)->inputDim;
    }
}

void SlideFrictionControlParam::calcMatrix(void (ContactConstraintParam::*calcFunc)(void))
{
    for(std::vector<ContactConstraintParam*>::iterator iter = ccParamVec.begin(); iter != ccParamVec.end(); ++iter){
        ((*iter)->*calcFunc)();
    }
}

void SlideFrictionControlParam::calcVector(void (ContactConstraintParam::*calcFunc)(void))
{
    for(std::vector<ContactConstraintParam*>::iterator iter = ccParamVec.begin(); iter != ccParamVec.end(); ++iter){
        ((*iter)->*calcFunc)();
    }
}

void SlideFrictionControlParam::calcEqualConstraints()
{
    int rowIdx = 0, colIdx = 0;
    equalMat = dmatrix::Zero(numEquals, inputDim);
    equalVec.resize(numEquals);

    // common constraint in different kinds of contact
    // sum of Fz
    equalVec(rowIdx) = F(2);
    for(std::vector<ContactConstraintParam*>::iterator iter = ccParamVec.begin(); iter != ccParamVec.end(); ++iter){
        Matrix33 R = (*iter)->R;
        equalMat.block(rowIdx,colIdx, 1,(*iter)->inputDim) = R.block(2,0,1,3) * (*iter)->inputForceConvertMat.block(0,0, 3,(*iter)->inputDim);//fz product input convertion matrix (fx,fy,fzi)
        colIdx += (*iter)->inputDim;
    }
    ++rowIdx;

    // Lz = sum of -(py-yg)*fx + (px-xg)*fy + nz
    int inputForceDim = 6;
    colIdx = 0;
    equalVec(rowIdx) = 0;// Lz = 0
    for(auto ccParam : ccParamVec){
        Vector3 p = ccParam->p;
        Matrix33 R = ccParam->R;
        dmatrix crossRot = dmatrix(1,2);
        crossRot << -(p(1)-CM(1)),(p(0)-CM(0));
        equalMat.block(rowIdx,colIdx, 1,ccParam->inputDim)
            = crossRot * R.block(0,0,2,3) * ccParam->inputForceConvertMat.block(0,0, 3,ccParam->inputDim)//fx,fy cordinate's input convertion matrix (fx,fy)
            + R.block(2,0,1,3) * ccParam->inputForceConvertMat.block(3,0, 3,ccParam->inputDim);//nz cordinate's input convertion matrix (nx(fzi),ny(fzi),nz)
        colIdx += ccParam->inputDim;
    }
    ++rowIdx;

    // each contact constraint
    colIdx = 0;
    calcMatrix(&ContactConstraintParam::calcEqualMatrix);
    calcVector(&ContactConstraintParam::calcEqualVector);
    hrp::dumpMatrix(equalMat, &ContactConstraintParam::equalMat, ccParamVec, rowIdx, colIdx);
    hrp::dumpVector(equalVec, &ContactConstraintParam::equalVec, ccParamVec, rowIdx);
}

void SlideFrictionControlParam::calcInequalConstraints()
{
    int rowIdx = 0, colIdx = 0;
    inequalMat = dmatrix::Zero(numInequals, inputDim);
    inequalMinVec.resize(numInequals);
    inequalMaxVec.resize(numInequals);
    calcMatrix(&ContactConstraintParam::calcInequalMatrix);
    calcVector(&ContactConstraintParam::calcInequalMinimumVector);
    calcVector(&ContactConstraintParam::calcInequalMaximumVector);
    hrp::dumpMatrix(inequalMat, &ContactConstraintParam::inequalMat, ccParamVec, rowIdx, colIdx);
    hrp::dumpVector(inequalMinVec, &ContactConstraintParam::inequalMinVec, ccParamVec, rowIdx);
    hrp::dumpVector(inequalMaxVec, &ContactConstraintParam::inequalMaxVec, ccParamVec, rowIdx);
}

void SlideFrictionControlParam::calcBoundVectors()
{
    int rowIdx = 0;
    minVec.resize(inputDim);
    maxVec.resize(inputDim);
    calcVector(&ContactConstraintParam::calcMinimumVector);
    calcVector(&ContactConstraintParam::calcMaximumVector);
    hrp::dumpVector(minVec, &ContactConstraintParam::minVec, ccParamVec, rowIdx);
    hrp::dumpVector(maxVec, &ContactConstraintParam::maxVec, ccParamVec, rowIdx);
}

void SlideFrictionControlParam::calcRefStateVector()
{
    const double m = controller()->m;
    dvector tmpVec(stateDim);
    tmpVec << m*CM(0),P(0), m*CM(1),P(1), L(0),L(1),L(2);//Lz
    setRefStateVector(tmpVec);
}

void SlideFrictionControlParam::calcErrorWeightVector()
{
    errorWeightVec = dvector(stateDim);
    double errorCMWeight = controller()->errorCMWeight, errorMomentumWeight = controller()->errorMomentumWeight,
        errorAngularMomentumWeight = controller()->errorAngularMomentumWeight, errorYawAngularMomentumWeight = controller()->errorYawAngularMomentumWeight;
    errorWeightVec << errorCMWeight,errorMomentumWeight, errorCMWeight,errorMomentumWeight, errorAngularMomentumWeight,errorAngularMomentumWeight,errorYawAngularMomentumWeight;// Lz
}

void SlideFrictionControlParam::calcInputWeightMatrix()
{
    int rowIdx = 0, colIdx = 0;
    inputWeightMat = dmatrix::Zero(inputDim,inputDim);
    double inputForceWeight = controller()->inputForceWeight, inputMomentWeight = controller()->inputMomentWeight, inputYawMomentWeight = controller()->inputYawMomentWeight;
    for(std::vector<ContactConstraintParam*>::iterator iter = ccParamVec.begin(); iter != ccParamVec.end(); ++iter){
        int inputDim = (*iter)->inputDim;
        dvector tmpVec(6);
        tmpVec << inputForceWeight,inputForceWeight,inputForceWeight, inputMomentWeight,inputMomentWeight,inputYawMomentWeight;// fx,fy,fz, nx,ny,nz
        inputWeightMat.block(rowIdx,colIdx, inputDim,inputDim) = (*iter)->inputForceConvertMat.transpose()*tmpVec.asDiagonal()*(*iter)->inputForceConvertMat;
        rowIdx += inputDim;
        colIdx += inputDim;
    }
}

void SlideFrictionControlParam::convertToMpcParam()
{
    inputDim = 0;
    // numEquals = 1;//Fzの合計
    numEquals = 2;//Fzの合計 + Lz=0
    numInequals = 0;
    for(std::vector<ContactConstraintParam*>::iterator iter = ccParamVec.begin(); iter != ccParamVec.end(); ++iter){
        numEquals += (*iter)->numEquals;
        numInequals += (*iter)->numInequals;
        inputDim += (*iter)->inputDim;
    }

    calcEqualConstraints();
    calcInequalConstraints();
    calcBoundVectors();

    calcInputMatrix();
    calcSystemMatrix();

    calcRefStateVector();

    calcErrorWeightVector();
    calcInputWeightMatrix();
}
