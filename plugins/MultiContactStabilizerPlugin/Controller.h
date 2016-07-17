/**
   @author Kunio Kojima
*/
#pragma once

#include <iostream>
#include <sstream>
#include <fstream>
#include <map>
#include <vector>
#include <deque>

#include <hrpUtil/EigenTypes.h>

#include "ContactConstraint.h"
#include "QP.h"

namespace hrp {

class ModelPredictiveControllerParam
{
public:
    int stateDim;
    int inputDim;

    dmatrix systemMat;// A
    dmatrix inputMat;// B
    dmatrix equalMat;
    dvector equalVec;
    dmatrix inequalMat;
    dvector inequalMinVec;
    dvector inequalMaxVec;
    dvector minVec;
    dvector maxVec;
    dvector refStateVec;
    dvector errorWeightVec;
    dvector inputWeightVec;
    // dmatrix outputMat;

    ModelPredictiveControllerParam(){};
};

class ModelPredictiveController
{
protected:
    QP qpInterface;

    std::vector<int> blockVec;

    dmatrix phiMat;
    dmatrix psiMat;
    dmatrix equalMat;
    dvector equalVec;
    dmatrix inequalMat;
    dvector inequalMinVec;
    dvector inequalMaxVec;
    dvector minVec;
    dvector maxVec;
    dvector refX;
    dmatrix errorWeightMat;// W1
    dmatrix inputWeightMat;// W2
    dvector U;

private:
    int numWindows_;// N
    void calcPhiMatrix();
    void calcPsiMatrix();
    void calcEqualConstraints();
    void calcInequalConstraints();
    void calcBoundVectors();
    void calcRefXVector();
    void calcErrorWeightMatrix();
    void calcInputWeightMatrix();

public:
    int stateDim;
    int psiCols, equalMatRows, inequalMatRows;
    bool isInitial;
    std::deque<ModelPredictiveControllerParam*> mpcParamDeque;
    dvector x0;

    ModelPredictiveController();

    int numWindows(){return numWindows_;}
    void setBlockVector(std::vector<int> vec){blockVec = vec; numWindows_ = blockVec.size();}
    void calcAugmentedMatrix();
    void updateX0Vector();
    virtual void setupQP() = 0;
    virtual int execQP() = 0;
};

class Test;
class MultiContactStabilizerParam;
class MultiContactStabilizer : public ModelPredictiveController
{
    friend class Test;
public:
    double m,dt;
    int unitInputDim;// 接触点ごとの入力次元
    double errorCMWeight,errorMomentumWeight,errorAngularMomentumWeight;
    double inputForceWeight,inputMomentWeight;

    MultiContactStabilizer();

    void setupQP();
    int execQP();
};

class MultiContactStabilizerParam : public ModelPredictiveControllerParam
{
private:
    MultiContactStabilizer* controller;
    int unitInputDim;
    double dt;

    void calcMatrix(void (ContactConstraintParam::*func)(void));
    void calcVector(void (ContactConstraintParam::*func)(void));

public:
    std::vector<ContactConstraintParam*> ccParamVec;
    int numEquals;
    int numInequals;
    Vector3 CM;
    Vector3 P;
    Vector3 L;
    Vector3 F;

    MultiContactStabilizerParam(MultiContactStabilizer* mcs)
        : ModelPredictiveControllerParam()
    {
        controller = mcs;
        stateDim = controller->stateDim;
        unitInputDim = controller->unitInputDim;
        dt = controller->dt;
        numEquals = 1;//Fzの合計
        numInequals = 0;
    }

    void calcInputMatrix();
    void calcSystemMatrix();
    void calcEqualConstraints();
    void calcInequalConstraints();
    void calcBoundVectors();
    void calcRefStateVector();
    void calcErrorWeightVector();
    void calcInputWeightVector();
    void convertToMPCParam();
};

}
