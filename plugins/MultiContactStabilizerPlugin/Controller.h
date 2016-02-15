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

#include "QP.h"

namespace hrp {

typedef struct CONTACTCONSTRAINTPARAM
{
    int contactState;
    int numEquals;
    int numInequals;
    std::string linkName;
    Vector3 p;
    Matrix33 R;
    std::vector<Vector3> edgeVec;
    double mu;
}ContactConstraintParam;

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
    void calcPhiMatrix();
    void calcPsiMatrix();
    void calcEqualConstraints();
    void calcInequalConstraints();
    void calcBoundVectors();
    void calcRefXVector();
    void calcErrorWeightMatrix();
    void calcInputWeightMatrix();

public:
    int numWindows;// N
    int stateDim;
    int psiCols, equalMatRows, inequalMatRows;
    bool isInitial;
    std::deque<ModelPredictiveControllerParam*> mpcParamDeque;
    dvector x0;

    ModelPredictiveController();

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

public:
    std::vector<ContactConstraintParam> ccParamVec;
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
    void calcEqualMatrix();
    void calcEqualVector();
    void calcInequalMatrix();
    void calcMinimumVector();
    void calcRefStateVector();
    void calcErrorWeightVector();
    void calcInputWeightVector();
    void convertToMPCParam();
};

}
