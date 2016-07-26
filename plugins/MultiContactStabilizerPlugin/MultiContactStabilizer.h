/**
   @author Kunio Kojima
*/
#pragma once

#include "ModelPredictiveController.h"

namespace hrp {

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

    int processCycle(float &processedTime);
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
    }

    void calcInputMatrix();
    void calcSystemMatrix();
    void calcEqualConstraints();
    void calcInequalConstraints();
    void calcBoundVectors();
    void calcRefStateVector();
    void calcErrorWeightVector();
    void calcInputWeightVector();
    void convertToMpcParam();
};

}
