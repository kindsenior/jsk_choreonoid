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
protected:
    ModelPredictiveControllerParam* copyMpcParam(ModelPredictiveController* controller, ModelPredictiveControllerParam* fromMpcParam);

public:
    double m;
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
    int unitInputDim;

    void calcMatrix(void (ContactConstraintParam::*func)(void));
    void calcVector(void (ContactConstraintParam::*func)(void));

    MultiContactStabilizer* controller(){return (MultiContactStabilizer*)controller_;}

public:
    std::vector<ContactConstraintParam*> ccParamVec;
    int numEquals;
    int numInequals;
    Vector3 CM;
    Vector3 P;
    Vector3 L;
    Vector3 F;

    MultiContactStabilizerParam(MultiContactStabilizer* mcs, MultiContactStabilizerParam* mcsParam)
        : ModelPredictiveControllerParam()
    {
        if(mcsParam != NULL) *this = *mcsParam;
        controller_ = mcs;
        stateDim = controller()->stateDim;
        unitInputDim = controller()->unitInputDim;
    }

    MultiContactStabilizerParam(int index, MultiContactStabilizer* mcs, MultiContactStabilizerParam* mcsParam)
        : ModelPredictiveControllerParam()
    {
        *this = MultiContactStabilizerParam(mcs, mcsParam);
        index_ = index;
    }


    MultiContactStabilizerParam(int index, MultiContactStabilizer* mcs)
        : ModelPredictiveControllerParam()
    {
        *this = MultiContactStabilizerParam(index, mcs, NULL);
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
