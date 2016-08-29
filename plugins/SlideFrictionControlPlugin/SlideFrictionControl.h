/**
   @author Kunio Kojima
*/
#pragma once

#include <UtilPlugin/UtilPlugin.h>
#include <MultiContactStabilizerPlugin/ModelPredictiveController.h>

namespace hrp {

class Test;
class SlideFrictionControlParam;
class SlideFrictionControl : public ModelPredictiveController<SlideFrictionControl, SlideFrictionControlParam>
{
    friend class OneSFCTest;

public:
    double m;
    double errorCMWeight,errorMomentumWeight,errorAngularMomentumWeight,errorYawAngularMomentumWeight;
    double inputForceWeight,inputMomentWeight,inputYawMomentWeight;

    SlideFrictionControl();

    void setupQP();
    int execQP();
};

class SlideFrictionControlParam : public ModelPredictiveControllerParam<SlideFrictionControl,SlideFrictionControlParam>
{
private:
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

    SlideFrictionControlParam(SlideFrictionControl* sfc, SlideFrictionControlParam* sfcParam)
        : ModelPredictiveControllerParam<SlideFrictionControl, SlideFrictionControlParam>()
    {
        if(sfcParam != NULL) *this = *sfcParam;
        controller_ = sfc;
        stateDim = controller()->stateDim;
    }

    SlideFrictionControlParam(int index, SlideFrictionControl* sfc, SlideFrictionControlParam* sfcParam)
        : ModelPredictiveControllerParam<SlideFrictionControl, SlideFrictionControlParam>()
    {
        *this = SlideFrictionControlParam(sfc, sfcParam);
        index_ = index;
    }


    SlideFrictionControlParam(int index, SlideFrictionControl* sfc)
        : ModelPredictiveControllerParam<SlideFrictionControl, SlideFrictionControlParam>()
    {
        *this = SlideFrictionControlParam(index, sfc, NULL);
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
