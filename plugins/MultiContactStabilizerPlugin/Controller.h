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

typedef struct MODELPREVIEWCONTROLLERPARAM
{
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
}ModelPreviewControllerParam;

class ModelPreviewController
{
public:
    int numWindows;// N
    int stateDim;
    std::deque<ModelPreviewControllerParam> mpcParamDeque;
    dvector x0;
    dmatrix psiMat;
    dmatrix phiMat;

    ModelPreviewController();
};

class MultiContactStabilizerParam;
class MultiContactStabilizer : public ModelPreviewController
{
public:
    std::deque<MultiContactStabilizerParam> mcsParamDeque;
    double m,dt;
    int unitInputDim;// 接触点ごとの入力次元

    MultiContactStabilizer();
};

class MultiContactStabilizerParam
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

    MultiContactStabilizerParam(MultiContactStabilizer* mcs){
        controller = mcs;
        unitInputDim = controller->unitInputDim;
        dt = controller->dt;
        numEquals = 0;
        numInequals = 0;
    }

    void calcInputMatrix(dmatrix& inputMat);
    void calcSystemMatrix(dmatrix& systemMat);
    void calcEqualMatrix(dmatrix& equalMat);
    void calcEqualVector(dvector& equalVec);
    void calcInequalMatrix(dmatrix& inequalMat);
    void calcMinimumVector(dvector& minVec);
    void calcRefStateVector(dvector& refStateVec);
    void calcErrorWeightVector(dvector& errorWeightVec);
    void calcInputWeightVector(dvector& inputWeightVec);
    void convertToMPCParam(ModelPreviewControllerParam& mpcParam);
};

}
