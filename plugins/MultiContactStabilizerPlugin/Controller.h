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
    void calcX0Vector();

public:
    int numWindows;// N
    int stateDim;
    int psiCols, equalMatRows, inequalMatRows;
    bool isInitial;
    std::deque<ModelPreviewControllerParam> mpcParamDeque;
    dvector x0;

    ModelPreviewController();

    void calcAugmentedMatrix();
    virtual void setupQP() = 0;
    virtual void execQP() = 0;
};

class MultiContactStabilizerParam;
class MultiContactStabilizer : public ModelPreviewController
{
public:
    std::deque<MultiContactStabilizerParam> mcsParamDeque;
    double m,dt;
    int unitInputDim;// 接触点ごとの入力次元

    MultiContactStabilizer();

    void setupQP();
    void execQP();
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
