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

template <typename containerClass, typename paramClass>
inline void dumpMatrix(dmatrix& retMat, dmatrix paramClass::*inMat, containerClass& container, int rowIdx, int colIdx, std::vector<bool> flagVec)
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
template <typename containerClass, typename paramClass>
inline void dumpMatrix(dmatrix& retMat, dmatrix paramClass::*inMat, containerClass& container, int rowIdx, int colIdx)
{
    std::vector<bool> flagVec(container.size(), true);
    hrp::dumpMatrix(retMat, inMat, container, rowIdx,colIdx, flagVec);
}

template <typename containerClass, typename paramClass>
inline void dumpVector(dvector& retVec, dvector paramClass::*inVec, containerClass& container, int rowIdx, std::vector<bool> flagVec)
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
template <typename containerClass, typename paramClass>
inline void dumpVector(dvector& retVec, dvector paramClass::*inVec, containerClass& container, int rowIdx)
{
    std::vector<bool> flagVec(container.size(), true);
    hrp::dumpVector(retVec, inVec, container, rowIdx, flagVec);
}

class ModelPredictiveControllerParam;
class ModelPredictiveController
{
protected:
    QP qpInterface;

    std::vector<int> blockVec;
    std::vector<bool> blockFlagVec;

    dmatrix phiMat;
    dmatrix psiMat;
    dmatrix equalMat;
    dvector equalVec;
    dmatrix inequalMat;
    dvector inequalMinVec;
    dvector inequalMaxVec;
    dvector minVec;
    dvector maxVec;
    dvector refXVec;
    dmatrix errorWeightMat;// W1
    dmatrix inputWeightMat;// W2
    dvector U;
    dmatrix blockMat;
    dmatrix blockMatInv;

    dmatrix HMat;
    dvector gVec;

    ModelPredictiveController* rootController();
    virtual ModelPredictiveControllerParam* copyMpcParam(ModelPredictiveController* container, ModelPredictiveControllerParam* fromMpcParam) = 0;

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
    void calcBlockMatrix();

public:
    double dt;
    int stateDim;
    int psiCols, URows, equalMatRows, inequalMatRows;
    bool isInitial;
    std::deque<ModelPredictiveControllerParam*> preMpcParamDeque;
    std::deque<ModelPredictiveControllerParam*> mpcParamDeque;
    dvector x0;
    ModelPredictiveController* parent;

    ModelPredictiveController();

    int numWindows(){return numWindows_;}
    void setBlockVector(std::vector<int> vec)
    {
        blockVec = vec;
        numWindows_ = std::accumulate(vec.begin(), vec.end(), 0);
        blockFlagVec.clear();
        for(std::vector<int>::iterator iter = blockVec.begin(); iter != blockVec.end(); ++iter){
            blockFlagVec.push_back(true);
            for(int i=(*iter)-1; i>0; --i){
                blockFlagVec.push_back(false);
            }
        }
    }
    void calcAugmentedMatrix();
    void updateX0Vector();
    virtual void setupQP() = 0;
    virtual int execQP() = 0;
    void pushAllPreMPCParamFromRoot();
};

class ModelPredictiveControllerParam
{
private:
    ModelPredictiveController* controller(){return controller_;};

protected:
    ModelPredictiveController* controller_;
    int index_;

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

    ModelPredictiveControllerParam()
    {
    };

    virtual void convertToMpcParam() = 0;
    int index(){return index_;}
};

}
