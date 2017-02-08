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

template <class ControllerClass, class ParamClass>
class ModelPredictiveControllerParam;

template <class ControllerClass, class ParamClass>
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

    ParamClass* copyMpcParam(ControllerClass* controller, ParamClass* fromMpcParam)
    {
        ParamClass* toMpcParam = new ParamClass(controller, fromMpcParam);
        return toMpcParam;
    }

private:
    int numWindows_;// N
    void calcPhiMatrix()
    {
        phiMat = dmatrix(stateDim*numWindows_,stateDim);
        dmatrix lastMat = dmatrix::Identity(stateDim,stateDim);
        for(typename std::deque<ParamClass*>::iterator iter = mpcParamDeque.begin(); iter != mpcParamDeque.end(); ++iter){
            int idx = std::distance(mpcParamDeque.begin(), iter);
            lastMat = (*iter)->systemMat * lastMat;
            phiMat.block(stateDim*idx,0, stateDim,stateDim) = lastMat;
        }
    }

    void calcPsiMatrix()
    {
        psiMat = dmatrix::Zero(stateDim*numWindows_,psiCols);
        int colIdx = 0;
        for(typename std::deque<ParamClass*>::iterator Biter = mpcParamDeque.begin(); Biter != mpcParamDeque.end(); ){// Biterは内側のforループでインクリメント
            dmatrix lastMat = (*Biter)->inputMat;
            int cols = lastMat.cols();
            int Bidx = std::distance(mpcParamDeque.begin(), Biter);// column index
            psiMat.block(stateDim*Bidx,colIdx, stateDim,cols) = lastMat;
            for(typename std::deque<ParamClass*>::iterator Aiter = ++Biter; Aiter != mpcParamDeque.end(); ++Aiter){
                int Aidx = std::distance(mpcParamDeque.begin(), Aiter);// row index
                lastMat = (*Aiter)->systemMat * lastMat;
                psiMat.block(stateDim*Aidx,colIdx, stateDim,cols) = lastMat;
            }
            colIdx += cols;
        }
    }

    void calcEqualConstraints()
    {
        equalMat = dmatrix::Zero(equalMatRows,URows);
        equalVec = dvector(equalMatRows);
        int rowIdx = 0;
        int colIdx = 0;
        hrp::dumpMatrix(equalMat, &ParamClass::equalMat, mpcParamDeque, rowIdx, colIdx, blockFlagVec);
        hrp::dumpVector(equalVec, &ParamClass::equalVec, mpcParamDeque, rowIdx, blockFlagVec);
    }

    void calcInequalConstraints()
    {
        inequalMat = dmatrix::Zero(inequalMatRows,URows);
        inequalMinVec = dvector(inequalMatRows);
        inequalMaxVec = dvector(inequalMatRows);
        int rowIdx = 0;
        int colIdx = 0;
        hrp::dumpMatrix(inequalMat, &ParamClass::inequalMat, mpcParamDeque, rowIdx, colIdx, blockFlagVec);
        hrp::dumpVector(inequalMinVec, &ParamClass::inequalMinVec, mpcParamDeque, rowIdx, blockFlagVec);
        hrp::dumpVector(inequalMaxVec, &ParamClass::inequalMaxVec, mpcParamDeque, rowIdx, blockFlagVec);
    }

    void calcBoundVectors()
    {
        minVec = dvector(URows);
        maxVec = dvector(URows);
        int rowIdx = 0;
        hrp::dumpVector(minVec, &ParamClass::minVec, mpcParamDeque, rowIdx, blockFlagVec);
        hrp::dumpVector(maxVec, &ParamClass::maxVec, mpcParamDeque, rowIdx, blockFlagVec);
    }

    void calcRefXVector()
    {
        refXVec = dvector(stateDim*numWindows_);
        for(typename std::deque<ParamClass*>::iterator iter = mpcParamDeque.begin(); iter != mpcParamDeque.end(); ++iter){
            int idx = std::distance(mpcParamDeque.begin(), iter);
            refXVec.block(stateDim*idx,0, stateDim,1) = (*iter)->refStateVec;
        }
    }

    void calcErrorWeightMatrix()
    {
        errorWeightMat = dmatrix::Zero(stateDim*numWindows_,stateDim*numWindows_);
        for(typename std::deque<ParamClass*>::iterator iter = mpcParamDeque.begin(); iter != mpcParamDeque.end(); ++iter){
            int idx = std::distance(mpcParamDeque.begin(), iter);
            errorWeightMat.block(stateDim*idx,stateDim*idx, stateDim,stateDim) = (*iter)->errorWeightVec.asDiagonal();
        }
    }

    void calcInputWeightMatrix()
    {
        inputWeightMat = dmatrix::Zero(psiCols,psiCols);
        int rowIdx = 0;
        int colIdx = 0;
        hrp::dumpMatrix(inputWeightMat, &ParamClass::inputWeightMat, mpcParamDeque, rowIdx, colIdx, blockFlagVec);
    }

    void calcBlockMatrix()
    {
        blockMat = dmatrix::Zero(psiCols,URows);
        blockMatInv = dmatrix::Zero(URows,psiCols);
        std::vector<bool>::iterator blockFlagIter = blockFlagVec.begin();
        typename std::deque<ParamClass*>::iterator colIter = mpcParamDeque.begin();
        int count = 1, rowIdx = 0, colIdx = 0;
        for(typename std::deque<ParamClass*>::iterator rowIter = mpcParamDeque.begin(); rowIter != mpcParamDeque.end(); ++rowIter, ++colIter){
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

public:
    double dt;
    int stateDim;
    int psiCols, URows, equalMatRows, inequalMatRows;
    bool isInitial;
    std::deque<ParamClass*> preMpcParamDeque;
    std::deque<ParamClass*> mpcParamDeque;
    dvector x0,u0;
    ParamClass* prevMpcParam;
    ControllerClass* parent;

    ModelPredictiveController()
    {
        isInitial = true;
        parent = NULL;
    }

    int processCycle(float &processedTime)
    {
        processedTime = 0;
        int ret = 0;
        clock_t st = clock(), et;
        double tmList[4] = {0,0,0,0};

        if(mpcParamDeque.size() == numWindows()){
            calcAugmentedMatrixAndSetupArgument();// phi,psi,W1,W2 U, x1->x0
            setupQP();

            et = clock();
            tmList[2] = (double) 1000*(et-st)/CLOCKS_PER_SEC;
            st = et;

            ret = execQP();

            et = clock();
            tmList[3] = (double) 1000*(et-st)/CLOCKS_PER_SEC;
            st = et;

            updateX0Vector();
            prevMpcParam = mpcParamDeque.front();
            mpcParamDeque.pop_front();
        }

        ParamClass* controllerParam =  preMpcParamDeque.front();
        if(preMpcParamDeque.size() > 1) preMpcParamDeque.pop_front();//cascadeではwait処理に変更

        et = clock();
        tmList[0] = (double) 1000*(et-st)/CLOCKS_PER_SEC;
        st = et;

        controllerParam->convertToMpcParam();
        mpcParamDeque.push_back(controllerParam);

        et = clock();
        tmList[1] = (double) 1000*(et-st)/CLOCKS_PER_SEC;
        st = et;

        std::cout << "  FK: " << tmList[0] << "[ms]  convert: " << tmList[1] << "[ms]  setup: " << tmList[2] << "[ms]  QP: " << tmList[3]  << "[ms]" << std::endl;

        processedTime = tmList[3];
        return ret;
    }

    ControllerClass* rootController()
    {
        if(parent != NULL) return parent->rootController();
        return (ControllerClass*) this;
    }

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

    void calcAugmentedMatrixAndSetupArgument()
    {
        psiCols = 0;
        equalMatRows = 0;
        inequalMatRows = 0;
        URows = 0;
        std::vector<bool>::iterator blockFlagIter = blockFlagVec.begin();
        for(typename std::deque<ParamClass*>::iterator iter = mpcParamDeque.begin(); iter != mpcParamDeque.end(); ++iter, ++blockFlagIter){
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

    void updateX0Vector()
    {
        ParamClass* mpcParam = mpcParamDeque[0];
        // U->u0
        // x0 = A0*x0 + B'0*u0
        u0 = U.block(0,0, mpcParam->inputMat.cols(),1);
        dmatrix x1 = mpcParam->systemMat*x0 + mpcParam->inputMat*u0;

        if(parent != NULL){
            ControllerClass* root = rootController();
            int stepNum = parent->dt/root->dt;// root->dtで割り切れる?
            // int nextPushIndex;
            // if(!parent->preMpcParamDeque.empty()) nextPushIndex = parent->preMpcParamDeque.back()->index() + stepNum;
            // else if(!parent->mpcParamDeque.empty()) nextPushIndex = parent->mpcParamDeque.back()->index() + stepNum;
            // else nextPushIndex = 0;

            int x0Index = mpcParamDeque[0]->index();
            int x1Index = mpcParamDeque[1]->index();// 0除算の可能性 最後以降は補間しないから大丈夫?
            std::cout << x0Index << ": " << x0.transpose() << std::endl;
            std::cout << x1Index << ": " << x1.transpose() << std::endl;
            std::cout << "root: " << root->dt << "  parent:" << parent->dt << std::endl;
            typename std::deque<ParamClass*>::iterator rootPreDequeIter = root->preMpcParamDeque.begin();
            ParamClass* targetMpcParam;
            while((*rootPreDequeIter)->index() < x0Index) rootPreDequeIter += stepNum;
            while((*rootPreDequeIter)->index() < x1Index){
                if(root == parent){
                    targetMpcParam = *rootPreDequeIter;
                }else{
                    parent->preMpcParamDeque.push_back(copyMpcParam(parent, *rootPreDequeIter));// pickup mpcParam from root mpc dtが変わるので行列は生成し直す
                    targetMpcParam = parent->preMpcParamDeque.back();
                }
                targetMpcParam->setRefStateVector(x0 + (x1-x0)*((*rootPreDequeIter)->index() - x0Index)/(double)(x1Index - x0Index)); // interpolate refX,U
                rootPreDequeIter += stepNum;
            }
        }

        x0 = x1;
    }

    virtual void setupQP() = 0;
    virtual int execQP() = 0;
    void pushAllPreMPCParamFromRoot()
    {
        ControllerClass* root = rootController();
        int stepNum = dt/root->dt;// root->dtで割り切れる?
        typename std::deque<ParamClass*>::iterator iter = root->preMpcParamDeque.begin();
        while(iter != root->preMpcParamDeque.end()){
            preMpcParamDeque.push_back(copyMpcParam((ControllerClass*)this ,*iter));
            for(int i=0; i<stepNum && iter !=root->preMpcParamDeque.end(); ++i, ++iter);
        }
    }
};

template <class ControllerClass, class ParamClass>
class ModelPredictiveControllerParam
{
private:

protected:
    ControllerClass* controller_;
    ControllerClass* controller(){return controller_;};
    int index_;
    bool isInitial_;

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
    dmatrix inputWeightMat;
    // dmatrix outputMat;

    ModelPredictiveControllerParam()
    {
        isInitial_ = true;
    };

    virtual void convertToMpcParam() = 0;
    int index(){return index_;}
    void setRefStateVector(const dvector& refStateVec_)
    {
        if(isInitial_){
            refStateVec = refStateVec_;
            isInitial_ = false;
        }
    }
};

}
