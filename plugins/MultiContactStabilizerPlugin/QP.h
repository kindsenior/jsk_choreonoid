/**
   @author Kunio Kojima
*/
#pragma once

#include <iostream>
#include <qpOASES.hpp>
#include <hrpUtil/EigenTypes.h>

#define COUT true

namespace hrp {

class QP : public qpOASES::QProblem
{
private:
    int valueDim;
    int equalDim;
    int inequalDim;

public:
    int maxCalcCount;
    
    QP(){};

    QP(int nV, int nE, int nIne)
        : qpOASES::QProblem(nV, nE+nIne)
    {
        valueDim = nV;
        equalDim = nE;
        inequalDim = nIne;
        // maxCalcCount = 10;
        maxCalcCount = 1000;
    };

    void execQP(dvector& U,
                const dmatrix& HMat, const dvector& gVec,
                const dmatrix& equalMat, const dvector& equalVec,
                const dmatrix& inequalMat, const dvector& inequalMinVec, const dvector& inequalMaxVec,
                const dvector& minVec, const dvector& maxVec);

};

}
