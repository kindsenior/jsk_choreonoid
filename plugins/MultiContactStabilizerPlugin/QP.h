/**
   @author Kunio Kojima
*/
#pragma once

#include <iostream>
#include <qpOASES.hpp>
#include <hrpUtil/EigenTypes.h>

#define COUT false

namespace hrp {

class QP : public qpOASES::QProblem
{
private:
    int valueDim;
    int equalDim;
    int inequalDim;

public:
    int maxCalcCount;

    dmatrix HMat;
    dvector gVec;
    dmatrix equalMat;
    dvector equalVec;
    dmatrix inequalMat;
    dvector inequalMinVec, inequalMaxVec;
    dvector minVec, maxVec;

    QP(){};

    QP(int nV, int nE, int nIne)
        : qpOASES::QProblem(nV, nE+nIne)
    {
        valueDim = nV;
        equalDim = nE;
        inequalDim = nIne;
        // maxCalcCount = 10;// fail
        // maxCalcCount = 100;// fail
        // maxCalcCount = 500;// success
        // maxCalcCount = 1000;
        // maxCalcCount = 10000;
        maxCalcCount = 20000;
        // maxCalcCount = 100000;
    };

    int execQP(dvector& U);
};

}
