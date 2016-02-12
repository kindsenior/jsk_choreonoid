/**
   @author Kunio Kojima
*/
#include "test.h"

using namespace std;
using namespace hrp;

void Test::testAugmentedMatrix(MultiContactStabilizer* mcs)
{
    std::deque<ModelPreviewControllerParam> mpcParamDeque =  mcs->mpcParamDeque;
    int stateDim = mcs->stateDim;
    dvector U(mcs->psiCols);
    dvector xk(stateDim);
    xk = mcs->x0;
    dvector x0(stateDim);
    x0 = mcs->x0;
    int colIdx = 0;
    for(std::deque<ModelPreviewControllerParam>::iterator iter = mpcParamDeque.begin(); iter != mpcParamDeque.end(); ++iter){
        int idx = std::distance(mpcParamDeque.begin(), iter);
        dmatrix inputMat = (*iter).inputMat;
        int inputDim = inputMat.cols();

        dvector uk(inputDim);
        for(int i=0; i<inputDim; ++i){
            uk(i) = i;
        }

        xk = (*iter).systemMat*xk + inputMat*uk;
        U.block(colIdx,0, inputDim,1) = uk;
        colIdx += inputDim;
    }

    cout << endl << endl;
    cout << "numWindows: " << mcs->numWindows << endl;
    cout << "stateDim: " << stateDim << endl;
    cout << "x0:" << endl << x0.transpose() << endl << endl;
    cout << "phiMat (" << mcs->phiMat.rows() << "x" << mcs->phiMat.cols() << "):" << endl << mcs->phiMat << endl << endl;
    cout << "psiMat (" << mcs->psiMat.rows() << "x" << mcs->psiMat.cols() << "):" << endl << mcs->psiMat << endl << endl;
    // cout << (mcs->phiMat*x0).transpose() << endl << endl;
    // cout << (mcs->psiMat*U).transpose() << endl << endl;
    dvector xk_(stateDim);
    xk_ = (mcs->phiMat*x0 + mcs->psiMat*U).block(stateDim*(mpcParamDeque.size()-1),0, stateDim,1);
    cout << "recurrence: " << xk.transpose() << endl << "matrix:     " << xk_.transpose() << endl << endl;
}
