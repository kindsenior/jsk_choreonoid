/**
   @author Kunio Kojima
*/
#include "QP.h"

using namespace hrp;
using namespace qpOASES;
using namespace std;

int QP::execQP(dvector& U)
{
    static int count = 0;
    if(COUT){
        cout << "H:" << endl << HMat << endl << endl;
        cout << "g:" << endl << gVec.transpose() << endl << endl;
        cout << "eqMat:" << endl << equalMat << endl << endl;
        cout << "eqVec:" << endl << equalVec.transpose() << endl << endl;
        cout << "ineqMat:" << endl << inequalMat << endl << endl;
        cout << "ineqMinVec:" << endl << inequalMinVec.transpose() << endl << endl;
        cout << "ineqMaxVec:" << endl << inequalMaxVec.transpose() << endl << endl;
        cout << "lb:" << endl << minVec.transpose() << endl << endl;
        cout << "ub:" << endl << maxVec.transpose() << endl << endl;
        cout << "U(before):" << endl << U.transpose() << endl << endl;
    }

    int Arows = inequalDim+equalDim;
    real_t H[valueDim*valueDim];
    real_t A[Arows*valueDim];
    real_t g[valueDim];
    real_t lb[valueDim];
    real_t ub[valueDim];
    real_t lbA[Arows];
    real_t ubA[Arows];
    for(int i=0; i < valueDim; ++i){
        for(int j=0; j < valueDim; ++j){
            H[i*valueDim+j] = HMat(i,j);
        }
        g[i] = gVec(i);
        lb[i] = minVec(i);
        ub[i] = maxVec(i);
    }
    // 不等式条件
    for(int i=0; i < inequalDim; ++i){
        for(int j=0; j < valueDim; ++j){
            A[i*valueDim+j] = inequalMat(i,j);
        }
        lbA[i] = inequalMinVec(i);
        ubA[i] = inequalMaxVec(i);
    }
    // 等式条件を不等式条件として代入
    int offset = inequalDim*valueDim;
    for(int i=0; i < equalDim; ++i){
        for(int j=0; j < valueDim; ++j){
            A[offset+i*valueDim+j] = equalMat(i,j);
        }
        lbA[inequalDim+i] = equalVec[i];
        ubA[inequalDim+i] = equalVec[i];
    }

    Options options;
    // options.enableFarBounds = BT_TRUE;//added
    //options.enableFlippingBounds = BT_FALSE;//default comment
    // options.initialStatusBounds = ST_INACTIVE;
    // options.numRefinementSteps = 1;
    // options.enableCholeskyRefactorisation = 1;
    options.printLevel = PL_LOW;// default comment
    // options.printLevel = PL_NONE;// default comment
    setOptions( options );

    returnValue ret = init(H,g,A,lb,ub,lbA,ubA, maxCalcCount);
    real_t xOpt[valueDim];
    getPrimalSolution(xOpt);
    for(int i=0; i < valueDim; ++i){
        U(i) = xOpt[i];
    }
    if(COUT){
        cout << "U(after):" << endl << U.transpose() << endl << endl;
        cout << "objVal: " << getObjVal() << endl;
    }
    // printf( "\n xOpt = [ %e, %e ]; objVal = %e\n\n", xOpt[0],xOpt[1],example.getObjVal() );
    ++count;

    return getSimpleStatus(ret);
};
