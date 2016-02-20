/**
   @author Kunio Kojima
*/
#include "test.h"

using namespace std;
using namespace hrp;

void Test::testAugmentedMatrix()
{
    cout << endl << "augmented matrix test" << endl;

    mcs->calcAugmentedMatrix();

    std::deque<ModelPredictiveControllerParam*> mpcParamDeque =  mcs->mpcParamDeque;
    int stateDim = mcs->stateDim;
    dvector U(mcs->psiCols);
    dvector xk(stateDim);
    xk = mcs->x0;
    dvector x0(stateDim);
    x0 = mcs->x0;
    int colIdx = 0;
    for(std::deque<ModelPredictiveControllerParam*>::iterator iter = mpcParamDeque.begin(); iter != mpcParamDeque.end(); ++iter){
        int idx = std::distance(mpcParamDeque.begin(), iter);
        dmatrix inputMat = (*iter)->inputMat;
        int inputDim = inputMat.cols();

        dvector uk(inputDim);
        for(int i=0; i<inputDim; ++i){
            uk(i) = i;
        }

        xk = (*iter)->systemMat*xk + inputMat*uk;
        U.block(colIdx,0, inputDim,1) = uk;
        colIdx += inputDim;
    }

    cout << "numWindows: " << mcs->numWindows << endl;
    cout << "stateDim: " << stateDim << endl;
    cout << "x0:" << x0.transpose() << endl << endl;
    // cout << "phiMat (" << mcs->phiMat.rows() << "x" << mcs->phiMat.cols() << "):" << endl << mcs->phiMat << endl << endl;
    // cout << "psiMat (" << mcs->psiMat.rows() << "x" << mcs->psiMat.cols() << "):" << endl << mcs->psiMat << endl << endl;
    // cout << (mcs->phiMat*x0).transpose() << endl << endl;
    // cout << (mcs->psiMat*U).transpose() << endl << endl;
    dvector xk_(stateDim);
    xk_ = (mcs->phiMat*x0 + mcs->psiMat*U).block(stateDim*(mpcParamDeque.size()-1),0, stateDim,1);
    cout << "recurrence: " << xk.transpose() << endl << "matrix:     " << xk_.transpose() << endl << endl;
}

void Test::processCycle(int i, std::vector<ContactConstraintParam*>& ccParamVec)
{
    cout << endl << "##############################" << endl << "turn:" << i << endl;

    clock_t st = clock();
    double tmList[4] = {0,0,0,0};

    MultiContactStabilizerParam* mcsParam = new MultiContactStabilizerParam(mcs);
    for(std::vector<ContactConstraintParam*>::iterator iter = ccParamVec.begin(); iter != ccParamVec.end(); ++iter){
        (*iter)->p = pMap[(*iter)->linkName];
        (*iter)->R = RMap[(*iter)->linkName];
    }
    mcsParam->ccParamVec = ccParamVec;

    static Vector3 g;
    g << 0,0,9.8;
    {
        CM += dCM;
        mcsParam->CM = CM;
        mcsParam->P = dCM*mcs->m;
        mcsParam->L << 0,0,0;
        mcsParam->F = mcs->m*dCM/mcs->dt + mcs->m*g;

        mOfs0 << i*mcs->dt << " " << CM.transpose() <<  " " << P.transpose() << " " << L.transpose() << endl;
        mOfs2 << i*mcs->dt << " "<< pMap["lleg"].transpose() << " " << pMap["rleg"].transpose() << endl;
    }

    clock_t et = clock();
    tmList[0] = (double) 1000*(et-st)/CLOCKS_PER_SEC;
    st = et;

    mcsParam->convertToMPCParam();
    mcs->mpcParamDeque.push_back(mcsParam);

    et = clock();
    tmList[1] = (double) 1000*(et-st)/CLOCKS_PER_SEC;
    st = et;

    if(mcs->mpcParamDeque.size() == mcs->numWindows){
        mcs->calcAugmentedMatrix();

        mcs->setupQP();

        et = clock();
        tmList[2] = (double) 1000*(et-st)/CLOCKS_PER_SEC;
        st = et;

        if(mcs->execQP()) failIdxVec.push_back(i - mcs->numWindows);

        et = clock();
        tmList[3] = (double) 1000*(et-st)/CLOCKS_PER_SEC;
        st = et;

        mcs->updateX0Vector();
        dvector x0(mcs->stateDim);
        x0 = mcs->x0;
        Vector3 CM,P,L;
        CM << x0[0],x0[2],0;
        P << x0[1],x0[3],0;
        L << x0[4],x0[5],0;
        CM /= mcs->m;
        mOfs1 << (i - mcs->numWindows + 1)*mcs->dt << " " << CM.transpose() <<  " " << P.transpose() << " " << L.transpose() << " " << endl;

        mcs->mpcParamDeque.pop_front();
    }

    cout << "  FK: " << tmList[0] << "[ms]  convert: " << tmList[1] << "[ms]  setup: " << tmList[2] << "[ms]  QP: " << tmList[3]  << "[ms]" << endl;
}

void Test::generateMotion(int from, int to, std::vector<ContactConstraintParam*>& ccParamVec)
{
    for(int i=from; i<to; ++i){
        processCycle(i,ccParamVec);
    }
}

int main(void)
{
    cout << "test function" << endl;

    Test test;

    std::deque<ModelPredictiveControllerParam*> mpcParamDeque;

    Vector3 pl,pr;
    pl << 0, 0.1,0;
    pr << 0,-0.1,0;
    test.pMap["lleg"] = pl;
    test.pMap["rleg"] = pr;
    test.RMap["lleg"] = Matrix33::Identity();
    test.RMap["rleg"] = Matrix33::Identity();

    std::vector<Vector3> edgeVec;
    hrp::Vector3 edge;
    edge << -1, 0, 0.1;
    edgeVec.push_back(edge);
    edge << 1, 0, 0.1;
    edgeVec.push_back(edge);
    edge << 0, -1, 0.05;
    edgeVec.push_back(edge);
    edge << 0, 1, 0.05;
    edgeVec.push_back(edge);

    test.mOfs0 << "time initCMx initCMy initCMz initPx initPy initPz initLx initLy initLz" << endl;
    test.mOfs1 << "time refCMx refCMy refCMz refPx refPy refPz refLx refLy refLz" << endl;
    test.mOfs2 << "time lx ly lz rx ry rz" << endl;
    int k = 0;
    std::vector<ContactConstraintParam*> ccParamVec;
    test.dCM = test.stride/(2*2*test.cycle);
    ccParamVec.push_back(new SimpleContactConstraintParam("lleg",edgeVec));
    ccParamVec.push_back(new SimpleContactConstraintParam("rleg",edgeVec));
    test.generateMotion(test.cycle*k, test.cycle*(k+test.r), ccParamVec);
    ccParamVec.clear();
    ccParamVec.push_back(new SimpleContactConstraintParam("lleg",edgeVec));
    test.generateMotion(test.cycle*(k+test.r), test.cycle*(k+1), ccParamVec);
    test.pMap["rleg"] += test.stride/2;
    ++k;

    test.dCM = test.stride/(2*test.cycle);

    for(int turn = 0; turn < 4; ++turn){
        ccParamVec.clear();
        ccParamVec.push_back(new SimpleContactConstraintParam("lleg",edgeVec));
        ccParamVec.push_back(new SimpleContactConstraintParam("rleg",edgeVec));
        test.generateMotion(test.cycle*k, test.cycle*(k+test.r), ccParamVec);
        ccParamVec.clear();
        ccParamVec.push_back(new SimpleContactConstraintParam("rleg",edgeVec));
        test.generateMotion(test.cycle*(k+test.r), test.cycle*(k+1), ccParamVec);
        test.pMap["lleg"] += test.stride;
        ++k;

        ccParamVec.clear();
        ccParamVec.push_back(new SimpleContactConstraintParam("lleg",edgeVec));
        ccParamVec.push_back(new SimpleContactConstraintParam("rleg",edgeVec));
        test.generateMotion(test.cycle*k, test.cycle*(k+test.r), ccParamVec);
        ccParamVec.clear();
        ccParamVec.push_back(new SimpleContactConstraintParam("lleg",edgeVec));
        test.generateMotion(test.cycle*(k+test.r), test.cycle*(k+1), ccParamVec);
        test.pMap["rleg"] += test.stride;
        ++k;
    }

    test.dCM = test.stride/(2*2*test.cycle);

    ccParamVec.clear();
    ccParamVec.push_back(new SimpleContactConstraintParam("lleg",edgeVec));
    ccParamVec.push_back(new SimpleContactConstraintParam("rleg",edgeVec));
    test.generateMotion(test.cycle*k, test.cycle*(k+test.r), ccParamVec);
    ccParamVec.clear();
    ccParamVec.push_back(new SimpleContactConstraintParam("rleg",edgeVec));
    test.generateMotion(test.cycle*(k+test.r), test.cycle*(k+1), ccParamVec);
    test.pMap["lleg"] += test.stride/2;
    ++k;

    test.stride = Vector3::Zero();
    ccParamVec.clear();
    ccParamVec.push_back(new SimpleContactConstraintParam("lleg",edgeVec));
    ccParamVec.push_back(new SimpleContactConstraintParam("rleg",edgeVec));
    test.generateMotion(test.cycle*k, test.cycle*(k+test.r), ccParamVec);

    FILE* p;
    p = popen("gnuplot","w");
    fprintf(p, "set grid\n");
    fprintf(p, "set yrange [0:2]\n");
    fprintf(p, "plot '/tmp/mcs-test_init.dat' u 1:%d t columnhead, '/tmp/mcs-test_ref.dat' u 1:%d t columnhead, '/tmp/mcs-test_contact.dat' u 1:%d t columnhead w lp, '/tmp/mcs-test_contact.dat' u 1:%d+3 t columnhead w lp\n", 2,2,2,2);// CoM x
    fflush(p);
    cout << "Please Enter" << endl;
    while(fgetc(stdin) != '\n');
    fprintf(p, "set yrange [-0.3:0.3]\n");
    fprintf(p, "plot '/tmp/mcs-test_init.dat' u 1:%d t columnhead, '/tmp/mcs-test_ref.dat' u 1:%d t columnhead, '/tmp/mcs-test_contact.dat' u 1:%d t columnhead w lp, '/tmp/mcs-test_contact.dat' u 1:%d+3 t columnhead w lp\n", 3,3,3,3);// CoM y
    fflush(p);
    while(fgetc(stdin) != '\n');
    fprintf(p, "set yrange [-10:10]\n");
    fprintf(p, "plot '/tmp/mcs-test_init.dat' u 1:%d t columnhead, '/tmp/mcs-test_init.dat' u 1:%d t columnhead, '/tmp/mcs-test_ref.dat' u 1:%d t columnhead, '/tmp/mcs-test_ref.dat' u 1:%d t columnhead\n", 5,6,5,6);// momentum
    fflush(p);
    while(fgetc(stdin) != '\n');
    fprintf(p, "set yrange [-0.3:0.3]\n");
    fprintf(p, "plot '/tmp/mcs-test_init.dat' u 1:%d t columnhead, '/tmp/mcs-test_init.dat' u 1:%d t columnhead, '/tmp/mcs-test_ref.dat' u 1:%d t columnhead, '/tmp/mcs-test_ref.dat' u 1:%d t columnhead\n", 8,9,8,9);// angular momentum
    fflush(p);
    while(fgetc(stdin) != '\n');    pclose(p);

    test.testAugmentedMatrix();
}
