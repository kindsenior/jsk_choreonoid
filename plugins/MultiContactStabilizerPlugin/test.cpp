/**
   @author Kunio Kojima
*/
#include "test.h"

using namespace std;
using namespace hrp;

void OneMCSTest::testAugmentedMatrix()
{
    cout << endl << "augmented matrix test" << endl;

    mcs->calcAugmentedMatrixAndSetupArgument();

    typename std::deque<MultiContactStabilizerParam*> mpcParamDeque =  mcs->mpcParamDeque;
    int stateDim = mcs->stateDim;
    dvector U(mcs->psiCols);
    dvector xk(stateDim);
    xk = mcs->x0;
    dvector x0(stateDim);
    x0 = mcs->x0;
    int colIdx = 0;
    for(typename std::deque<MultiContactStabilizerParam*>::iterator iter = mpcParamDeque.begin(); iter != mpcParamDeque.end(); ++iter){
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

    cout << "numWindows: " << mcs->numWindows() << endl;
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


void OneMCSTest::generateOnePhase(int from, int to, std::vector<ContactConstraintParam*>& ccParamVec)
{
    static Vector3 g;
    g << 0,0,9.8;
    for(int i=from; i<to; ++i){
        MultiContactStabilizerParam* mcsParam = new MultiContactStabilizerParam(i, mcs);
        for(std::vector<ContactConstraintParam*>::iterator iter = ccParamVec.begin(); iter != ccParamVec.end(); ++iter){
            (*iter)->p = pMap[(*iter)->linkName];
            (*iter)->R = RMap[(*iter)->linkName];
        }
        mcsParam->ccParamVec = ccParamVec;

        {
            CM += dCM;
            mcsParam->CM = CM;
            mcsParam->P = dCM*mcs->m;
            mcsParam->L << 0,0,0;
            mcsParam->F = mcs->m*dCM/mcs->dt + mcs->m*g;

            mOfs0 << i*mcs->dt << " " << CM.transpose() <<  " " << P.transpose() << " " << L.transpose() << endl;
            mOfs2 << i*mcs->dt << " "<< pMap["lleg"].transpose() << " " << pMap["rleg"].transpose() << endl;
        }

        mcs->preMpcParamDeque.push_back(mcsParam);
    }
}

void OneMCSTest::generateMotion()
{
    int cycle = stepTime/mcs->dt;

    Vector3 pl,pr;
    pl << 0, 0.1,0;
    pr << 0,-0.1,0;
    pMap["lleg"] = pl;
    pMap["rleg"] = pr;
    RMap["lleg"] = Matrix33::Identity();
    RMap["rleg"] = Matrix33::Identity();

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

    mOfs0 << "time initCMx initCMy initCMz initPx initPy initPz initLx initLy initLz" << endl;
    mOfs1 << "time refCMx refCMy refCMz refPx refPy refPz refLx refLy refLz processTime" << endl;
    mOfs2 << "time lx ly lz rx ry rz" << endl;
    int k = 0;
    std::vector<ContactConstraintParam*> ccParamVec;
    dCM = stride/(2*2*cycle);
    ccParamVec.push_back(new SimpleContactConstraintParam("lleg",edgeVec));
    ccParamVec.push_back(new SimpleContactConstraintParam("rleg",edgeVec));
    generateOnePhase(cycle*k, cycle*(k+r), ccParamVec);
    ccParamVec.clear();
    ccParamVec.push_back(new SimpleContactConstraintParam("lleg",edgeVec));
    generateOnePhase(cycle*(k+r), cycle*(k+1), ccParamVec);
    pMap["rleg"] += stride/2;
    ++k;

    dCM = stride/(2*cycle);

    for(int turn = 0; turn < 4; ++turn){
        ccParamVec.clear();
        ccParamVec.push_back(new SimpleContactConstraintParam("lleg",edgeVec));
        ccParamVec.push_back(new SimpleContactConstraintParam("rleg",edgeVec));
        generateOnePhase(cycle*k, cycle*(k+r), ccParamVec);
        ccParamVec.clear();
        ccParamVec.push_back(new SimpleContactConstraintParam("rleg",edgeVec));
        generateOnePhase(cycle*(k+r), cycle*(k+1), ccParamVec);
        pMap["lleg"] += stride;
        ++k;

        ccParamVec.clear();
        ccParamVec.push_back(new SimpleContactConstraintParam("lleg",edgeVec));
        ccParamVec.push_back(new SimpleContactConstraintParam("rleg",edgeVec));
        generateOnePhase(cycle*k, cycle*(k+r), ccParamVec);
        ccParamVec.clear();
        ccParamVec.push_back(new SimpleContactConstraintParam("lleg",edgeVec));
        generateOnePhase(cycle*(k+r), cycle*(k+1), ccParamVec);
        pMap["rleg"] += stride;
        ++k;
    }

    dCM = stride/(2*2*cycle);

    ccParamVec.clear();
    ccParamVec.push_back(new SimpleContactConstraintParam("lleg",edgeVec));
    ccParamVec.push_back(new SimpleContactConstraintParam("rleg",edgeVec));
    generateOnePhase(cycle*k, cycle*(k+r), ccParamVec);
    ccParamVec.clear();
    ccParamVec.push_back(new SimpleContactConstraintParam("rleg",edgeVec));
    generateOnePhase(cycle*(k+r), cycle*(k+1), ccParamVec);
    pMap["lleg"] += stride/2;
    ++k;

    stride = Vector3::Zero();
    ccParamVec.clear();
    ccParamVec.push_back(new SimpleContactConstraintParam("lleg",edgeVec));
    ccParamVec.push_back(new SimpleContactConstraintParam("rleg",edgeVec));
    generateOnePhase(cycle*k, cycle*(k+r), ccParamVec);
}

void OneMCSTest::execControl()
{
    float avgTime = 0;
    std::vector<int> failIdxVec;
    int numFrames = mcs->preMpcParamDeque.size();
    for(int i=0; i < numFrames + mcs->numWindows(); ++i){
        cout << endl << "##############################" << endl << "processCycle() turn:" << i << endl;
        float processedTime;
        if(mcs->processCycle(processedTime)) failIdxVec.push_back(i - mcs->numWindows());
        avgTime += processedTime;

        // file output for plot
        if(i >= mcs->numWindows()){
            dvector x0 = mcs->x0;

            Vector3 CM,P,L;
            CM << x0[0],x0[2],0;
            P << x0[1],x0[3],0;
            L << x0[4],x0[5],0;
            CM /= mcs->m;
            mOfs1 << (i - mcs->numWindows())*mcs->dt << " " << CM.transpose() <<  " " << P.transpose() << " " << L.transpose() << " " << processedTime << endl;
        }
    }
    cout << "average time: " << avgTime/numFrames << "[msec]" << endl;
}

void TwoStepMCSTest::execControl()
{
    mcs1->pushAllPreMPCParamFromRoot();
    mOfs3 << "time refCMx refCMy refCMz refPx refPy refPz refLx refLy refLz processTime" << endl;

    float avgTime = 0;
    std::vector<int> failIdxVec;
    int numFrames = mcs1->preMpcParamDeque.size();
    for(int i=0; i < numFrames + mcs1->numWindows(); ++i){
        cout << endl << "##############################" << endl << "processCycle() turn:" << i << endl;
        float processedTime;
        if(mcs1->processCycle(processedTime)) failIdxVec.push_back(i - mcs1->numWindows());
        avgTime += processedTime;

        // file output for plot
        if(i >= mcs1->numWindows()){
            dvector x0 = mcs1->x0;

            Vector3 CM,P,L;
            CM << x0[0],x0[2],0;
            P << x0[1],x0[3],0;
            L << x0[4],x0[5],0;
            CM /= mcs1->m;
            mOfs3 << (i - mcs1->numWindows())*mcs1->dt << " " << CM.transpose() <<  " " << P.transpose() << " " << L.transpose() << " " << processedTime << endl;
        }
    }
    cout << "average time: " << avgTime/numFrames << "[msec]" << endl;

    OneMCSTest::execControl();
}

void printUsage(std::vector<string> argvStringVec)
{
    cout << "Usage: mcs-test [mode]" << endl;
    cout << " [mode] is one of";
    for(std::vector<string>::iterator iter = argvStringVec.begin(); iter != argvStringVec.end(); ++iter){
        cout << " " << *iter;
    }
    cout << endl;
}

int main(int argc, char** argv)
{
    OneMCSTest* test;
    std::vector<string> argvStringVec {"one", "two"};
    if(argc > 1){
        string argvStr = argv[1];
        if(argvStr ==  argvStringVec[0]){
            cout << "execute OneMCSTest" << endl;
            test = (OneMCSTest*) new OneMCSTest();
        }else if(argvStr == argvStringVec[1]){
            cout << "execute TwoStepMCSTest" << endl;
            test = (OneMCSTest*) new TwoStepMCSTest();
        }else{
            printUsage(argvStringVec);
            return 0;
        }
    }else{
        printUsage(argvStringVec);
        return 0;
    }

    test->generateMotion();
    test->execControl();

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
    while(fgetc(stdin) != '\n');
    fprintf(p, "set yrange [0:20]\n");
    fprintf(p, "plot '/tmp/mcs-test_ref.dat' u 1:%d t columnhead\n", 11);// process time
    fflush(p);
    while(fgetc(stdin) != '\n');
    pclose(p);

    test->testAugmentedMatrix();
}
