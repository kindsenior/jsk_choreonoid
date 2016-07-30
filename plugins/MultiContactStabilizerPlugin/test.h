/**
   @author Kunio Kojima
*/
#pragma once

#include "MultiContactStabilizer.h"
#include <time.h>

namespace hrp {

class OneMCSTest
{
public:
    MultiContactStabilizer* mcs;
    std::ofstream mOfs0,mOfs1,mOfs2;
    std::vector<int> failIdxVec;
    Vector3 stride,dCM,CM,P,L,F;
    double stepTime,r;
    std::map<std::string,Vector3> pMap;
    std::map<std::string,Matrix33> RMap;

    OneMCSTest()
    {
        mOfs0.open("/tmp/mcs-test_init.dat");
        mOfs1.open("/tmp/mcs-test_ref.dat");
        mOfs2.open("/tmp/mcs-test_contact.dat");

        stride << 0.4,0,0;
        // stride << 0,0,0;
        stepTime = 1.25;
        r = 0.25/stepTime;// double support ratio
        CM << 0,0,1;

        mcs = new MultiContactStabilizer();

        mcs->m = 60;
        mcs->dt = 0.05;
        std::vector<int> vec{1,1,1,1,1,1,1,1,1,1,1,1,1};
        mcs->setBlockVector(vec);
        mcs->errorCMWeight = 150;
        mcs->errorMomentumWeight = 10;
        mcs->errorAngularMomentumWeight = 10000;
        mcs->inputForceWeight = 0.001;
        mcs->inputMomentWeight = 10;
    };

    virtual void testAugmentedMatrix();

    void generateOnePhase(int from, int to, std::vector<ContactConstraintParam*>& ccParamVec);
    void generateMotion();
    virtual void execControl();
};

class TwoStepMCSTest : public OneMCSTest
{
protected:
    MultiContactStabilizer* mcs1;
    std::ofstream mOfs3;

public:
    TwoStepMCSTest()
        : OneMCSTest()
    {
        mcs->m = 60;
        mcs->dt = 0.002;// 2ms
        std::vector<int> vec{1,1,1,1,1};
        mcs->setBlockVector(vec);
        mcs->errorCMWeight = 100000000;
        mcs->errorMomentumWeight = 1;
        mcs->errorAngularMomentumWeight = 200;
        mcs->inputForceWeight = 10;
        mcs->inputMomentWeight = 20;

        mcs1 = new MultiContactStabilizer();

        mcs1->parent = mcs;
        mcs1->m = mcs->m;
        mcs1->dt = 0.1;// 100ms
        std::vector<int> vec1{1,1,1,1,1,1,1};
        mcs1->setBlockVector(vec1);
        mcs1->errorCMWeight = 1000;
        mcs1->errorMomentumWeight = 0.0001;
        mcs1->errorAngularMomentumWeight = 200;
        mcs1->inputForceWeight = 10;
        mcs1->inputMomentWeight = 20;

        mOfs3.open("/tmp/mcs-test_ref_sparse.dat");
    }

    void execControl();
};

}
