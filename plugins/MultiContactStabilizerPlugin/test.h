/**
   @author Kunio Kojima
*/
#pragma once

#include "MultiContactStabilizer.h"
#include <time.h>

namespace hrp {

class Test
{
public:
    MultiContactStabilizer* mcs;
    std::ofstream mOfs0,mOfs1,mOfs2;
    std::vector<int> failIdxVec;
    Vector3 stride,dCM,CM,P,L,F;
    double stepTime,r;
    std::map<std::string,Vector3> pMap;
    std::map<std::string,Matrix33> RMap;

    Test()
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

    void testAugmentedMatrix();

    void generateOnePhase(int from, int to, std::vector<ContactConstraintParam*>& ccParamVec);
    void generateMotion();
    void execControl();
};

}
