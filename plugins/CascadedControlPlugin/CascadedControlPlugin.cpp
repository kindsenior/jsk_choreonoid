/**
   @author Kunio Kojima
*/
#include "CascadedControlPlugin.h"

using namespace boost;
using namespace cnoid;
using namespace std;
using namespace hrp;

bool CascadedControlPlugin::initialize()
{
    mBar = new CascadedControlBar(this);
    addToolBar(mBar);
    return true;
}

MultiValueSeq::Frame operator+(const MultiValueSeq::Frame& frame0, const MultiValueSeq::Frame& frame1)
{
    MultiValueSeq::Frame frame = frame0;
    for(int i=0; i<frame.size(); ++i){
        frame[i] += frame1[i];
    }
    return frame;
}

MultiValueSeq::Frame operator-(const MultiValueSeq::Frame& frame0, const MultiValueSeq::Frame& frame1)
{
    MultiValueSeq::Frame frame = frame0;
    for(int i=0; i<frame.size(); ++i){
        frame[i] -= frame1[i];
    }
    return frame;
}
MultiValueSeq::Frame operator*(const double d, const MultiValueSeq::Frame& frame0)
{
    MultiValueSeq::Frame frame = frame0;
    for(int i=0; i<frame.size(); ++i){
        frame[i] *= d;
    }
    return frame;
}

void cnoid::interpolateExtraSeq(BodyMotionItemPtr& bodyMotionItemPtr, double T)
{
    BodyMotionPtr motion = bodyMotionItemPtr->motion();
    const double dt = 1.0/motion->frameRate();
    const int cycle = T/dt;
    const int numFrames = motion->numFrames();
    Vector3SeqPtr refZmpSeqPtr = bodyMotionItemPtr->motion()->getOrCreateExtraSeq<Vector3Seq>("ZMP");
		MultiValueSeqPtr refWrenchesSeqPtr = bodyMotionItemPtr->motion()->getOrCreateExtraSeq<MultiValueSeq>("wrenches");
    for(int i=0; i<numFrames-cycle; i+=cycle){
        Vector3d front = refZmpSeqPtr->at(i);
        Vector3d diff = refZmpSeqPtr->at(i+cycle) - front;
        MultiValueSeq::Frame frontWrenches = refWrenchesSeqPtr->frame(i);
        MultiValueSeq::Frame diffWrenches = refWrenchesSeqPtr->frame(i+cycle) - frontWrenches;
        for(int j=0; j<cycle; ++j){
            double r = ((double)j/cycle);
            refZmpSeqPtr->at(i+j) = front + r*diff;
            refWrenchesSeqPtr->frame(i+j) = frontWrenches  + r*diffWrenches;
        }
    }
}

void CascadedControlPlugin::execControl()
{
    stringstream ss,fnamess;
    MessageView::instance()->putln("CascadedControl called !");
    cout << "CascadedControl()" << endl;

    BodyItemPtr bodyItemPtr;
    PoseSeqItemPtr poseSeqItemPtr;
    PoseSeqPtr poseSeqPtr;
    if(!getSelectedPoseSeqSet(bodyItemPtr, body, poseSeqItemPtr, poseSeqPtr, mBodyMotionItemPtr, motion)) return;

    mPoseSeqPath = boost::filesystem::path(poseSeqItemPtr->filePath());
    cout << "PoseSeqPath: " << mPoseSeqPath << endl;

    std::vector<Link*> endEffectorLinkVec;
    if(!getEndEffectorLinkVector(endEffectorLinkVec, body)) return;

    generateOptionalData(body, poseSeqItemPtr, endEffectorLinkVec);

    // BodyMotion作成
    generateBodyMotionFromBar(body, poseSeqItemPtr, mBodyMotionItemPtr);

    frameRate = motion->frameRate();
    dt = 1.0/frameRate;
    numFrames = motion->numFrames();

    generateInitSeq(body, poseSeqItemPtr);

    // 接触候補セットの作成
    std::set<Link*> contactLinkCandidateSet;
    calcContactLinkCandidateSet(contactLinkCandidateSet, body, poseSeqPtr);

    // MultiContactStabilizer* parentMcs = new MultiContactStabilizer();
    // parentMcs->m = body->mass();
    // parentMcs->dt = dt;
    // MultiContactStabilizerSetupLayout* parentMcsLayout = (MultiContactStabilizerSetupLayout*) mBar->dialog->layout()->itemAt(0)->layout();
    // parentMcs->setBlockVector(parentMcsLayout->blockSpinVec->value());
    // parentMcs->errorCMWeight = parentMcsLayout->errorCMWeightSpin->value();
    // parentMcs->errorMomentumWeight = parentMcsLayout->errorMomentumWeightSpin->value();
    // parentMcs->errorAngularMomentumWeight = parentMcsLayout->errorAngularMomentumWeightSpin->value();
    // parentMcs->inputForceWeight = parentMcsLayout->inputForceWeightSpin->value();
    // parentMcs->inputMomentWeight = parentMcsLayout->inputMomentWeightSpin->value();

    // MultiContactStabilizer* childMcs = new MultiContactStabilizer();
    // childMcs->parent = parentMcs;
    // childMcs->m = body->mass();
    // MultiContactStabilizerSetupLayout* childMcsLayout = (MultiContactStabilizerSetupLayout*) mBar->dialog->layout()->itemAt(1)->layout();
    // childMcs->dt = childMcsLayout->dtSpin->value();
    // childMcs->setBlockVector(childMcsLayout->blockSpinVec->value());
    // childMcs->errorCMWeight = childMcsLayout->errorCMWeightSpin->value();
    // childMcs->errorMomentumWeight = childMcsLayout->errorMomentumWeightSpin->value();
    // childMcs->errorAngularMomentumWeight = childMcsLayout->errorAngularMomentumWeightSpin->value();
    // childMcs->inputForceWeight = childMcsLayout->inputForceWeightSpin->value();
    // childMcs->inputMomentWeight = childMcsLayout->inputMomentWeightSpin->value();

    // generatePreModelPredictiveControlParamDeque(parentMcs, body, poseSeqPtr, motion, contactLinkCandidateSet);
    // {// add last param for interpolation
    //     MultiContactStabilizerParam* lastMcsParam = (MultiContactStabilizerParam*) parentMcs->preMpcParamDeque.back();
    //     MultiContactStabilizerParam* tmpParam = new MultiContactStabilizerParam(lastMcsParam->index()+1, parentMcs, lastMcsParam);
    //     parentMcs->preMpcParamDeque.push_back(tmpParam);
    // }
    // childMcs->pushAllPreMPCParamFromRoot();

    // sweepControl(mPoseSeqPath, childMcsLayout->getParamString(), childMcs, body, mBodyMotionItemPtr);// childモーション走査
    // sweepControl(mPoseSeqPath, parentMcsLayout->getParamString(), parentMcs, body, mBodyMotionItemPtr);// parentモーション走査


    SlideFrictionControl* parentSfc = new SlideFrictionControl();
    parentSfc->m = body->mass();
    parentSfc->dt = dt;
    SlideFrictionControlSetupLayout* parentSfcLayout = (SlideFrictionControlSetupLayout*) mBar->dialog->layout()->itemAt(0)->layout();
    parentSfc->setBlockVector(parentSfcLayout->blockSpinVec->value());
    parentSfc->errorCMWeight = parentSfcLayout->errorCMWeightSpin->value();
    parentSfc->errorMomentumWeight = parentSfcLayout->errorMomentumWeightSpin->value();
    parentSfc->errorAngularMomentumWeight = parentSfcLayout->errorAngularMomentumWeightSpin->value();
    parentSfc->errorYawAngularMomentumWeight = parentSfcLayout->errorYawAngularMomentumWeightSpin->value();
    parentSfc->inputForceWeight = parentSfcLayout->inputForceWeightSpin->value();
    parentSfc->inputMomentWeight = parentSfcLayout->inputMomentWeightSpin->value();
    parentSfc->inputYawMomentWeight = parentSfcLayout->inputYawMomentWeightSpin->value();

    SlideFrictionControl* childSfc = new SlideFrictionControl();
    childSfc->parent = parentSfc;
    childSfc->m = body->mass();
    SlideFrictionControlSetupLayout* childSfcLayout = (SlideFrictionControlSetupLayout*) mBar->dialog->layout()->itemAt(1)->layout();
    childSfc->dt = childSfcLayout->dtSpin->value();
    childSfc->setBlockVector(childSfcLayout->blockSpinVec->value());
    childSfc->errorCMWeight = childSfcLayout->errorCMWeightSpin->value();
    childSfc->errorMomentumWeight = childSfcLayout->errorMomentumWeightSpin->value();
    childSfc->errorAngularMomentumWeight = childSfcLayout->errorAngularMomentumWeightSpin->value();
    childSfc->errorYawAngularMomentumWeight = childSfcLayout->errorYawAngularMomentumWeightSpin->value();
    childSfc->inputForceWeight = childSfcLayout->inputForceWeightSpin->value();
    childSfc->inputMomentWeight = childSfcLayout->inputMomentWeightSpin->value();
    childSfc->inputYawMomentWeight = childSfcLayout->inputYawMomentWeightSpin->value();

    generatePreModelPredictiveControlParamDeque(parentSfc, body, poseSeqPtr, motion, contactLinkCandidateSet);
    {// add last param for interpolation
        SlideFrictionControlParam* lastSfcParam = (SlideFrictionControlParam*) parentSfc->preMpcParamDeque.back();
        SlideFrictionControlParam* tmpParam = new SlideFrictionControlParam(lastSfcParam->index()+1, parentSfc, lastSfcParam);
        parentSfc->preMpcParamDeque.push_back(tmpParam);
    }
    childSfc->pushAllPreMPCParamFromRoot();

    sweepControl(mPoseSeqPath, childSfcLayout->getParamString(), childSfc, body, mBodyMotionItemPtr, contactLinkCandidateSet);// childモーション走査
    interpolateExtraSeq(mBodyMotionItemPtr, childSfc->dt);
    // sweepControl(mPoseSeqPath, parentSfcLayout->getParamString(), parentSfc, body, mBodyMotionItemPtr, contactLinkCandidateSet);// parentモーション走査

    cout << "Finished CascadedControl" << endl;
}

CNOID_IMPLEMENT_PLUGIN_ENTRY(CascadedControlPlugin)
