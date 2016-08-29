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

    // BodyMotion作成
    generateBodyMotionFromBar(body, poseSeqItemPtr, mBodyMotionItemPtr);

    frameRate = motion->frameRate();
    dt = 1.0/frameRate;
    numFrames = motion->numFrames();

    generateInitSeq(body, poseSeqItemPtr);

    // 接触候補セットの作成
    std::set<Link*> contactLinkCandidateSet;
    calcContactLinkCandidateSet(contactLinkCandidateSet, body, poseSeqPtr);

    MultiContactStabilizer* parentMcs = new MultiContactStabilizer();
    parentMcs->m = body->mass();
    parentMcs->dt = dt;
    MultiContactStabilizerSetupLayout* parentMcsLayout = (MultiContactStabilizerSetupLayout*) mBar->dialog->layout()->itemAt(0)->layout();
    parentMcs->setBlockVector(parentMcsLayout->blockSpinVec->value());
    parentMcs->errorCMWeight = parentMcsLayout->errorCMWeightSpin->value();
    parentMcs->errorMomentumWeight = parentMcsLayout->errorMomentumWeightSpin->value();
    parentMcs->errorAngularMomentumWeight = parentMcsLayout->errorAngularMomentumWeightSpin->value();
    parentMcs->inputForceWeight = parentMcsLayout->inputForceWeightSpin->value();
    parentMcs->inputMomentWeight = parentMcsLayout->inputMomentWeightSpin->value();

    MultiContactStabilizer* childMcs = new MultiContactStabilizer();
    childMcs->parent = parentMcs;
    childMcs->m = body->mass();
    MultiContactStabilizerSetupLayout* childMcsLayout = (MultiContactStabilizerSetupLayout*) mBar->dialog->layout()->itemAt(1)->layout();
    childMcs->dt = childMcsLayout->dtSpin->value();
    childMcs->setBlockVector(childMcsLayout->blockSpinVec->value());
    childMcs->errorCMWeight = childMcsLayout->errorCMWeightSpin->value();
    childMcs->errorMomentumWeight = childMcsLayout->errorMomentumWeightSpin->value();
    childMcs->errorAngularMomentumWeight = childMcsLayout->errorAngularMomentumWeightSpin->value();
    childMcs->inputForceWeight = childMcsLayout->inputForceWeightSpin->value();
    childMcs->inputMomentWeight = childMcsLayout->inputMomentWeightSpin->value();

    generatePreModelPredictiveControlParamDeque(parentMcs, body, poseSeqPtr, motion, contactLinkCandidateSet);
    {// add last param for interpolation
        MultiContactStabilizerParam* lastMcsParam = (MultiContactStabilizerParam*) parentMcs->preMpcParamDeque.back();
        MultiContactStabilizerParam* tmpParam = new MultiContactStabilizerParam(lastMcsParam->index()+1, parentMcs, lastMcsParam);
        parentMcs->preMpcParamDeque.push_back(tmpParam);
    }
    childMcs->pushAllPreMPCParamFromRoot();

    sweepControl(mPoseSeqPath, childMcsLayout->getParamString(), childMcs, body, mBodyMotionItemPtr);// childモーション走査
    sweepControl(mPoseSeqPath, parentMcsLayout->getParamString(), parentMcs, body, mBodyMotionItemPtr);// parentモーション走査

    cout << "Finished CascadedControl" << endl;
}

CNOID_IMPLEMENT_PLUGIN_ENTRY(CascadedControlPlugin)
