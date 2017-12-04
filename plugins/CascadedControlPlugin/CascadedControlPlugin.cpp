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

MultiValueSeq::Frame operator-(const MultiValueSeq::Frame& frame0, const MultiValueSeq::Frame& frame1)
{
    MultiValueSeq::Frame frame = frame0;
    for(int i=0; i<frame.size(); ++i){
        frame[i] = frame0[i] - frame1[i];
    }
    return frame;
}

void cnoid::interpolateExtraSeq(BodyMotionItemPtr& bodyMotionItemPtr, double T)
{
    cout << "interpolateExtraSeq()" << endl;
    BodyMotionPtr motion = bodyMotionItemPtr->motion();
    const double dt = 1.0/motion->frameRate();
    const int cycle = T/dt;
    const int numFrames = motion->numFrames();
    Vector3SeqPtr refZmpSeqPtr = bodyMotionItemPtr->findSubItem<Vector3SeqItem>("ZMP")->seq();
    Vector3SeqPtr refCMSeqPtr = bodyMotionItemPtr->findSubItem<Vector3SeqItem>("refCM")->seq();
    Vector3SeqPtr refPSeqPtr = bodyMotionItemPtr->findSubItem<Vector3SeqItem>("refP")->seq();
    Vector3SeqPtr refLSeqPtr = bodyMotionItemPtr->findSubItem<Vector3SeqItem>("refL")->seq();
    MultiValueSeqPtr refWrenchesSeqPtr = bodyMotionItemPtr->findSubItem<MultiValueSeqItem>("wrenches")->seq();
    for(int i=0; i<numFrames-cycle; i+=cycle){
        Vector3d frontZmp = refZmpSeqPtr->at(i);
        Vector3d diffZmp = refZmpSeqPtr->at(i+cycle) - frontZmp;
        Vector3d frontCM = refCMSeqPtr->at(i);
        Vector3d diffCM = refCMSeqPtr->at(i+cycle) - frontCM;
        Vector3d frontP = refPSeqPtr->at(i);
        Vector3d diffP = refPSeqPtr->at(i+cycle) - frontP;
        Vector3d frontL = refLSeqPtr->at(i);
        Vector3d diffL = refLSeqPtr->at(i+cycle) - frontL;
        MultiValueSeq::Frame frontWrenches = refWrenchesSeqPtr->frame(i);
        MultiValueSeq::Frame nextWrenches = refWrenchesSeqPtr->frame(i+cycle);
        for(int j=0; j<cycle; ++j){
            double r = ((double)j/cycle);
            refZmpSeqPtr->at(i+j) = frontZmp + r*diffZmp;
            Vector3d CM = frontCM + r*diffCM;
            CM.z() = refCMSeqPtr->at(i+j).z(); // keep z coodinates
            refCMSeqPtr->at(i+j) = CM;
            Vector3d P = frontP + r*diffP;
            P.z() = refPSeqPtr->at(i+j).z(); // keep z coodinates
            refPSeqPtr->at(i+j) = P;
            refLSeqPtr->at(i+j) = frontL + r*diffL;
            for(int k=0; k<frontWrenches.size(); ++k){
                refWrenchesSeqPtr->frame(i+j)[k] = (1-r)*frontWrenches[k] + r*nextWrenches[k];
            }
        }
    }
}

void CascadedControlPlugin::execControl(bool loadFlg)
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
    parentSfc->numXDivisions = parentSfcLayout->xDivisionNumSpin->value();
    parentSfc->numYDivisions = parentSfcLayout->yDivisionNumSpin->value();

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
    childSfc->numXDivisions = childSfcLayout->xDivisionNumSpin->value();
    childSfc->numYDivisions = childSfcLayout->yDivisionNumSpin->value();
    // overwrite parentSfc's numDivisions by childSfc's
    // numDivisionsは本来sfcではなくsfcparamの設定変数なのでparent,childにかかわらず共通であるべき?
    parentSfc->numXDivisions = childSfc->numXDivisions;
    parentSfc->numYDivisions = childSfc->numYDivisions;

    if(loadFlg){
        loadExtraSeq(mPoseSeqPath ,childSfcLayout->getParamString(), childSfc, body, mBodyMotionItemPtr, contactLinkCandidateSet);
    }else{
        generateVerticalTrajectory(body, poseSeqItemPtr, contactLinkCandidateSet);
        generatePreModelPredictiveControlParamDeque(parentSfc, body, poseSeqItemPtr, contactLinkCandidateSet);
        {// add last param for interpolation
            SlideFrictionControlParam* lastSfcParam = (SlideFrictionControlParam*) parentSfc->preMpcParamDeque.back();
            SlideFrictionControlParam* tmpParam = new SlideFrictionControlParam(lastSfcParam->index()+1, parentSfc, lastSfcParam);
            parentSfc->preMpcParamDeque.push_back(tmpParam);
        }
        childSfc->pushAllPreMPCParamFromRoot();

        sweepControl(mPoseSeqPath, childSfcLayout->getParamString(), childSfc, body, mBodyMotionItemPtr, contactLinkCandidateSet);// childモーション走査
    }
    interpolateExtraSeq(mBodyMotionItemPtr, childSfc->dt);
    // sweepControl(mPoseSeqPath, parentSfcLayout->getParamString(), parentSfc, body, mBodyMotionItemPtr, contactLinkCandidateSet);// parentモーション走査

    cout << "Finished CascadedControl" << endl;
}

CNOID_IMPLEMENT_PLUGIN_ENTRY(CascadedControlPlugin)
