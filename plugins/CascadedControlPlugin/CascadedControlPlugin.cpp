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
    BodyMotion& motion = *(bodyMotionItemPtr->motion());
    const double dt = 1.0/motion.frameRate();
    const int cycle = T/dt;
    const int numFrames = motion.numFrames();
    std::shared_ptr<Vector3Seq> refZmpSeq = getOrCreateVector3SeqOfBodyMotionItem("ZMP", bodyMotionItemPtr);
    std::shared_ptr<Vector3Seq> refCMSeq = getOrCreateVector3SeqOfBodyMotionItem("refCM", bodyMotionItemPtr);
    std::shared_ptr<Vector3Seq> refPSeq = getOrCreateVector3SeqOfBodyMotionItem("refP", bodyMotionItemPtr);
    std::shared_ptr<Vector3Seq> refLSeq = getOrCreateVector3SeqOfBodyMotionItem("refL", bodyMotionItemPtr);
    std::shared_ptr<MultiValueSeq> refWrenchesSeq = getOrCreateMultiValueSeqOfBodyMotionItem("wrenches", bodyMotionItemPtr);
    std::shared_ptr<MultiValueSeq> optionalDataSeq = getOrCreateMultiValueSeqOfBodyMotionItem("optionaldata", bodyMotionItemPtr);

    const double wrenchDim = 6;
    const int numContacts = refWrenchesSeq->frame(0).size()/wrenchDim;
    for(int i=0; i<numFrames-cycle; i+=cycle){
        Vector3d frontZmp = refZmpSeq->at(i);
        Vector3d diffZmp = refZmpSeq->at(i+cycle) - frontZmp;
        Vector3d frontCM = refCMSeq->at(i);
        Vector3d diffCM = refCMSeq->at(i+cycle) - frontCM;
        Vector3d frontP = refPSeq->at(i);
        Vector3d diffP = refPSeq->at(i+cycle) - frontP;
        Vector3d frontL = refLSeq->at(i);
        Vector3d diffL = refLSeq->at(i+cycle) - frontL;
        for(int j=1; j<cycle; ++j){
            double r = ((double)j/cycle);
            refZmpSeq->at(i+j) = frontZmp + r*diffZmp;
            Vector3d CM = frontCM + r*diffCM;
            CM.z() = refCMSeq->at(i+j).z(); // keep z coodinates
            refCMSeq->at(i+j) = CM;
            Vector3d P = frontP + r*diffP;
            P.z() = refPSeq->at(i+j).z(); // keep z coodinates
            refPSeq->at(i+j) = P;
            refLSeq->at(i+j) = frontL + r*diffL;
        }

        MultiValueSeq::Frame frontWrenches = refWrenchesSeq->frame(i);
        MultiValueSeq::Frame nextWrenches = refWrenchesSeq->frame(i+cycle);
        std::vector<VectorXd> frontWrenchVec, nextWrenchVec;
        std::vector<int> frontIdxVec, nextIdxVec;
        for(int k=0; k<numContacts; ++k){// find front wrench
            frontWrenchVec.push_back(VectorXd::Zero(wrenchDim));
            for(int ii=0; ii<wrenchDim; ++ii) frontWrenchVec[k](ii) = frontWrenches[k*wrenchDim+ii];
            frontIdxVec.push_back(0);
            if(!optionalDataSeq->frame(i+0)[k]){
                for(int j=1; j<cycle; ++j){
                    MultiValueSeq::Frame optionalDataFrame = optionalDataSeq->frame(i+j);
                    if(optionalDataFrame[k] == 0){
                        frontWrenchVec[k] = VectorXd::Zero(wrenchDim);
                        frontIdxVec[k] = j;
                    }
                }
            }
        }
        for(int k=0; k<numContacts; ++k){// find next wrench
            nextWrenchVec.push_back(VectorXd::Zero(wrenchDim));
            for(int ii=0; ii<wrenchDim; ++ii) nextWrenchVec[k](ii) = nextWrenches[k*wrenchDim+ii];
            nextIdxVec.push_back(cycle);
            if(!optionalDataSeq->frame(i+cycle)[k]){
                for(int j=cycle; j>1; --j){
                    MultiValueSeq::Frame optionalDataFrame = optionalDataSeq->frame(i+j);
                    if(optionalDataFrame[k] == 0){
                        nextWrenchVec[k] = VectorXd::Zero(wrenchDim);
                        nextIdxVec[k] = j;
                    }
                }
            }
        }
        for(int k=0; k<numContacts; ++k){// interpolate wrench
            for(int j=frontIdxVec[k]+1; j<nextIdxVec[k]; ++j){
                double r= ((double)(j-frontIdxVec[k])/(nextIdxVec[k]-frontIdxVec[k]));
                for(int ii=0; ii<wrenchDim; ++ii){
                    refWrenchesSeq->frame(i+j)[k*wrenchDim+ii] = (1-r)*frontWrenchVec[k](ii) + r*nextWrenchVec[k](ii);
                }
            }
        }
        // overwrite optionalData (slide contact -1 -> 0 or 1)
        for(int j=0; j<cycle; ++j){
            for(int k=0; k<numContacts; ++k){
                // if(optionalDataSeq->frame(i+j)[k] == -1) optionalDataSeq->frame(i+j)[k] = 0;// single foot slide
                if(optionalDataSeq->frame(i+j)[k] == -1) optionalDataSeq->frame(i+j)[k] = 1;// both feet slide (temporally toe/heel contact)
            }
        }
    }

    {
        int finalDivisibleIdx = ((int) numFrames/cycle)*cycle;
        cout << "Remainder frame is from " << finalDivisibleIdx << " (" << finalDivisibleIdx*dt << "[sec]) to " << numFrames << " (" << numFrames*dt << "[sec])" << endl;
        MultiValueSeq::Frame lastWrenchFrame = refWrenchesSeq->frame(finalDivisibleIdx);
        for(int i=finalDivisibleIdx+1; i<numFrames; ++i){
            refZmpSeq->at(i) = refZmpSeq->at(finalDivisibleIdx);
            refCMSeq->at(i) = refCMSeq->at(finalDivisibleIdx);
            refPSeq->at(i) = refPSeq->at(finalDivisibleIdx);
            refLSeq->at(i) = refLSeq->at(finalDivisibleIdx);
            for(int k=0; k<lastWrenchFrame.size(); ++k) refWrenchesSeq->frame(i)[k] = lastWrenchFrame[k];
        }
    }

    // remove nan during jumping phase
    Vector3d prevZmp;
    int prevIdx;
    for(int i=0; i<numFrames; ++i){
        Vector3d zmp = refZmpSeq->at(i);
        if(std::isnan(zmp.x())){ // check x coordinate
            int nextVaridIdx = i;
            while(std::isnan(refZmpSeq->at(nextVaridIdx).x())) ++nextVaridIdx; // check x coordinate
            Vector3d diffZmp = refZmpSeq->at(nextVaridIdx) - prevZmp;
            cout << "jumping phase(nan check): " << prevIdx << " -> " << nextVaridIdx << endl;
            int startInterpolationIdx = prevIdx + cycle;
            int stopInterpolationIdx = nextVaridIdx - cycle;
            for(int j=prevIdx+1; j<startInterpolationIdx; ++j){
                refZmpSeq->at(j) = prevZmp;
            }
            for(int j=startInterpolationIdx; j<stopInterpolationIdx; ++j){
                refZmpSeq->at(j) = ((double)(j - startInterpolationIdx))/(stopInterpolationIdx - startInterpolationIdx)*diffZmp + prevZmp;
            }
            for(int j=stopInterpolationIdx; j<nextVaridIdx; ++j){
                refZmpSeq->at(j) = prevZmp + diffZmp;
            }

            i = nextVaridIdx;
        }else{
            prevIdx = i;
            prevZmp = zmp;
        }
    }
}

void CascadedControlPlugin::execControl(bool loadFlg)
{
    stringstream ss,fnamess;
    MessageView::instance()->putln("CascadedControl called !");
    cout << "\x1b[31m" << "CascadedControl()" << "\x1b[m" << endl;

    BodyItemPtr bodyItemPtr;
    PoseSeqItemPtr poseSeqItemPtr;
    PoseSeqPtr poseSeqPtr;
    if(!getSelectedPoseSeqSet(bodyItemPtr, body, poseSeqItemPtr, poseSeqPtr, mBodyMotionItemPtr)) return;
    BodyMotion& motion = *(mBodyMotionItemPtr->motion());

    mLogPath = boost::filesystem::path(poseSeqItemPtr->filePath());
    mLogPath = mLogPath.parent_path() / "log" / mLogPath.stem();
    cout << "LogPath: " << mLogPath << endl;

    std::vector<Link*> endEffectorLinkVec;
    if(!getEndEffectorLinkVector(endEffectorLinkVec, body)) return;

    generateOptionalData(body, poseSeqItemPtr, endEffectorLinkVec);

    // BodyMotion作成
    generateBodyMotionFromBar(body, poseSeqItemPtr, mBodyMotionItemPtr);

    frameRate = motion.frameRate();
    dt = 1.0/frameRate;
    numFrames = motion.numFrames();

    generateInitSeq(body, poseSeqItemPtr, endEffectorLinkVec);

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

    // sweepControl(mLogPath, childMcsLayout->getParamString(), childMcs, body, mBodyMotionItemPtr);// childモーション走査
    // sweepControl(mLogPath, parentMcsLayout->getParamString(), parentMcs, body, mBodyMotionItemPtr);// parentモーション走査


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
        loadExtraSeq(mLogPath ,childSfcLayout->getParamString(), childSfc, body, mBodyMotionItemPtr, contactLinkCandidateSet);
    }else{
        generateVerticalTrajectory(body, poseSeqItemPtr, contactLinkCandidateSet, childSfcLayout->takeoffPhaseRatioSpinArray->value(),childSfcLayout->landingPhaseRatioSpinArray->value());
        generatePreModelPredictiveControlParamDeque(parentSfc, bodyItemPtr, poseSeqItemPtr, contactLinkCandidateSet);
        {// add last param for interpolation
            SlideFrictionControlParam* lastSfcParam = (SlideFrictionControlParam*) parentSfc->preMpcParamDeque.back();
            SlideFrictionControlParam* tmpParam = new SlideFrictionControlParam(lastSfcParam->index()+1, parentSfc, lastSfcParam);
            parentSfc->preMpcParamDeque.push_back(tmpParam);
        }
        childSfc->pushAllPreMPCParamFromRoot();

        sweepControl(mLogPath, childSfcLayout->getParamString(), childSfc, body, mBodyMotionItemPtr, contactLinkCandidateSet);// childモーション走査
    }
    interpolateExtraSeq(mBodyMotionItemPtr, childSfc->dt);
    // sweepControl(mLogPath, parentSfcLayout->getParamString(), parentSfc, body, mBodyMotionItemPtr, contactLinkCandidateSet);// parentモーション走査

    generateTorque(body, poseSeqItemPtr, endEffectorLinkVec);

    cout << "\x1b[31m" << "Finished CascadedControl" << "\x1b[m" << endl;
}

CNOID_IMPLEMENT_PLUGIN_ENTRY(CascadedControlPlugin)
