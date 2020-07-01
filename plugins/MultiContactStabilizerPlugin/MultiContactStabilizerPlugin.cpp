/**
   @author Kunio Kojima
*/
#include "MultiContactStabilizerPlugin.h"

using namespace boost;
using namespace cnoid;
using namespace std;
using namespace hrp;

bool MultiContactStabilizerPlugin::initialize()
{
    mBar = new MultiContactStabilizerBar(this);
    addToolBar(mBar);
    return true;
}

void cnoid::sweepControl(boost::filesystem::path logPath, std::string paramStr, MultiContactStabilizer* mcs, BodyPtr& body, BodyMotionItemPtr& bodyMotionItemPtr)
{
    stringstream fnamess; fnamess.str("");
    fnamess << logPath.string() << "_MCS_refPL" << paramStr << "_" << (int) 1/mcs->rootController()->dt << "fps.dat";
    ofstream ofs;
    ofs.open( fnamess.str().c_str(), ios::out );

    const double dt = mcs->dt;
    const int numFrames = bodyMotionItemPtr->motion()->numFrames()*mcs->rootController()->dt/mcs->dt;

    float avgTime = 0;
    std::vector<int> failIdxVec;

    ofs << "time refCMx refCMy refCMz refPx refPy refPz refLx refLy refLz processTime" << endl;

    std::shared_ptr<Vector3Seq> refCMSeq = getOrCreateVector3SeqOfBodyMotionItem("refCM", bodyMotionItemPtr);
    std::shared_ptr<Vector3Seq> refPSeq = getOrCreateVector3SeqOfBodyMotionItem("refP", bodyMotionItemPtr);
    std::shared_ptr<Vector3Seq> refLSeq = getOrCreateVector3SeqOfBodyMotionItem("refL", bodyMotionItemPtr);

    for(int i=0; i < numFrames + mcs->numWindows(); ++i){
        // if(i > mcs->numWindows() + 1) goto BREAK;

        cout << endl << "##############################" << endl << "processCycle() turn:" << i << endl;
        float processedTime;
        if(mcs->processCycle(processedTime)) failIdxVec.push_back(i - mcs->numWindows());
        avgTime += processedTime;

        if(i >= mcs->numWindows()){
            VectorXd x0(mcs->stateDim);
            x0 = mcs->x0;

            Vector3d CM,P,L;
            CM << x0[0],x0[2],0;
            P << x0[1],x0[3],0;
            L << x0[4],x0[5],0;
            CM /= body->mass();
            refCMSeq->at(i - mcs->numWindows()) = CM;
            refPSeq->at(i - mcs->numWindows()) = P;
            refLSeq->at(i - mcs->numWindows()) = L;
            ofs << (i - mcs->numWindows())*dt << " " << CM.transpose() <<  " " << P.transpose() << " " << L.transpose() << " " << processedTime << endl;
        }
    }
    // {// for final frame's SeqPtr
    //     int finalMotionIdx = numFrames*cycle;
    //     refCMSeqPtr->at(finalMotionIdx) = refCMSeqPtr->at(finalMotionIdx-cycle);
    //     refPSeqPtr->at(finalMotionIdx) = refPSeqPtr->at(finalMotionIdx-cycle);
    //     refLSeqPtr->at(finalMotionIdx) = refLSeqPtr->at(finalMotionIdx-cycle);
    // }
 // BREAK:


    ofs.close();

    for(std::vector<int>::iterator iter = failIdxVec.begin(); iter != failIdxVec.end(); ++iter) cout << "Failed in " << *iter << ":(" << (*iter)*dt << " sec)" << endl;
    if(failIdxVec.empty()) cout << "All QP succeeded" << endl;
    failIdxVec.clear();

    cout << "total time: " << avgTime/1000.0 << "[sec]" << endl;
    cout << "average time: " << avgTime/numFrames << "[msec]" << endl;
}

namespace{

void generateContactConstraintParamVec(std::vector<ContactConstraintParam*>& ccParamVec, const std::set<Link*>& contactLinkCandidateSet, PoseSeq::iterator poseIter, const PoseSeqPtr& poseSeqPtr)
{
    for(std::set<Link*>::iterator linkIter = contactLinkCandidateSet.begin(); linkIter != contactLinkCandidateSet.end(); ++linkIter){
        int linkIdx = (*linkIter)->index();
        int contactState = getPrevContactState(poseIter, poseSeqPtr, linkIdx);
        if(contactState < 2){// 接触フラグが0か1 要改良
            std::vector<hrp::Vector3> edgeVec;
            hrp::Vector3 edge;// 要改良 直線の係数a,b,cを代入
            edge << -1, 0, 0.1;
            edgeVec.push_back(edge);
            edge << 1, 0, 0.1;
            edgeVec.push_back(edge);
            edge << 0, -1, 0.05;
            edgeVec.push_back(edge);
            edge << 0, 1, 0.05;
            edgeVec.push_back(edge);

            ContactConstraintParam* ccParam;
            if(contactState == 0){// static contact
                // ccParam = (SimpleContactConstraintParam*) new SimpleContactConstraintParam((*linkIter)->name(), edgeVec);
                ccParam = (StaticContactConstraintParam*) new StaticContactConstraintParam((*linkIter)->name(), edgeVec, 0.5);// 摩擦係数 要改良
            }else if(contactState == 1){// slide contact
                ccParam = (SlideContactConstraintParam*) new SlideContactConstraintParam((*linkIter)->name(), edgeVec, 0.5, getPrevDirection(poseIter, poseSeqPtr, linkIdx));
            }
            ccParamVec.push_back(ccParam);
        }
    }
}

void generateMultiContactStabilizerParam(MultiContactStabilizerParam* mcsParam, Vector3d& lastP,  BodyPtr body, std::vector<ContactConstraintParam*>& ccParamVec, double dt)
{

    static Vector3 g;
    g << 0,0,9.8;

    // CM,P,L
    Vector3d CM,P,L,F;
    CM = body->calcCenterOfMass();
    body->calcTotalMomentum(P,L);
    L -= CM.cross(P);// convert to around CoM
    mcsParam->CM = CM;
    mcsParam->P = P;
    // mcsParam->L = L;
    mcsParam->L << 0,0,0;
    mcsParam->F = (P - lastP)/dt + body->mass()*g;

    // 接触点座標系の更新 等式と不等式数の合計
    for(std::vector<ContactConstraintParam*>::iterator iter = ccParamVec.begin(); iter != ccParamVec.end(); ++iter){
        (*iter)->p = body->link((*iter)->linkName)->p();
        (*iter)->R =  body->link((*iter)->linkName)->R();
    }
    mcsParam->ccParamVec = ccParamVec;

    lastP = P;
}

}

void cnoid::generatePreModelPredictiveControlParamDeque(MultiContactStabilizer* mcs, BodyPtr body, const PoseSeqPtr poseSeqPtr, const BodyMotion& motion, const std::set<Link*>& contactLinkCandidateSet)
{
    const int frameRate = motion.frameRate();
    const int numFrames = motion.numFrames();
    const double dt = 1.0/motion.frameRate();
    Vector3d lastP, tmpL;
    updateBodyState(body, motion, 0);
    body->calcForwardKinematics(true, true);
    body->calcCenterOfMass();
    body->calcTotalMomentum(lastP,tmpL);

    // cnoidのクラス(BodyMotion)からmpcParamDequeを生成
    int index = 0;
    for(PoseSeq::iterator frontPoseIter = (++poseSeqPtr->begin()),backPoseIter = poseSeqPtr->begin(); frontPoseIter != poseSeqPtr->end(); incContactPose(frontPoseIter,poseSeqPtr,body)){
        if(!isContactStateChanging(frontPoseIter, poseSeqPtr, body)) continue;

        // 接触状態依存のパラメータのみ設定(動作軌道に依存するパラメータは後で設定)
        std::vector<ContactConstraintParam*> ccParamVec;
        ::generateContactConstraintParamVec(ccParamVec, contactLinkCandidateSet, frontPoseIter, poseSeqPtr);

        for(int i=backPoseIter->time()*frameRate; i < frontPoseIter->time()*frameRate; ++i){
            updateBodyState(body, motion, min(i,numFrames-1));
            body->calcForwardKinematics(true, true);

            MultiContactStabilizerParam* mcsParam = new MultiContactStabilizerParam(index, mcs);
            // 動作軌道に依存するパラメータの設定
            ::generateMultiContactStabilizerParam(mcsParam, lastP, body, ccParamVec, dt);

            mcs->preMpcParamDeque.push_back((MultiContactStabilizerParam*) mcsParam);
            ++index;
        }

        backPoseIter = frontPoseIter;
    }
}

void MultiContactStabilizerPlugin::execControl()
{
    stringstream ss,fnamess;
    MessageView::instance()->putln("MultiContactStabilizer called !");
    cout << "MultiContactStabilizer()" << endl;

    BodyItemPtr bodyItemPtr;
    PoseSeqItemPtr poseSeqItemPtr;
    PoseSeqPtr poseSeqPtr;
    if(!getSelectedPoseSeqSet(bodyItemPtr, body, poseSeqItemPtr, poseSeqPtr, mBodyMotionItemPtr)) return;
    BodyMotion& motion = *(mBodyMotionItemPtr->motion());

    mLogPath = getLogPath( boost::filesystem::path(poseSeqItemPtr->filePath()) );
    cout << "LogPath: " << mLogPath << endl;

    // BodyMotion作成
    generateBodyMotionFromBar(body, poseSeqItemPtr, mBodyMotionItemPtr);

    frameRate = motion.frameRate();
    dt = 1.0/frameRate;
    numFrames = motion.numFrames();

    std::vector<Link*> dummyVec;
    generateInitSeq(body, poseSeqItemPtr, dummyVec);

    // 接触候補セットの作成
    std::set<Link*> contactLinkCandidateSet;
    calcContactLinkCandidateSet(contactLinkCandidateSet, body, poseSeqPtr);

    mcs = new MultiContactStabilizer();
    mcs->m = body->mass();
    mcs->dt = dt;
    mcs->setBlockVector((mBar->dialog->layout())->blockSpinVec->value());
    mcs->errorCMWeight = (mBar->dialog->layout())->errorCMWeightSpin->value();
    mcs->errorMomentumWeight = (mBar->dialog->layout())->errorMomentumWeightSpin->value();
    mcs->errorAngularMomentumWeight = (mBar->dialog->layout())->errorAngularMomentumWeightSpin->value();
    mcs->inputForceWeight = (mBar->dialog->layout())->inputForceWeightSpin->value();
    mcs->inputMomentWeight = (mBar->dialog->layout())->inputMomentWeightSpin->value();

    generatePreModelPredictiveControlParamDeque(mcs, body, poseSeqPtr, motion, contactLinkCandidateSet);

    sweepControl(mLogPath, mBar->dialog->layout()->getParamString(), mcs, body, mBodyMotionItemPtr);

    cout << "Finished MultiContactStabilizer" << endl;
}

CNOID_IMPLEMENT_PLUGIN_ENTRY(MultiContactStabilizerPlugin)
