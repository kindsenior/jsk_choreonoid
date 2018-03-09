/**
   @author Kunio Kojima
*/
#include "RMControlPlugin.h"

#define BASE_LINK "LLEG_JOINT5"

using namespace boost;
using namespace cnoid;
using namespace std;

bool RMControlPlugin::initialize()
{
    mBar = new RMControlBar(this);
    addToolBar(mBar);

    return true;
}

namespace {
// スプライン補間
void splineInterpolation(Vector3d& a0, Vector3d& a1, Vector3d& a2, Vector3d& a3,
                         const Vector3d f0, const Vector3d v0, const Vector3d f1, const Vector3d v1, const double tau)
{
    a0 = f0;
    a1 = v0;
    MatrixXd A(2,3);
    Matrix2d T;
    T <<
        pow(tau, 2), pow(tau, 3),
        2 * tau, 3 * pow(tau, 2);
    MatrixXd P(2,3);
    P.row(0) = f1.transpose() - f0.transpose()  - v0.transpose() * tau;
    P.row(1) = v1.transpose() - v0.transpose();
    A = T.inverse() * P;
    a2 = A.block(0,0, 1,3).transpose();
    a3 = A.block(1,0, 1,3).transpose();

}

void minjerkInterpolation(Vector3d& a0, Vector3d& a1, Vector3d& a2, Vector3d& a3, Vector3d& a4, Vector3d& a5,
                          const Vector3d& x0, const Vector3d& dx0, const Vector3d& ddx0, const Vector3d& x1, const Vector3d& dx1, const Vector3d& ddx1, const double& T)
{
    a0 = x0;
    a1 = dx0;
    a2 = ddx0/2;

    const double T2 = pow(T,2), T3 = pow(T,3), T4 = pow(T,4), T5 = pow(T,5);

    a3 = ddx1/(2*T)  - 3*ddx0/(2*T)  - 4*dx1/T2 - 6*dx0/T2 + 10*x1/T3 - 10*x0/T3;
    a4 = -ddx1/T2    + 3*ddx0/(2*T2) + 7*dx1/T3 + 8*dx0/T3 - 15*x1/T4 + 15*x0/T4;
    a5 = ddx1/(2*T3) - ddx0/(2*T3)   - 3*dx1/T4 - 3*dx0/T4 + 6*x1/T5  - 6*x0/T5;
}

}

// ロボットモデル依存の部分あり
// 各種行列を計算
void RMControlPlugin::calcMatrixies(MatrixXd& A_, MatrixXd& M, MatrixXd& H)
{
    stringstream ss;

    // 重心
    // const Vector3d currentCM = mBody->calcCenterOfMass();

    // Ml Mr Hl Hr作成
    // MatrixXd M,H;
    calcCMJacobian(mBody,NULL,M);
    calcAngularMomentumJacobian(mBody, NULL, H);
    M = mBody->mass() * M;
    H.block(0,mBody->numJoints(), 3,3) = MatrixXd::Zero(3,3);
    H.block(0,mBody->numJoints()+3, 3,3) = mSubMasses[mBody->rootLink()->index()].Iw;
    MatrixXd MH(M.rows()+H.rows(), M.cols());
    MH.block(0,0, M.rows(),M.cols()) = M; MH.block(M.rows(),0, H.rows(),H.cols()) = H;

    // Link* rootLink = mBody->rootLink();
    // Link* lFoot = mBody->link("RLEG_JOINT5"); Link* rFoot = mBody->link("LLEG_JOINT5");
    // JointPathPtr mJpl = getCustomJointPath(mBody, rootLink, lFoot);
    // JointPathPtr mJpr = getCustomJointPath(mBody, rootLink, rFoot);

    // Hlleg = H.block(0,mBody->link("LLEG_JOINT0")->jointId, 3,mJpl->numJoints());
    // Hrleg = H.block(0,mBody->link("RLEG_JOINT0")->jointId, 3,mJpr->numJoints());

    // Mfree, Hfree作成
    // freeジョイントがない場合の処理にはおそらく未対応
    // free関節かどうか

    // Fl,Fr作成




    // A_を計算
    A_ = MatrixXd(MH.cols(), wholeBodyConstraintPtr->complementJointIdSet().size());
    A_ = extractMatrix(MH, wholeBodyConstraintPtr->complementJointIdSet());
    for(auto jointPathPtr : wholeBodyConstraintPtr->constraintJointPathVec){
        A_ -= extractMatrix(MH, jointPathPtr->exclusiveJointIdSet()) * inverseJacobian((JointPathPtr&) jointPathPtr) * extractMatrix(jointPathPtr->jacobianWholeBody(), wholeBodyConstraintPtr->complementJointIdSet());
    }

    // Hlleg = H.block(0,mBody->link("LLEG_JOINT0")->jointId, 3,mJpl->numJoints());
    // Hrleg = H.block(0,mBody->link("RLEG_JOINT0")->jointId, 3,mJpr->numJoints());

    // MessageView::instance()->putln(ss.str());
    return;
}

// 目標運動量・角運動量軌道生成
// void generateRefPLSeq(BodyPtr body,BodyItem* bodyItem, const BodyMotionPtr motion,const PoseSeqPtr poseSeq,
void RMControlPlugin::generateRefPLSeq(BodyMotionItem* motionItem ,const PoseSeqPtr poseSeq,
                                       const Vector3d initDCM, const Vector3d endDCM, const Vector3d initL, const Vector3d endL)
{
    const BodyMotionPtr motion = motionItem->motion();

    Vector3SeqPtr refCMSeqPtr = motionItem->motion()->getOrCreateExtraSeq<Vector3Seq>("refCM");
    Vector3SeqPtr refPSeqPtr = motionItem->motion()->getOrCreateExtraSeq<Vector3Seq>("refP");
    Vector3SeqPtr refLSeqPtr = motionItem->motion()->getOrCreateExtraSeq<Vector3Seq>("refL");

    stringstream ss,fnamess;
    fnamess << mPoseSeqPath.parent_path().string() << "/" << getBasename(mPoseSeqPath) << "_RMC_refPL_" << motion->frameRate() << "fps.dat";
    ofstream ofs(fnamess.str().c_str());
    ofs << "time refCMx refCMy refCMz refPx refPy refPz refLx refLy refLz" << endl;

    fnamess.str("");
    fnamess << mPoseSeqPath.parent_path().string() << "/" << getBasename(mPoseSeqPath) << "_RMC_initPL_" << motion->frameRate() << "fps.dat";
    ofstream ofs1(fnamess.str().c_str());
    ofs1 << "time initCMx initCMy initCMz initPx initPy initPz initLx initLy initLz" << endl;

    const double dt = 1.0/motion->frameRate();
    const double g = 9.80665;
    const double m = mBody->mass();

    // 離着陸Pose設定
    PoseSeq::iterator takeoffPoseItr = poseSeq->begin();
    PoseSeq::iterator landingPoseItr = poseSeq->begin();
    PoseSeq::iterator initRMCPoseItr = poseSeq->begin();
    PoseSeq::iterator endRMCPoseItr = poseSeq->begin();
    bool prevTouchFlg = true;
    for(PoseSeq::iterator poseItr = poseSeq->begin(); poseItr != poseSeq->end(); ++poseItr){
        bool TouchFlg = false;

        // Poseの接地判定
        Pose::LinkInfoMap::iterator itr = poseItr->get<Pose>()->ikLinkBegin();
        for(int i = 0; i <= 10; ++i){// 10番目まで調べる
            TouchFlg |= itr->second.isTouching() && !itr->second.isBaseLink();
            ++itr;
        }

        // Poseの離着陸判定
        if(prevTouchFlg && !TouchFlg){
            takeoffPoseItr = --poseItr;
            initRMCPoseItr = --poseItr;
            ++poseItr;++poseItr;
        }
        if(!prevTouchFlg && TouchFlg){
            landingPoseItr = poseItr;
            endRMCPoseItr = ++poseItr;
            --poseItr;
        }

        prevTouchFlg = TouchFlg;
    }


    // 離着陸Frame設定
    const int takeoffFrame = takeoffPoseItr->time() * motion->frameRate();
    const int landingFrame = landingPoseItr->time() * motion->frameRate();
    const int initRMCFrame = initRMCPoseItr->time() * motion->frameRate();
    const int endRMCFrame = endRMCPoseItr->time() * motion->frameRate();
    const double jumptime = (landingFrame - takeoffFrame) * dt;
    cout << "initRMCFrame " << initRMCFrame << endl;
    cout << "takeoffFrame " << takeoffFrame << endl;
    cout << "landingFrame " << landingFrame << endl;
    cout << "endRMCFrame " << endRMCFrame << endl;
    cout << "jumptime " << jumptime << endl;


    // 離着陸CM
    (BodyMotion::ConstFrame) motion->frame(takeoffFrame) >> *mBody;
    mBody->calcForwardKinematics();
    const Vector3d takeoffCM = mBody->calcCenterOfMass();
    (BodyMotion::ConstFrame) motion->frame(landingFrame) >> *mBody;
    mBody->calcForwardKinematics();
    const Vector3d landingCM = mBody->calcCenterOfMass();

    // 離陸運動量
    Vector3d takeoffDCM = (landingCM - takeoffCM) / jumptime;
    Vector3d landingDCM = takeoffDCM;
    takeoffDCM.z() = 0.5 * g * jumptime + (landingCM.z() - takeoffCM.z()) / jumptime;
    landingDCM.z() = takeoffDCM.z() - g * jumptime;

    // 跳躍期角運動量
    Vector3d jumpL = Vector3d::Zero();// 重心回り

    // 初期終端重心
    (BodyMotion::ConstFrame) motion->frame(initRMCFrame) >> *mBody;
    mBody->calcForwardKinematics();
    const Vector3d initCM = mBody->calcCenterOfMass();
    (BodyMotion::ConstFrame) motion->frame(endRMCFrame) >> *mBody;
    mBody->calcForwardKinematics();
    const Vector3d endCM = mBody->calcCenterOfMass();

    cout << endl;
    cout << "takeoffCM: " << takeoffCM.transpose() << endl;
    cout << "landingCM: " << landingCM.transpose() << endl;
    cout << "initCM: "  << initCM.transpose() << endl;
    cout << "endCM: " << endCM.transpose() << endl;
    cout << endl;

    // 目標重心位置
    for(int i = 0; i < motion->numFrames(); ++i){
        (BodyMotion::ConstFrame) motion->frame(i) >> *mBody;
        mBody->calcForwardKinematics();
        refCMSeqPtr->at(i) = mBody->calcCenterOfMass();

        Vector3d v,w;
        VectorXd dq,ddq;
        calcDifferential(motion, i, v, w, dq, ddq);
        mBody->rootLink()->v() = v;
        mBody->rootLink()->w() = w;
        Vector3d P,L;
        calcSubMass(mBody->rootLink(), mSubMasses);// リンク先重心・慣性行列更新
        calcTotalMomentum(P, L, mBody, mSubMasses[mBody->rootLink()->index()].Iw, dq);

        ofs1 << dt*i;
        ofs1 << " " << refCMSeqPtr->at(i).transpose();//重心 2,3,4
        ofs1 << " " << P.transpose();// 運動量 5,6,7
        ofs1 << " " << L.transpose();// 角運動量 8,9,10
        ofs1 << endl;
        if(i == 1000)cout << "refCM " << refCMSeqPtr->at(i).transpose() << endl;
    }

    // 目標運動量 motion loop
    Vector3d a0,a1,a2,a3,a4,a5;
    std::vector<Vector3d> a;
    a.resize(6);
    int startFrame,endFrame;
    for(int i = 0; i < motion->numFrames(); ++i){

        // 角運動量
        refLSeqPtr->at(i) = Vector3d::Zero(); // overwrite angular momentum to 0

        // 運動量
        if(takeoffFrame < i && i < landingFrame){// 跳躍期
            refCMSeqPtr->at(i) = takeoffCM + (landingCM - takeoffCM) * (i-takeoffFrame)*dt /jumptime;
            refCMSeqPtr->at(i).z() = - 0.5 * g * (i-takeoffFrame)*dt * (i-landingFrame)*dt
                + (landingCM.z()* (i-takeoffFrame)*dt - takeoffCM.z()* (i-landingFrame)*dt) / jumptime;
            refPSeqPtr->at(i) = m * takeoffDCM;
            refPSeqPtr->at(i).z() = - m * g * (i - takeoffFrame)*dt + m * takeoffDCM.z();
        }else if(initRMCFrame <= i && i <= endRMCFrame){// 接地期
            // Vector3d f0,v0,f1,v1;
            // if(i <= takeoffFrame){
            //     startFrame = initRMCFrame;
            //     endFrame = takeoffFrame;
            //     f0 = initCM; v0 = initDCM;
            //     f1 = takeoffCM; v1 = takeoffDCM;
            // }else if(landingFrame <= i){
            //     startFrame = landingFrame;
            //     endFrame = endRMCFrame;
            //     f0 = landingCM; v0 = landingDCM;
            //     f1 = endCM; v1 = endDCM;
            // }
            // double tau = (endFrame - startFrame) * dt;

            // スプライン補間/躍度最小補間
            if(i == initRMCFrame){
                startFrame = initRMCFrame;
                endFrame = takeoffFrame;
                // splineInterpolation(a0,a1,a2,a3, initCM,initDCM, takeoffCM,takeoffDCM, (takeoffFrame-initRMCFrame)*dt);
                setCubicSplineInterpolation(a, initCM,initDCM, takeoffCM,takeoffDCM, (takeoffFrame-initRMCFrame)*dt);
            }else if(i == landingFrame){
                startFrame = landingFrame;
                endFrame = endRMCFrame;
                // splineInterpolation(a0,a1,a2,a3, landingCM,landingDCM, endCM,endDCM, (endRMCFrame-landingFrame)*dt);
                setCubicSplineInterpolation(a, landingCM,landingDCM, endCM,endDCM, (endRMCFrame-landingFrame)*dt);
                // minjerkInterpolation(a0,a1,a2,a3,a4,a5, landingCM,landingDCM,Vector3d(0,0,-g), endCM,endDCM,Vector3d::Zero(3), (endRMCFrame-landingFrame)*dt);
            }

            // if(i <= takeoffFrame){
            if(i <= takeoffFrame || true){
                // refCMSeqPtr->at(i) = a0 + a1 * (i-startFrame)*dt + a2 * pow ( (i-startFrame)*dt , 2 ) + a3 * pow( (i-startFrame)*dt, 3 );
                // refPSeqPtr->at(i) = ( a1 + 2 * a2 * (i-startFrame)*dt + 3 * a3 * pow ( (i-startFrame)*dt , 2 ) ) * m;
                refCMSeqPtr->at(i) = a[0] + a[1] * (i-startFrame)*dt + a[2] * pow ( (i-startFrame)*dt , 2 ) + a[3] * pow( (i-startFrame)*dt, 3 );
                refPSeqPtr->at(i) = ( a[1] + 2 * a[2] * (i-startFrame)*dt + 3 * a[3] * pow ( (i-startFrame)*dt , 2 ) ) * m;
            }else{
                double dT = (i-startFrame)*dt;
                double dT2 = pow(dT,2), dT3 = pow(dT,3), dT4 = pow(dT,4), dT5 = pow(dT,5);
                refCMSeqPtr->at(i) = a[0] + a[1]*dT + a[2]*dT2 + a[3]*dT3 + a[4]*dT4 + a[5]*dT5;
                refPSeqPtr->at(i) = (a[1] + 2*a[2]*dT + 3*a[3]*dT2 + 4*a[4]*dT3 + 5*a[5]*dT4) * m;
            }

            // 境界条件表示
            if(i == initRMCFrame || i == landingFrame){
                cout << "time " << i*dt << endl;
                // cout << "f0 " << f0.transpose() << endl;
                cout << "refCMSeq " << refCMSeqPtr->at(i).transpose() << endl;
                // cout << "v0 " << v0.transpose() << endl;
                cout << "refPSeq " << refPSeqPtr->at(i).transpose() << endl;
                cout << endl;
            }else if(i == takeoffFrame || i == endRMCFrame){
                cout << "time " << i*dt << endl;
                // cout << "f1 " << f1.transpose() << endl;
                cout << "refCMSeq " << refCMSeqPtr->at(i).transpose() << endl;
                // cout << "v1 " << v1.transpose() << endl;
                cout << "refPSeq " << refPSeqPtr->at(i).transpose() << endl;
                cout << endl;
            }

        }else{// 非制御期
            int nextFrame = std::min(i + 1, motion->numFrames() - 1);
            refPSeqPtr->at(i) = m * (refCMSeqPtr->at(nextFrame) - refCMSeqPtr->at(i)) /dt;
        }

        ofs << i*dt ;
        ofs <<  " " << refCMSeqPtr->at(i).transpose();// 重心 2,3,4
        ofs <<  " " << refPSeqPtr->at(i).transpose();// 運動量 5,6,7
        ofs <<  " " << refLSeqPtr->at(i).transpose();// 角運動量 8,9,10
        ofs << endl;

    }// end motion loop

    setSubItem("refCM", refCMSeqPtr, motionItem);
    setSubItem("refP", refPSeqPtr, motionItem);
    setSubItem("refL", refLSeqPtr, motionItem);

    // MessageView::instance()->putln(ss.str());
    ofs.close();
}

void RMControlPlugin::loadRefPLSeq(BodyMotionItem* motionItem ,const PoseSeqPtr poseSeq,
                                   const Vector3d initP, const Vector3d endP, const Vector3d initL, const Vector3d endL)
{
    const BodyMotionPtr motion = motionItem->motion();

    stringstream ss,fnamess;
    fnamess << mPoseSeqPath.parent_path().string() << "/" << getBasename(mPoseSeqPath) << "_RMC_refPL_" << motion->frameRate() << "fps.dat";
    ofstream ofs(fnamess.str().c_str());
    ofs << "time refCMx refCMy refCMz refPx refPy refPz refLx refLy refLz" << endl;
    const double dt = 1.0/motion->frameRate();
    const double g = 9.80665;
    const double m = mBody->mass();

    // 目標重心位置
    Vector3SeqPtr refCMSeqPtr = motionItem->findSubItem<Vector3SeqItem>("refCM")->seq();
    Vector3SeqPtr refPSeqPtr = motionItem->findSubItem<Vector3SeqItem>("refP")->seq();
    Vector3SeqPtr refLSeqPtr = motionItem->findSubItem<Vector3SeqItem>("refL")->seq();

    // 目標運動量 motion loop
    for(int i = 0; i < motion->numFrames(); ++i){
        // refLSeq[i] = Vector3d::Zero();
        // refLSeqPtr->at(i) = refLSeqPtr->at(i);

        int nextFrame = std::min( i + 1, motion->numFrames() - 1 );
        // refPSeq[i] = ( refCMSeqPtr->at(nextFrame) - refCMSeqPtr->at(i) ) / dt;
        // refPSeqPtr->at(i) = refPSeqPtr->at(i);

        ofs << i*dt;
        ofs << " " << refCMSeqPtr->at(i).transpose();// 重心 2,3,4
        ofs << " " << refPSeqPtr->at(i).transpose();// 運動量 5,6,7
        ofs << " " << refLSeqPtr->at(i).transpose();// 角運動量 8,9,10
        ofs << endl;

    }// end motion loop

    // setSubItem("refP", refPSeqPtr, motionItem);
    // setSubItem("refL", refLSeqPtr, motionItem);

    MessageView::instance()->putln(ss.str());
    ofs.close();
}

void RMControlPlugin::generateRefWrenchSeq(BodyPtr& body, PoseSeqItemPtr& poseSeqItem, const std::vector<Link*>& endEffectorLinkVec)
{
    PoseSeqPtr poseSeq = poseSeqItem->poseSeq();
    BodyMotionItemPtr bodyMotionItem = poseSeqItem->bodyMotionItem();
    BodyMotionPtr motion = bodyMotionItem->motion();

    int numLinks = endEffectorLinkVec.size();
    MultiValueSeqPtr refWrenchesSeqPtr = bodyMotionItem->motion()->getOrCreateExtraSeq<MultiValueSeq>("wrenches");
    refWrenchesSeqPtr->setNumParts(6*numLinks,true);

    Vector3SeqPtr refPSeqPtr = bodyMotionItem->findSubItem<Vector3SeqItem>("refP")->seq();

    const int numFrames = motion->numFrames();
    const int frameRate = motion->frameRate();
    const double dt = 1.0/motion->frameRate();
    const double g = 9.80665;
    const double m = mBody->mass();
    for(PoseSeq::iterator frontPoseIter = (++poseSeq->begin()),backPoseIter = poseSeq->begin(); frontPoseIter != poseSeq->end(); incContactPose(frontPoseIter,poseSeq,mBody)){
        if(!isContactStateChanging(frontPoseIter, poseSeq, mBody)) continue;

        std::vector<int> contactStateVec;
        int numContactLinks = 0;
        for(auto endEffectorLink : endEffectorLinkVec){
            int contactState = getPrevContactState(frontPoseIter, poseSeq, endEffectorLink->index());
            contactStateVec.push_back(contactState);
            if(contactState < 2) ++numContactLinks;
        }

        for(int i=backPoseIter->time()*frameRate; (i < frontPoseIter->time()*frameRate) && (i < numFrames); ++i){
            MultiValueSeq::Frame wrenches = refWrenchesSeqPtr->frame(i);
            int idx = 0;
            Vector3d f = (refPSeqPtr->at(i+1) - refPSeqPtr->at(i))/dt + m*Vector3d(0,0,g);
            for(int contactState: contactStateVec){// in the order of endEffectorLinkVec
                switch (contactState){
                case 0: // stopped contact foot
                    for(int count=0; count < 3; ++count,++idx) wrenches[idx] = f(count)/numContactLinks; // uniform force distribution
                    for(int count=0; count < 3; ++count,++idx) wrenches[idx] = 0;
                    break;
                case 1: // slide foot
                    cout << "\x1b[31m" << "RMC does not support slide contacts" << "\x1b[m" << endl;
                    break;
                case 2: // swing stoped foot (same with swing foot)
                    // break;
                case 3:// swing foot
                    for(int count=0; count<6; ++count,++idx) wrenches[idx] = 0;
                    break;
                default:
                    cout << "\x1b[31m" << "The contact type not supported in RMC" <<  "\x1b[m" << endl;
                    break;
                }
            }
            refWrenchesSeqPtr->frame(i) = wrenches;
        }

        backPoseIter = frontPoseIter;
    }
    refWrenchesSeqPtr->frame(numFrames) = refWrenchesSeqPtr->frame(numFrames-1);// final frame
    setSubItem("wrenches", refWrenchesSeqPtr, bodyMotionItem);
}

void RMControlPlugin::modifyJumpingTrajectory(PoseSeqItemPtr& poseSeqItem, const std::set<Link*>& contactLinkCandidateSet)
{
    PoseSeqPtr poseSeq = poseSeqItem->poseSeq();
    BodyMotionPtr motion = poseSeqItem->bodyMotionItem()->motion();

    Vector3SeqPtr refCMSeqPtr = poseSeqItem->bodyMotionItem()->findSubItem<Vector3SeqItem>("refCM")->seq();

    const int frameRate = motion->frameRate();
    // const int numFrames = motion->numFrames();
    const double dt = 1.0/motion->frameRate();

    for(PoseSeq::iterator frontPoseIter = (++poseSeq->begin()),backPoseIter = poseSeq->begin(); frontPoseIter != poseSeq->end(); incContactPose(frontPoseIter,poseSeq,mBody)){
        if(!isContactStateChanging(frontPoseIter, poseSeq, mBody)) continue;

        std::set<Link*> swingLinkSet;
        getNextTargetContactLinkSet(swingLinkSet, mBody, 3, contactLinkCandidateSet, backPoseIter, poseSeq);
        if(swingLinkSet.size() > 0){
            cout << "swing phase: " << backPoseIter->time() << " -> " << frontPoseIter->time() << " [sec]" << endl;
            for(int i=backPoseIter->time()*frameRate; i < frontPoseIter->time()*frameRate; ++i){
                (BodyMotion::ConstFrame) motion->frame(i) >> *mBody;
                mBody->calcForwardKinematics();
                Vector3d initCM = mBody->calcCenterOfMass();
                Vector3d refCM = refCMSeqPtr->at(i);
                // Vector3d rootPos = mBody->rootLink()->p();
                // for(auto swingLink: swingLinkSet){
                //     Link* targetLink = mBody->link(swingLink->name());
                //     JointPathPtr jointPath = getCustomJointPath(mBody, mBody->rootLink(), targetLink);
                //     jointPath->calcInverseKinematics(targetLink->p() - rootPos + refCM, targetLink->R());
                // }
                mBody->rootLink()->p() += refCM - initCM;
                mBody->calcForwardKinematics();
                motion->frame(i) << *mBody;
            }
        }
        backPoseIter = frontPoseIter;
    }
}

void RMControlPlugin::sweepControl(boost::filesystem::path poseSeqPath ,std::string paramStr, BodyPtr& body, BodyMotionItemPtr& bodyMotionItem, const std::vector<Link*>& endEffectorLinkVec)
{
    BodyMotionPtr motion = bodyMotionItem->motion();

    Vector3SeqPtr refPSeqPtr = bodyMotionItem->findSubItem<Vector3SeqItem>("refP")->seq();
    Vector3SeqPtr refLSeqPtr = bodyMotionItem->findSubItem<Vector3SeqItem>("refL")->seq();

    stringstream ss,fnamess;

    // 選択行列作成 step 1
    VectorXd SVec(6); SVec << 1,1,1, 1,1,0;
    const int numSelects = (SVec.array() != 0).count();
    MatrixXd S(numSelects,6);
    for(int i = 0; i < numSelects; ++i){
        if(SVec(i) != 0) S.row(i) = SVec(i)*MatrixXd::Identity(6,6).row(i);
    }
    cout << " Finished Step 1: setting select array" << endl;

    // for(int i = 0; i < 3; ++i){
    for(int i = 0; i < 1; ++i){
        fnamess.str("");
        fnamess << mPoseSeqPath.parent_path().string() << "/" << getBasename(mPoseSeqPath) << "_RMC_PL_" << motion->frameRate() << "fps_" << i << ".dat";
        ofstream ofs( fnamess.str().c_str() );
        if(!ofs){ cerr << "File Open Error: " << fnamess << endl; return;}
        ofs << "time CMx CMy CMz Px Py Pz Lx Ly Lz" << endl;

        fnamess.str("");
        fnamess << mPoseSeqPath.parent_path().string() << "/" << getBasename(mPoseSeqPath) << "_RMC_EE_" << motion->frameRate() << "fps_" << i << ".dat";
        ofstream eeOfs( fnamess.str().c_str() );
        if(!eeOfs){ cerr << "File Open Error: " << fnamess << endl; return;}
        eeOfs << "time";
        std::vector<string> columnHeadKeyVec={"px","py","pz", "vx","vy","vz", "wx","wy","wz"};
        for(auto link : endEffectorLinkVec) for(auto columnHeadKey : columnHeadKeyVec) eeOfs << " " << link->name() << columnHeadKey;
        eeOfs << endl;

        double dt = 1.0/motion->frameRate();

        // 運動量ヤコビアンが正しいかどうかの確認
        (BodyMotion::ConstFrame) motion->frame(0) >> *mBody;// 腰座標、q更新 body更新

        VectorXd dq_(mBody->numJoints());
        MultiValueSeq::Frame q = motion->jointPosSeq()->frame(0);
        for(int k=0; k < motion->numJoints(); ++k){
            Link* joint = mBody->joint(k);
            joint->q() = q[k];
            joint->dq() = -0.3;
            joint->ddq() = 0;
            dq_[k] = joint->dq();
        }
        mBody->calcForwardKinematics(true,true);// v,wを更新

        mBody->rootLink()->v() = Vector3d::Zero();
        mBody->rootLink()->w() << 0.1, 0.1, 0.1;
        // mBody->rootLink()->w() << 0, 0, 0;
        mBody->calcForwardKinematics(true,true);

        mBody->calcCenterOfMass();// wcを更新

        Vector3 P_world;Vector3 L_world;// 運動量、角運動量(世界座標原点)
        mBody->calcTotalMomentum(P_world,L_world);
        Vector3 L_g,P_g;// 運動量ヤコビアン、角運動量ヤコビアン(重心基準)
        calcSubMass( mBody->rootLink(), mSubMasses);
        calcTotalMomentum(P_g, L_g, mBody, mSubMasses[mBody->rootLink()->index()].Iw, dq_);

        cout << "correct P " << P_world.transpose() << endl;
        cout << "my P " << P_g.transpose() << endl;
        cout << "correct L " << L_world.transpose() << endl;
        cout << "my L " << (mBody->centerOfMass().cross(P_g) + L_g).transpose() << endl << endl;// 原点回りに変換


        // // 脚のヤコビアンが正しいかどうかの確認
        // motion->frame(0) >> *body;// 腰座標、q更新 body更新
        // for(int k=0; k < motion->numJoints(); ++k){
        //   Link* joint = mBody->joint(k);
        //   joint->dq() = 0.1;
        //   joint->ddq() = 0;
        // }
        // mBody->calcForwardKinematics(true,true);// v,wを更新
        // MatrixXd Jl_,Jr_; mJpl->calcJacobian(Jl_); mJpr->calcJacobian(Jr_);// 脚・腰間ヤコビアン
        // VectorXd dql_ = VectorXd::Constant(6,0.1);
        // VectorXd dqr_ = VectorXd::Constant(6,0.1);
        // VectorXd xil_(6);        VectorXd xir_(6);
        // xil_.block(0,0, 3,1) = mBody->link("LLEG_JOINT5")->v; xil_.block(3,0, 3,1) = mBody->link("LLEG_JOINT5")->w();
        // xir_.block(0,0, 3,1) = mBody->link("RLEG_JOINT5")->v; xir_.block(3,0, 3,1) = mBody->link("RLEG_JOINT5")->w();
        // ss << "lleg xi " << endl << xil_.transpose() << endl;
        // ss << "lleg xi from J" << endl << (Jl_*dql_).transpose() << endl;
        // ss << "lleg xi " << endl << xir_.transpose() << endl;
        // ss << "rleg xi from J" << endl << (Jr_*dqr_).transpose() << endl;
        // ss << "lleg dq " << endl << dql_.transpose() << endl;
        // ss << "lleg dq from J inv " << endl << (Jl_.inverse() * xil_).transpose() << endl;
        // ss << "rleg dq " << endl << dqr_.transpose() << endl;
        // ss << "rleg dq from J inv " << endl << (Jr_.inverse() * xir_).transpose() << endl;


        // motion loop 計算 step 2 〜 6
        // for(int currentFrame = 0; currentFrame < 1; ++currentFrame){
        for(int currentFrame = 0; currentFrame < motion->numFrames(); ++currentFrame){

            ofs << currentFrame * dt;
            eeOfs << currentFrame * dt;
            // ss << "###########################################################" << endl;
            // cout << "time " << currentFrame * dt << endl;

            // prevFrame,nextFrame設定
            int prevFrame = std::max(currentFrame - 1, 0);
            int nextFrame = std::min(currentFrame + 1, motion->numFrames() - 1);

            // // 足先更新
            // Vector3d v_,w_;
            // Matrix3 R_;
            // motion->frame(nextFrame) >> *mBody;
            // mBody->calcForwardKinematics();
            // v_ = mBody->link(BASE_LINK)->p();
            // R_ = mBody->link(BASE_LINK)->R;
            // motion->frame(currentFrame) >> *mBody;
            // mBody->calcForwardKinematics();
            // v_ = ( v_ - mBody->link(BASE_LINK)->p() ) /dt;
            // AngleAxis aa  = AngleAxis( R_ * mBody->link(BASE_LINK)->R.inverse() );
            // w_ = aa.axis() * aa.angle()/dt;


            // 足先速度調査
            (BodyMotion::ConstFrame) motion->frame(nextFrame) >> *mBody;
            mBody->calcForwardKinematics();
            for(auto jointPathPtr : wholeBodyConstraintPtr->constraintJointPathVec) {
                Link* link = jointPathPtr->endLink();
                jointPathPtr->se3.translation() = link->p();
                jointPathPtr->se3.rotation() = link->R();
            }
            (BodyMotion::ConstFrame) motion->frame(currentFrame) >> *mBody;
            mBody->calcForwardKinematics();
            for(auto jointPathPtr : wholeBodyConstraintPtr->constraintJointPathVec) {
                Link* link = jointPathPtr->endLink();
                jointPathPtr->twist.segment(0,3) = ( jointPathPtr->se3.translation() - link->p() ) /dt;
                AngleAxis aa  = AngleAxis( link->R().inverse() * jointPathPtr->se3.rotation().toRotationMatrix() );
                jointPathPtr->twist.segment(3,3) = link->R() * aa.axis()*aa.angle()/dt;
            }

            // q,dq,ddq,rootLink p,R,v,w更新
            updateBodyState(mBody, motion, currentFrame);

            mBody->calcForwardKinematics(true,true);// 状態更新
            mBody->calcCenterOfMass(); // update wc
            calcSubMass(mBody->rootLink(), mSubMasses);

            wholeBodyConstraintPtr->update();


            // ss << "root v after << " << mBody->rootLink()->v().transpose() << endl;
            // ss << "root w" << endl << mBody->rootLink()->w() << endl;
            // ss << "ref xib " << xib.transpose() << endl;

            // ss << "arm v " << mBody->link("LARM_JOINT3")->v().transpose() << endl;
            // ss << "leg v " << mBody->link("LLEG_JOINT5")->v().transpose() << endl;
            // ss << "arm w" << endl << mBody->link("LARM_JOINT3")->w() << endl;
            // ss << "leg w" << endl << mBody->link("LLEG_JOINT5")->w() << endl;

            // step 2
            // cout << " step2";
            MatrixXd A,A_,M,H;
            calcMatrixies(A_, M,H);
            A = S * A_;

            int dof = wholeBodyConstraintPtr->freeJointIdSet().size();

            // cout << " Finished Step 2" << endl;

            // step 3 y計算
            // cout << " step3";
            // 目標足先速度

            VectorXd refM(6);
            refM.block(0,0, 3,1) = refPSeqPtr->at(currentFrame);
            refM.block(3,0, 3,1) = refLSeqPtr->at(currentFrame);
            VectorXd y(numSelects);
            y = refM;
            for(auto jointPathPtr : wholeBodyConstraintPtr->constraintJointPathVec) {
                MatrixXd MH(6,6);
                MH.block(0,0, 3,6) = extractMatrix(M, jointPathPtr->exclusiveJointIdSet()) * inverseJacobian((JointPathPtr&) jointPathPtr);
                MH.block(3,0, 3,6) = extractMatrix(H, jointPathPtr->exclusiveJointIdSet()) * inverseJacobian((JointPathPtr&) jointPathPtr);
                y -= MH * jointPathPtr->twist;
            }
            y = S*y;

            // cout << " Finished Step 3" << endl;

            // step 4 腰座標速度,free関節角速度計算 目標量から制御量の算出
            // cout << " step4";
            VectorXd currentDq(mBody->numJoints());
            for(int i=0; i<mBody->numJoints(); ++i) currentDq(i) = mBody->joint(i)->dq();
            // ss << "ref dqfree " << dqfree.transpose() << endl << endl;

            VectorXd valVec( 6 + dof );
            valVec.head(dof) = wholeBodyConstraintPtr->jointExtendMatrix().block(0,0, mBody->numJoints(),wholeBodyConstraintPtr->freeJointIdSet().size()).transpose() * currentDq;
            valVec.segment(dof,3) = mBody->rootLink()->v();
            valVec.segment(dof+3,3) = mBody->rootLink()->w();

            // ss << "ref valVec " << valVec.transpose() << endl;

            VectorXd dq = VectorXd::Zero(mBody->numJoints()+6);

            MatrixXd Ainv;
            Ainv = PseudoInverse(A);// 正方行列でない場合inverseは使えない
            valVec = Ainv * y + ( MatrixXd::Identity(valVec.rows(),valVec.rows()) - Ainv * A ) * valVec;

            dq += wholeBodyConstraintPtr->jointExtendMatrix() * valVec;

            // cout << endl;
            // cout << "S " << endl << S << endl;
            // cout << "Jl " << endl << Jl << endl;
            // cout << "Jl inv " << endl << Jl.inverse() << endl;
            // cout << "Fl " << endl << Fl << endl;
            // cout << "M " << endl << M.transpose() << endl;
            // cout << "Ml " << endl << Ml << endl;
            // cout << "Mfree " << endl << Mfree << endl;
            // cout << "H " << endl << H << endl;
            // cout << "Hl " << endl << Hl << endl;
            // cout << "Hfree " << endl << Hfree << endl;
            // cout << endl;
            // cout << "A_ " << endl << A_ << endl;
            // cout << "A " << endl << A << endl;
            // cout << "A inv " << endl << Ainv << endl;
            // cout << "y " << y.transpose() << endl;
            // cout << endl;

            // ss << "valVec " << valVec.transpose() << endl;
            // ss << "xib " << xib.transpose() << endl;

            // cout << " Finished Step 4" << endl;

            // step 5 脚関節角速度計算
            // cout << " step5";
            for(auto jointPathPtr : wholeBodyConstraintPtr->constraintJointPathVec) {
                MatrixXd F = extractMatrix(jointPathPtr->jacobianWholeBody(), jointPathPtr->commonJointIdSet());
                dq += jointPathPtr->jointExtendMatrix() * inverseJacobian((JointPathPtr&) jointPathPtr) * (jointPathPtr->twist - F * dq.segment(mBody->numJoints(),6));
            }

            // cout << " Finished Step 5" << endl;

            // step 6 関節角速度計算
            // cout << " step6";
            // ss << "dqfree " << dqfree.transpose() << endl;
            // cout << "dq " << dq.transpose() << endl;

            // RootLink位置更新
            (BodyMotion::ConstFrame) motion->frame(currentFrame) >> *mBody;
            mBody->rootLink()->p() += dq.segment(mBody->numJoints(),3) * dt;
            Vector3d omega = mBody->rootLink()->R().transpose() * dq.segment(mBody->numJoints()+3,3);// w is in world frame, omega is in body frame
            if(omega.norm() != 0) mBody->rootLink()->R() = mBody->rootLink()->R() * AngleAxisd(omega.norm()*dt, omega.normalized());
            else cout << "RootLink orientation is not modified (idx:" << currentFrame << ")"<< endl;

            // // BaseLink更新
            // mBody->link(BASE_LINK)->p += mBody->link(BASE_LINK)->v() * dt;
            // Vector3d omega = mBody->link(BASE_LINK)->w();
            // if(omega.norm() != 0)mBody->link(BASE_LINK)->R() = mBody->link(BASE_LINK)->R() * AngleAxisd(omega.norm()*dt, omega.normalized());

            // q更新
            for(int i = 0; i < mBody->numJoints(); ++i){
                mBody->joint(i)->q() += dq[mBody->joint(i)->jointId()] * dt;
            }
            mBody->calcForwardKinematics();// 状態更新
            // bodyItems[0]->calcForwardKinematics();// 状態更新

            // 足先修正
            // mJpl->calcInverseKinematics(lp,lR);
            // mJpr->calcInverseKinematics(rp,rR);

            motion->frame(nextFrame) << *mBody;// frame更新

            // cout << " Finished Step 6" << endl;


            // 計画後重心軌道をファイル書き出し
            {
                (BodyMotion::ConstFrame) motion->frame(currentFrame) >> *mBody;// 腰・q更新
                mBody->calcForwardKinematics();

                mBody->rootLink()->v() = dq.segment(mBody->numJoints(),3);
                mBody->rootLink()->w() = dq.segment(mBody->numJoints()+3,3);
                Vector3d P,L;
                mBody->calcCenterOfMass();// update wc
                calcSubMass(mBody->rootLink(), mSubMasses);// リンク先重心・慣性行列更新
                calcTotalMomentum(P, L, mBody, mSubMasses[mBody->rootLink()->index()].Iw, dq);

                ofs << " "  << mBody->centerOfMass().transpose();// 重心 2,3,4
                ofs << " " << P.transpose() << " " << L.transpose();// 運動量 5,6,7 角運動量 8,9,10
                ofs << endl;

                for(auto link : endEffectorLinkVec){
                    // eeOfs << " " << link->p().transpose() << " " << link->v().transpose() << " " << link->w().transpose();
                    eeOfs << " " << link->p().transpose();
                    if( link->name().find("LLEG") != std::string::npos ) eeOfs << " " << wholeBodyConstraintPtr->constraintJointPathVec[0]->twist.transpose();
                    else if ( link->name().find("RLEG") != std::string::npos ) eeOfs << " " << wholeBodyConstraintPtr->constraintJointPathVec[1]->twist.transpose();
                    else eeOfs << " " << link->v().transpose() << " " << link->w().transpose();
                }
                eeOfs << endl;
            }

        }// end motion loop

        ofs.close();

    }
}

// 分解運動量制御
void RMControlPlugin::execControl()
{
    MessageView::instance()->putln("RMControl called !");
    cout << "\x1b[31m" << "RMControl()" << "\x1b[m" << endl;

    // yamlファイルから足先リンク取得
    // YAMLReader parser;
    // parser.load("/home/kunio/Dropbox/choreonoid/models/HRP2JSK/hrp2jsk.yaml");
    // MappingPtr info = parser.document()->toMapping();
    // // info->findListing("linkGroup");
    // Listing& linkGroupList = *info->findListing("footLinks");
    // ValueNode& node = linkGroupList[0];
    // std::cout << (node.type() == ValueNode::SCALAR) << " " <<(node.type() == ValueNode::MAPPING) << std::endl;
    // Mapping& group = *node.toMapping();
    // std::cout << group["link"].toString() << std::endl;


    // 最初のBodyItemのみ処理
    ItemList<BodyItem> bodyItems = ItemTreeView::mainInstance()->checkedItems<BodyItem>();// checkedItems チェックされたアイテムを取得
    mBody = bodyItems[0]->body();
    // bodyItems[0]->setCurrentBaseLink( mBody->link(BASE_LINK) );// 左足をbaseに設定
    mSubMasses.resize(mBody->numLinks());
    cout << "numLinks:" <<  mBody->numLinks() << endl;

    // 足先リンク取得
    Listing& footLinkInfos = *mBody->info()->findListing("footLinks");
    if(!footLinkInfos.isValid()){
        cout << " Please load model from yaml file" << endl;
        return;
    }
    mRFootLink = mBody->link( (*footLinkInfos[0].toMapping())["link"].toString() );
    mLFootLink = mBody->link( (*footLinkInfos[1].toMapping())["link"].toString() );
    cout << " Finished setting foot links" << endl;

    // 左右足Pathを設定
    wholeBodyConstraintPtr = std::make_shared<WholeBodyConstraint>(mBody);
    wholeBodyConstraintPtr->constraintJointPathVec.push_back(std::make_shared<ConstraintJointPath>(mBody->rootLink(), mLFootLink));
    wholeBodyConstraintPtr->constraintJointPathVec.push_back(std::make_shared<ConstraintJointPath>(mBody->rootLink(), mRFootLink));


    BodyItemPtr bodyItem;
    PoseSeqItemPtr poseSeqItem;
    PoseSeqPtr poseSeq;
    BodyMotionItemPtr bodyMotionItem;
    BodyMotionPtr motion;
    if(!getSelectedPoseSeqSet(bodyItem, mBody, poseSeqItem, poseSeq, bodyMotionItem, motion)) return;

    mPoseSeqPath = boost::filesystem::path(poseSeqItem->filePath());
    cout << "PoseSeqPath: " << mPoseSeqPath << endl;

    std::vector<Link*> endEffectorLinkVec;
    if(!getEndEffectorLinkVector(endEffectorLinkVec, mBody)) return;

    generateOptionalData(mBody, poseSeqItem, endEffectorLinkVec);

    // BodyMotion作成
    if(motion->numFrames() == 0) generateBodyMotionFromBar(mBody, poseSeqItem, bodyMotionItem);
    else cout << "numFrames: " << motion->numFrames() << "  Need not generate motion" << endl;

    std::set<Link*> contactLinkCandidateSet;
    calcContactLinkCandidateSet(contactLinkCandidateSet, mBody, poseSeqItem->poseSeq());

    // 目標運動量軌道作成
    Vector3d initDCM,endDCM,initL,endL;// initDCMとendPを明示的に0で初期化していない
    string modeStr = mBar->dialog->initialTrajectoryCombo->currentText().toStdString();
    cout << "control mode: " << modeStr << endl;
    string generateMode = mBar->dialog->initialTrajectoryCombo->itemText(Generate).toStdString();
    string loadMode = mBar->dialog->initialTrajectoryCombo->itemText(Load).toStdString();
    if(modeStr == generateMode){
        generateRefPLSeq(bodyMotionItem, poseSeqItem->poseSeq(), initDCM, endDCM, initL, endL);
        cout << " Generated ref P/L" << endl;

        generateRefWrenchSeq(mBody, poseSeqItem, endEffectorLinkVec);
        cout << " Generated ref wrenches" << endl;
    }else{
        loadRefPLSeq(bodyMotionItem, poseSeqItem->poseSeq(), initDCM, endDCM, initL, endL);
        cout << " Loaded ref P/L" << endl;
    }

    // 跳躍期間のrootLinkを目標重心軌道に合わせて修正
    // modifyJumpingTrajectory(poseSeqItem, contactLinkCandidateSet);

    sweepControl(mPoseSeqPath , "", mBody, bodyMotionItem, endEffectorLinkVec);// ParamString is not gotten from ParamSetupLayout and is empty

    cout << "\x1b[31m" << "Finished RMControl" << "\x1b[m" << endl;
}

CNOID_IMPLEMENT_PLUGIN_ENTRY(RMControlPlugin)

