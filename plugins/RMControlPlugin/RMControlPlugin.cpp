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


// ロボットモデル依存の部分あり
// 各種行列を計算
void RMControlPlugin::calcMatrixies(MatrixXd& A_, MatrixXd& Jl, MatrixXd& Jr, MatrixXd& Fl, MatrixXd& Fr,
                                    MatrixXd& M, MatrixXd& H, MatrixXd& Mb, MatrixXd& Mfree, MatrixXd& Hb, MatrixXd& Hfree,
                                    MatrixXd& Ml, MatrixXd& Mr, MatrixXd& Hl, MatrixXd& Hr, vector<Constraint>& jointConstraintVec)
{
    stringstream ss;

    // 重心
    const Vector3d currentCM = mBody->calcCenterOfMass();

    // Ml Mr Hl Hr作成
    // MatrixXd M,H;
    MatrixXd M_( 3, mBody->numJoints()+6 );
    MatrixXd H_( 3, mBody->numJoints()+6 );
    calcCMJacobian(mBody,NULL,M_);
    calcAngularMomentumJacobian(mBody, NULL, H_);// 運動量ヤコビアン、角運動量ヤコビアン
    M = mBody->mass() * M_.block( 0,0, 3, mBody->numJoints() );
    H = H_.block( 0,0, 3, mBody->numJoints() );

    Link* rootLink = mBody->rootLink();
    // Link* lFoot = mBody->link("RLEG_JOINT5"); Link* rFoot = mBody->link("LLEG_JOINT5");
    // JointPathPtr mJpl = getCustomJointPath(mBody, rootLink, lFoot);
    // JointPathPtr mJpr = getCustomJointPath(mBody, rootLink, rFoot);
    mJpl->calcJacobian(Jl); mJpr->calcJacobian(Jr);// 脚・腰間ヤコビアン
    Ml = M.block(0,mBody->link("LLEG_JOINT0")->jointId(), 3,mJpl->numJoints()) * Jl.inverse();
    Mr = M.block(0,mBody->link("RLEG_JOINT0")->jointId(), 3,mJpr->numJoints()) * Jr.inverse();
    Hl = H.block(0,mBody->link("LLEG_JOINT0")->jointId(), 3,mJpl->numJoints()) * Jl.inverse();
    Hr = H.block(0,mBody->link("RLEG_JOINT0")->jointId(), 3,mJpr->numJoints()) * Jr.inverse();

    // Hlleg = H.block(0,mBody->link("LLEG_JOINT0")->jointId, 3,mJpl->numJoints());
    // Hrleg = H.block(0,mBody->link("RLEG_JOINT0")->jointId, 3,mJpr->numJoints());

    // Mfree, Hfree作成
    // freeジョイントがない場合の処理にはおそらく未対応
    // free関節かどうか
    for(int i = 0; i < mBody->numJoints(); ++i) jointConstraintVec.push_back(Free);
    for(int i = 0; i < mJpl->numJoints(); ++i) jointConstraintVec[ mBody->link("LLEG_JOINT0")->jointId() + i ] = LLEG;
    for(int i = 0; i < mJpr->numJoints(); ++i) jointConstraintVec[ mBody->link("RLEG_JOINT0")->jointId() + i ] = RLEG;
    Mfree = MatrixXd(3, mBody->numJoints() - mJpl->numJoints() - mJpr->numJoints() );
    Hfree = MatrixXd(3, mBody->numJoints() - mJpl->numJoints() - mJpr->numJoints() );
    int idx = 0;
    for(int i = 0; i < jointConstraintVec.size(); ++i){
        if(jointConstraintVec[i] == Free){
            Mfree.col(idx) = M.col(i);
            Hfree.col(idx) = H.col(i);
            ++idx;
        }
    }

    // Fl,Fr作成
    Vector3d rl = mLFootLink->p() - rootLink->p();
    Fl = MatrixXd(6,6);
    Fl.block(0,0, 6,6).setIdentity();
    for(int i = 0; i < 3; ++i)Fl.block(0,3+i, 3,1) = Matrix3::Identity().col(i).cross(rl);

    Vector3d rr = mRFootLink->p() - rootLink->p();
    Fr = MatrixXd(6,6);
    Fr.block(0,0, 6,6).setIdentity();
    for(int i = 0; i < 3; ++i)Fr.block(0,3+i, 3,1) = Matrix3::Identity().col(i).cross(rr);

    // Mb,Hb作成
    Mb = MatrixXd(3,6);
    Vector3d rc = currentCM - rootLink->p();
    Mb.block(0,0, 3,3).setIdentity();
    for(int i = 0; i < 3; ++i)Mb.block(0,3+i, 3,1) = Matrix3::Identity().col(i).cross(rc);
    Mb *= mBody->mass();

    Hb = MatrixXd(3,6);
    Hb.block(0,0, 3,3) = MatrixXd::Zero(3,3);
    // Hb.block(0,3, 3,3) = mBody->rootLink()->subIw;
    // vector<SubMassInertia> subMassInertias(mBody->numLinks()); calcSubMassInertia( mBody->rootLink(), subMassInertias_);
    // Matrix3 I_ = subMassInertias_[mBody->rootLink()->index()].Iw;
    Hb.block(0,3, 3,3) = mSubMasses[mBody->rootLink()->index()].Iw;// こっちであってる?
    // Hb.block(0,3, 3,3) = mSubMasses[mBody->rootLink()->index()].Iw + mBody->mass() * D( mBody->centerOfMass() - mBody->rootLink()->p() );

    Mb = Mb - Ml * Fl - Mr * Fr;// 脚は左右とも動かない
    Hb = Hb - Hl * Fl - Hr * Fr;

    // A_を計算
    A_ = MatrixXd(6, Mb.cols()+Mfree.cols() );
    A_.block(0,0, Mb.rows(),Mb.cols()) = Mb;
    A_.block(0,Mb.cols(), Mfree.rows(),Mfree.cols()) = Mfree;
    A_.block(Mb.rows(),0, Hb.rows(),Hb.cols()) = Hb;
    A_.block(Mb.rows(),Mb.cols(), Hfree.rows(),Hfree.cols()) = Hfree;

    // Hlleg = H.block(0,mBody->link("LLEG_JOINT0")->jointId, 3,mJpl->numJoints());
    // Hrleg = H.block(0,mBody->link("RLEG_JOINT0")->jointId, 3,mJpr->numJoints());

    // MessageView::instance()->putln(ss.str());
    return;
}


// スプライン補間
void RMControlPlugin::splineInterpolation(const Vector3d f0, const Vector3d v0, const Vector3d f1, const Vector3d v1, const double tau,
                                          Vector3d& a0, Vector3d& a1, Vector3d& a2, Vector3d& a3)
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


// 目標運動量・角運動量軌道生成
// void generateRefPLSeq(BodyPtr body,BodyItem* bodyItem, const BodyMotionPtr motion,const PoseSeqPtr poseSeq,
void RMControlPlugin::generateRefPLSeq(BodyMotionItem* motionItem ,const PoseSeqPtr poseSeq,
                                       const Vector3d initDCM, const Vector3d endDCM, const Vector3d initL, const Vector3d endL,
                                       Vector3Seq& refPSeq, Vector3Seq& refLSeq)
{

    // BodyItem* bodyItem = motionItem->findOwnerItem<BodyItem>(true);// motionItemからbodyItemを見つける
    const BodyMotionPtr motion = motionItem->motion();

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
        for(int i = 0; i <= 2; ++i){// 3番目まで調べる
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
    motion->frame(takeoffFrame) >> *mBody;
    mBody->calcForwardKinematics();
    const Vector3d takeoffCM = mBody->calcCenterOfMass();
    motion->frame(landingFrame) >> *mBody;
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
    motion->frame(initRMCFrame) >> *mBody;
    mBody->calcForwardKinematics();
    const Vector3d initCM = mBody->calcCenterOfMass();
    motion->frame(endRMCFrame) >> *mBody;
    mBody->calcForwardKinematics();
    const Vector3d endCM = mBody->calcCenterOfMass();

    // Vector3Seq refCMSeq(motion->numFrames(), motion->frameRate());
    // refPSeq = Vector3Seq(motion->numFrames(), motion->frameRate());
    // refLSeq = Vector3Seq(motion->numFrames(), motion->frameRate());
    Vector3Seq refCMSeq(motion->numFrames());
    refPSeq = Vector3Seq(motion->numFrames());
    refLSeq = Vector3Seq(motion->numFrames());

    cout << endl;
    cout << "takeoffCM: " << takeoffCM.transpose() << endl;
    cout << "landingCM: " << landingCM.transpose() << endl;
    cout << "initCM: "  << initCM.transpose() << endl;
    cout << "endCM: " << endCM.transpose() << endl;
    cout << endl;

    // 目標重心位置
    for(int i = 0; i < motion->numFrames(); ++i){
        motion->frame(i) >> *mBody;
        mBody->calcForwardKinematics();
        refCMSeq[i] = mBody->calcCenterOfMass();

        Vector3d v,w;
        VectorXd dq,ddq;
        calcDifferential(motion, i, v, w, dq, ddq);
        mBody->rootLink()->v() = v;
        mBody->rootLink()->w() = w;
        Vector3d P,L;
        calcSubMass(mBody->rootLink(), mSubMasses);// リンク先重心・慣性行列更新
        calcTotalMomentum(P, L, mBody, mSubMasses[mBody->rootLink()->index()].Iw, dq);

        ofs1 << dt*i;
        ofs1 << " " << refCMSeq[i].transpose();//重心 2,3,4
        ofs1 << " " << P.transpose();// 運動量 5,6,7
        ofs1 << " " << L.transpose();// 角運動量 8,9,10
        ofs1 << endl;
        if(i == 1000)cout << "refCM " << refCMSeq[i].transpose() << endl;
    }

    // 目標運動量 motion loop
    for(int i = 0; i < motion->numFrames(); ++i){

        // 角運動量
        refLSeq[i] = Vector3d::Zero();


        // 運動量
        if(takeoffFrame < i && i < landingFrame){// 跳躍期
            refCMSeq[i] = takeoffCM + (landingCM - takeoffCM) * (i-takeoffFrame)*dt /jumptime;
            refCMSeq[i].z() = - 0.5 * g * (i-takeoffFrame)*dt * (i-landingFrame)*dt
                + (landingCM.z()* (i-takeoffFrame)*dt - takeoffCM.z()* (i-landingFrame)*dt) / jumptime;
            refPSeq[i] = m * takeoffDCM;
            refPSeq[i].z() = - m * g * (i - takeoffFrame)*dt + m * takeoffDCM.z();
        }else if(initRMCFrame <= i && i <= endRMCFrame){// 接地期
            int startFrame,endFrame;
            Vector3d f0,v0,f1,v1;
            if(i <= takeoffFrame){
                startFrame = initRMCFrame;
                endFrame = takeoffFrame;
                f0 = initCM; v0 = initDCM;
                f1 = takeoffCM; v1 = takeoffDCM;
            }else if(landingFrame <= i){
                startFrame = landingFrame;
                endFrame = endRMCFrame;
                f0 = landingCM; v0 = landingDCM;
                f1 = endCM; v1 = endDCM;
            }
            double tau = (endFrame - startFrame) * dt;
            Vector3d a0,a1,a2,a3;
            splineInterpolation(f0, v0, f1, v1, tau, a0, a1, a2, a3);// スプライン補間
            refCMSeq[i] = a0 + a1 * (i-startFrame)*dt + a2 * pow ( (i-startFrame)*dt , 2 ) + a3 * pow( (i-startFrame)*dt, 3 );
            refPSeq[i] = ( a1 + 2 * a2 * (i-startFrame)*dt + 3 * a3 * pow ( (i-startFrame)*dt , 2 ) ) * m;

            // 境界条件表示
            if(i == initRMCFrame || i == landingFrame){
                cout << "time " << i*dt << endl;
                cout << "f0 " << f0.transpose() << endl;
                cout << "refCMSeq " << refCMSeq[i].transpose() << endl;
                cout << "v0 " << v0.transpose() << endl;
                cout << "refPSeq " << refPSeq[i].transpose() << endl;
                cout << endl;
            }else if(i == takeoffFrame || i == endRMCFrame){
                cout << "time " << i*dt << endl;
                cout << "f1 " << f1.transpose() << endl;
                cout << "refCMSeq " << refCMSeq[i].transpose() << endl;
                cout << "v1 " << v1.transpose() << endl;
                cout << "refPSeq " << refPSeq[i].transpose() << endl;
                cout << endl;
            }

        }else{// 非制御期
            int nextFrame = std::min(i + 1, motion->numFrames() - 1);
            refPSeq[i] = m * (refCMSeq[nextFrame] - refCMSeq[i]) /dt;
        }

        ofs << i*dt ;
        ofs <<  " " << refCMSeq[i].transpose();// 重心 2,3,4
        ofs <<  " " << refPSeq[i].transpose();// 運動量 5,6,7
        ofs <<  " " << refLSeq[i].transpose();// 角運動量 8,9,10
        ofs << endl;

    }// end motion loop

    // MessageView::instance()->putln(ss.str());
    ofs.close();
}

void RMControlPlugin::loadRefPLSeq(BodyMotionItem* motionItem ,const PoseSeqPtr poseSeq,
                                   const Vector3d initP, const Vector3d endP, const Vector3d initL, const Vector3d endL,
                                   Vector3Seq& refPSeq, Vector3Seq& refLSeq)
{

    // BodyItem* bodyItem = motionItem->findOwnerItem<BodyItem>(true);// motionItemからbodyItemを見つける
    const BodyMotionPtr motion = motionItem->motion();

    stringstream ss,fnamess;
    // fnamess << "/home/kunio/Dropbox/log/choreonoid/refPL_" << motion->frameRate() << ".dat";
    fnamess << mPoseSeqPath.parent_path().string() << "/" << getBasename(mPoseSeqPath) << "_RMC_refPL_" << motion->frameRate() << "fps.dat";
    ofstream ofs(fnamess.str().c_str());
    ofs << "time refCMx refCMy refCMz refPx refPy refPz refLx refLy refLz" << endl;
    const double dt = 1.0/motion->frameRate();
    const double g = 9.80665;
    const double m = mBody->mass();

    // Vector3Seq refCMSeq(motion->numFrames());
    refPSeq = Vector3Seq(motion->numFrames());
    refLSeq = Vector3Seq(motion->numFrames());

    // 目標重心位置
    Vector3SeqPtr refCMSeqPtr = motionItem->findSubItem<Vector3SeqItem>("refCM")->seq();
    Vector3SeqPtr refPSeqPtr = motionItem->findSubItem<Vector3SeqItem>("refP")->seq();
    Vector3SeqPtr refLSeqPtr = motionItem->findSubItem<Vector3SeqItem>("refL")->seq();

    // 目標運動量 motion loop
    for(int i = 0; i < motion->numFrames(); ++i){
        // refLSeq[i] = Vector3d::Zero();
        refLSeq[i] = refLSeqPtr->at(i);

        int nextFrame = std::min( i + 1, motion->numFrames() - 1 );
        // refPSeq[i] = ( refCMSeqPtr->at(nextFrame) - refCMSeqPtr->at(i) ) / dt;
        refPSeq[i] = refPSeqPtr->at(i);

        ofs << i*dt;
        ofs << " " << refCMSeqPtr->at(i).transpose();// 重心 2,3,4
        ofs << " " << refPSeq[i].transpose();// 運動量 5,6,7
        ofs << " " << refLSeq[i].transpose();// 角運動量 8,9,10
        ofs << endl;

    }// end motion loop

    MessageView::instance()->putln(ss.str());
    ofs.close();
}

// 分解運動量制御
void RMControlPlugin::execControl()
{
    stringstream ss,fnamess;
    MessageView::instance()->putln("RMControl called !");
    cout << "RMControl()" << endl;

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


    // 選択行列作成 step 1
    VectorXd SVec(6); SVec << 0.1,0.1,0.1, 1000,1000,0;
    const int numSelects = (SVec.array() != 0).count();
    MatrixXd S(numSelects,6);
    for(int i = 0; i < numSelects; ++i){
        if(SVec(i) != 0) S.row(i) = SVec(i)*MatrixXd::Identity(6,6).row(i);
    }
    cout << " Finished Step 1: setting select array" << endl;

    // 最初のBodyItemのみ処理
    ItemList<BodyItem> bodyItems = ItemTreeView::mainInstance()->checkedItems<BodyItem>();// checkedItems チェックされたアイテムを取得
    mBody = bodyItems[0]->body();
    bodyItems[0]->setCurrentBaseLink( mBody->link(BASE_LINK) );// 左足をbaseに設定
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
    mJpl = getCustomJointPath( mBody, mBody->rootLink(), mLFootLink );
    mJpr = getCustomJointPath( mBody, mBody->rootLink(), mRFootLink );

    // PoseSeqItem loop
    // for(Item* pChildItem = bodyItems[0]->childItem(); pChildItem; pChildItem = pChildItem->nextItem()){
    // PoseSeqItem* pPoseSeqItem = dynamic_cast<PoseSeqItem*>(pChildItem);
    PoseSeqItemPtr pPoseSeqItem = ItemTreeView::mainInstance()->selectedItems<PoseSeqItem>()[0];
    mPoseSeqPath = boost::filesystem::path(pPoseSeqItem->filePath());
    if(pPoseSeqItem){
        cout << "  PoseSeqName: " << pPoseSeqItem->name() << endl;

        BodyMotionItem* bodyMotionItem = pPoseSeqItem->bodyMotionItem();
        BodyMotionPtr motion = bodyMotionItem->motion();

        // BodyMotion作成
        BodyMotionGenerationBar* bmgb = BodyMotionGenerationBar::instance();// インスタンス生成
        PoseProvider* provider = pPoseSeqItem->interpolator().get();
        bmgb->shapeBodyMotion(mBody, provider, bodyMotionItem, true);
        cout << " Generated motion" << endl;

        // 目標運動量軌道作成
        Vector3Seq refPSeq,refLSeq;
        Vector3d initDCM,endDCM,initL,endL;// initDCMとendPを明示的に0で初期化していない
        string modeStr = mBar->dialog->initialTrajectoryCombo->currentText().toStdString();
        cout << "control mode: " << modeStr << endl;
        string generateMode = mBar->dialog->initialTrajectoryCombo->itemText(Generate).toStdString();
        string loadMode = mBar->dialog->initialTrajectoryCombo->itemText(Load).toStdString();
        if(modeStr == generateMode){
            generateRefPLSeq( bodyMotionItem, pPoseSeqItem->poseSeq(), initDCM, endDCM, initL, endL, refPSeq, refLSeq );
            cout << " Generated ref P/L" << endl;
        }else{
            loadRefPLSeq( bodyMotionItem, pPoseSeqItem->poseSeq(), initDCM, endDCM, initL, endL, refPSeq, refLSeq );
            cout << " Loaded ref P/L" << endl;
        }

        // Item* pChildItem = bodyItems[0]->childItem();
        for(int i = 0; i < 3; ++i){
            fnamess.str("");
            fnamess << mPoseSeqPath.parent_path().string() << "/" << getBasename(mPoseSeqPath) << "_RMC_PL_" << motion->frameRate() << "fps_" << i << ".dat";
            ofstream ofs( fnamess.str().c_str() );
            if(!ofs){ cerr << "File Open Error" << endl; return;}
            ofs << "time CMx CMy CMz Px Py Pz Lx Ly Lz" << endl;

            double dt = 1.0/motion->frameRate();

            // 運動量ヤコビアンが正しいかどうかの確認
            motion->frame(0) >> *mBody;// 腰座標、q更新 body更新

            VectorXd dq_(mBody->numJoints());
            MultiValueSeq::Frame q = motion->jointPosSeq()->frame(0);
            for(int k=0; k < motion->numJoints(); ++k){
                Link* joint = mBody->joint(k);
                joint->q() = q[k];
                joint->dq() = -0.3;
                joint->ddq() = 0;
                dq_[k] = joint->dq();
            }
            bodyItems[0]->calcForwardKinematics(true,true);// v,wを更新

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
                Vector3d lv,lw,rv,rw, lp, rp;
                Matrix3d lR,rR;
                motion->frame(nextFrame) >> *mBody;
                mBody->calcForwardKinematics();
                lp = mLFootLink->p(); rp = mRFootLink->p();
                lR = mLFootLink->R(); rR = mRFootLink->R();
                motion->frame(currentFrame) >> *mBody;
                mBody->calcForwardKinematics();
                lv = ( lp - mLFootLink->p() ) /dt;
                rv = ( rp - mRFootLink->p() ) /dt;
                AngleAxis laa  = AngleAxis( lR * mLFootLink->R().inverse() );
                AngleAxis raa  = AngleAxis( rR * mRFootLink->R().inverse() );
                lw = laa.axis() * laa.angle()/dt;
                rw = raa.axis() * raa.angle()/dt;

                // q,dq,ddq,rootLink p,R,v,w更新
                updateBodyState(mBody, motion, currentFrame);

                mBody->calcForwardKinematics(true,true);// 状態更新
                calcSubMass(mBody->rootLink(), mSubMasses);

                VectorXd xib(6);// xib
                xib.block(0,0, 3,1) = mBody->rootLink()->v();
                xib.block(3,0, 3,1) = mBody->rootLink()->w();

                // ss << "root v after << " << mBody->rootLink()->v().transpose() << endl;
                // ss << "root w" << endl << mBody->rootLink()->w() << endl;
                // ss << "ref xib " << xib.transpose() << endl;

                // ss << "arm v " << mBody->link("LARM_JOINT3")->v().transpose() << endl;
                // ss << "leg v " << mBody->link("LLEG_JOINT5")->v().transpose() << endl;
                // ss << "arm w" << endl << mBody->link("LARM_JOINT3")->w() << endl;
                // ss << "leg w" << endl << mBody->link("LLEG_JOINT5")->w() << endl;

                // step 2
                // cout << " step2";
                MatrixXd A,A_,Jl,Jr,Fl,Fr,M,H,Mb,Mfree,Hb,Hfree,Ml,Mr,Hl,Hr;
                vector<Constraint> jointConstraintVec;
                calcMatrixies( A_, Jl, Jr, Fl, Fr,M,H,Mb,Mfree,Hb,Hfree, Ml, Mr, Hl, Hr, jointConstraintVec);
                A = S * A_;

                int dof = 0;// free関節の数
                for(int i = 0; i < jointConstraintVec.size(); ++i)if(jointConstraintVec[i] == Free)++dof;
            
                // cout << " Finished Step 2" << endl;

                // step 3 y計算
                // cout << " step3";
                // 目標足先速度
                VectorXd xil(6);// xil
                xil.block(0,0, 3,1) = lv;
                xil.block(3,0, 3,1) = lw;
                VectorXd xir(6);// xir
                xir.block(0,0, 3,1) = rv;
                xir.block(3,0, 3,1) = rw;

                VectorXd refM(6);
                refM.block(0,0, 3,1) = refPSeq[currentFrame];
                refM.block(3,0, 3,1) = refLSeq[currentFrame];
                MatrixXd MHl(6,6);
                MHl.block(0,0, 3,6) = Ml; MHl.block(3,0, 3,6) = Hl;
                MatrixXd MHr(6,6);
                MHr.block(0,0, 3,6) = Mr; MHr.block(3,0, 3,6) = Hr;
                VectorXd y(numSelects);
                y = S * (refM  - MHl * xil - MHr * xir);

                // cout << " Finished Step 3" << endl;

                // step 4 腰座標速度,free関節角速度計算 目標量から制御量の算出
                // cout << " step4";
                VectorXd dqfree(dof); // dqfree
                int idx = 0;
                for(int i = 0; i < mBody->numJoints(); ++i){
                    if(jointConstraintVec[i] == Free){
                        dqfree[idx] = mBody->joint(i)->dq();// 目標free関節速度
                        ++idx;
                    }
                }
                // ss << "ref dqfree " << dqfree.transpose() << endl << endl;

                VectorXd valVec( 6 + dof );
                valVec.block(0,0, 6,1) = xib;
                valVec.block(6,0, dof,1) = dqfree;

                // ss << "ref valVec " << valVec.transpose() << endl;

                MatrixXd Ainv;
                Ainv = PseudoInverse(A);// 正方行列でない場合inverseは使えない
                valVec = Ainv * y + ( MatrixXd::Identity(valVec.rows(),valVec.rows()) - Ainv * A ) * valVec;
                xib = valVec.block(0,0, 6,1);
                dqfree = valVec.block(6,0, dof,1);

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
                VectorXd dql((mBody->numJoints() - dof) / 2);
                VectorXd dqr((mBody->numJoints() - dof) / 2);
                dql = Jl.inverse() * (xil - Fl * xib);
                dqr = Jr.inverse() * (xir - Fr * xib);

                // cout << " Finished Step 5" << endl;

                // step 6 関節角速度計算
                // cout << " step6";
                VectorXd dq(mBody->numJoints());
                int freeIdx = 0,llegIdx = 0,rlegIdx = 0;
                for(int i = 0; i < mBody->numJoints(); ++i){
                    switch (jointConstraintVec[i]){
                    case Free:
                        dq[i] = dqfree[freeIdx]; ++freeIdx;
                        break;
                    case LLEG:
                        dq[i] = dql[llegIdx]; ++llegIdx;
                        break;
                    case RLEG:
                        dq[i] = dqr[rlegIdx]; ++rlegIdx;
                        break;
                    default:
                        MessageView::instance()->putln("register the type of constraint");
                        break;
                    }
                }
                // ss << "dqfree " << dqfree.transpose() << endl;
                // cout << "dq " << dq.transpose() << endl;

                // RootLink位置更新
                motion->frame(currentFrame) >> *mBody;
                mBody->rootLink()->p() += xib.block(0,0, 3,1) * dt;
                Vector3d omega = xib.block(3,0, 3,1);
                if(omega.norm() != 0) mBody->rootLink()->R() = mBody->rootLink()->R() * AngleAxisd(omega.norm()*dt, omega.normalized());
                else cout << "RootLink orientation is not modified (idx:" << currentFrame << ")"<< endl;


                // // BaseLink更新
                // mBody->link(BASE_LINK)->p += mBody->link(BASE_LINK)->v() * dt;
                // Vector3d omega = mBody->link(BASE_LINK)->w();
                // if(omega.norm() != 0)mBody->link(BASE_LINK)->R() = mBody->link(BASE_LINK)->R() * AngleAxisd(omega.norm()*dt, omega.normalized());

                // q更新
                for(int i = 0; i < mBody->numJoints(); ++i){
                    mBody->joint(i)->q() += dq[i] * dt;
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
                    motion->frame(currentFrame) >> *mBody;// 腰・q更新
                    mBody->calcForwardKinematics();

                    mBody->rootLink()->v() = xib.block(0,0, 3,1);
                    mBody->rootLink()->w() = xib.block(3,0, 3,1);
                    Vector3d P,L;
                    calcSubMass(mBody->rootLink(), mSubMasses);// リンク先重心・慣性行列更新
                    calcTotalMomentum(P, L, mBody, mSubMasses[mBody->rootLink()->index()].Iw, dq);

                    ofs << " "  << mBody->calcCenterOfMass().transpose();// 重心 2,3,4
                    ofs << " " << P.transpose() << " " << L.transpose();// 運動量 5,6,7 角運動量 8,9,10
                    ofs << endl;
                }
            
            }// end motion loop

            ofs.close();

        }
    }// end poseSeqItem loop



    cout << "Finished RMControl" << endl;
}

CNOID_IMPLEMENT_PLUGIN_ENTRY(RMControlPlugin)

