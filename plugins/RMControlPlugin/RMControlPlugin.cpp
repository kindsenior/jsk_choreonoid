/**
   @author Kunio Kojima
*/
#include "RMControlPlugin.h"

#define BASE_LINK "LLEG_JOINT5"

using namespace boost;
using namespace cnoid;
using namespace std;

MatrixXd pinv(const MatrixXd A){
  if(A.rows() < A.cols()){
    return A.transpose() * ( A * A.transpose() ).inverse();
  }else{
    return ( A.transpose() * A ).inverse() * A.transpose();
  }
}

namespace {

Matrix3d D(Vector3d r)
{
    Matrix3d r_cross;
    r_cross <<
        0.0,  -r(2), r(1),
        r(2),    0.0,  -r(0),
        -r(1), r(0),    0.0;
    return r_cross.transpose() * r_cross;
}

}

void RMControlPlugin::calcSubMass(Link* link, vector<SubMass>& subMasses)
{
    Matrix3d R = link->R();
    SubMass& sub = subMasses[link->index()];
    sub.m = link->m();
    sub.mwc = link->m() * link->wc();

    for(Link* child = link->child(); child; child = child->sibling()){
        calcSubMass(child, subMasses);
        SubMass& childSub = subMasses[child->index()];
        sub.m += childSub.m;
        sub.mwc += childSub.mwc;
    }

    sub.Iw = R * link->I() * R.transpose() + link->m() * D( link->wc() - sub.mwc/sub.m );
    for(Link* child = link->child(); child; child = child->sibling()){
        SubMass& childSub = subMasses[child->index()];
        sub.Iw += childSub.Iw + childSub.m * D( childSub.mwc/childSub.m - sub.mwc/sub.m );
    }
}

// ロボットモデル依存の部分あり
// 各種行列を計算
void RMControlPlugin::calcMatrixies( MatrixXd& A_, MatrixXd& Jl, MatrixXd& Jr, MatrixXd& Fl, MatrixXd& Fr,
                                     MatrixXd& M, MatrixXd& H, MatrixXd& Mb, MatrixXd& Mfree, MatrixXd& Hb, MatrixXd& Hfree,
                                     MatrixXd& Ml, MatrixXd& Mr, MatrixXd& Hl, MatrixXd& Hr, vector<Constraint>& jointConstraintVec){
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
        if( jointConstraintVec[i] == Free ){
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
    // Hb.block(0,3, 3,3) = mSubMasses[mBody->rootLink()->index()].Iw;// とりあえず
    Hb.block(0,3, 3,3) = mSubMasses[mBody->rootLink()->index()].Iw + mBody->mass() * D( mBody->centerOfMass() - mBody->rootLink()->p() );

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
                                          Vector3d& a0, Vector3d& a1, Vector3d& a2, Vector3d& a3){
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
void RMControlPlugin::generateRefPLSeq( BodyMotionItem* motionItem ,const PoseSeqPtr poseSeq,
                                        const Vector3d initP, const Vector3d endP, const Vector3d initL, const Vector3d endL, 
                                        Vector3Seq& refPSeq, Vector3Seq& refLSeq){

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

    // 離着陸Pose設定
    PoseSeq::iterator takeoffPoseItr = poseSeq->begin();
    PoseSeq::iterator landingPoseItr = poseSeq->begin();
    PoseSeq::iterator initRMCPoseItr = poseSeq->begin();
    PoseSeq::iterator endRMCPoseItr = poseSeq->begin();
    bool prevTouchFlg = true;
    for( PoseSeq::iterator poseItr = poseSeq->begin(); poseItr != poseSeq->end(); ++poseItr){
        bool TouchFlg = false;

        // Poseの接地判定
        Pose::LinkInfoMap::iterator itr = poseItr->get<Pose>()->ikLinkBegin();
        for(int i = 0; i <= 2; ++i){// 3番目まで調べる
            TouchFlg |= itr->second.isTouching() && !itr->second.isBaseLink();
            ++itr;
        }

        // Poseの離着陸判定
        if( prevTouchFlg && !TouchFlg ){
            takeoffPoseItr = --poseItr;
            initRMCPoseItr = --poseItr;
            ++poseItr;++poseItr;
        }
        if( !prevTouchFlg && TouchFlg ){
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
    Vector3d takeoffP = m * (landingCM - takeoffCM) / jumptime;
    Vector3d landingP = takeoffP;
    takeoffP.z() = (0.5 * g * jumptime + (landingCM.z() - takeoffCM.z()) / jumptime) * m;
    landingP.z() = takeoffP.z() - m * g * jumptime;

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


    // 目標重心位置
    for(int i = 0; i < motion->numFrames(); ++i){
        motion->frame(i) >> *mBody;
        mBody->calcForwardKinematics();
        refCMSeq[i] = m * mBody->calcCenterOfMass();
        // ss << "refCM " << refCMSeq[i].transpose() << endl;
    }

    // 目標運動量 motion loop
    for(int i = 0; i < motion->numFrames(); ++i){

        // 角運動量
        refLSeq[i] = Vector3d::Zero();


        // 運動量
        if( takeoffFrame < i && i < landingFrame){// 跳躍期
            refCMSeq[i] = (takeoffCM + (landingCM - takeoffCM) * (i-takeoffFrame)*dt /jumptime) * m;
            refCMSeq[i].z() = - 0.5 * m*g * (i-takeoffFrame)*dt * (i-landingFrame)*dt
                + (landingCM.z()* (i-takeoffFrame)*dt - takeoffCM.z()* (i-landingFrame)*dt) * m / jumptime;
            refPSeq[i] = takeoffP;
            refPSeq[i].z() = - m * g * (i - takeoffFrame)*dt + takeoffP.z();
        }else if ( initRMCFrame <= i && i <= endRMCFrame ){// 接地期
            int startFrame,endFrame;
            Vector3d f0,v0,f1,v1;
            if( i <= takeoffFrame ){
                startFrame = initRMCFrame;
                endFrame = takeoffFrame;
                f0 = m * initCM; v0 = initP;
                f1 = m * takeoffCM; v1 = takeoffP;
            }else if( landingFrame <= i ){
                startFrame = landingFrame;
                endFrame = endRMCFrame;
                f0 = m * landingCM; v0 = landingP;
                f1 = m * endCM; v1 = endP;
            }
            double tau = (endFrame - startFrame) * dt;
            Vector3d a0,a1,a2,a3;
            splineInterpolation(f0, v0, f1, v1, tau, a0, a1, a2, a3);// スプライン補間
            refCMSeq[i] = a0 + a1 * (i-startFrame)*dt + a2 * pow ( (i-startFrame)*dt , 2 ) + a3 * pow( (i-startFrame)*dt, 3 );
            refPSeq[i] = a1 + 2 * a2 * (i-startFrame)*dt + 3 * a3 * pow ( (i-startFrame)*dt , 2 );

            // 境界条件表示
            if( i == initRMCFrame || i == landingFrame){
                cout << "time " << i*dt << endl;
                cout << "f0 " << f0.transpose() << endl;
                cout << "refCMSeq " << refCMSeq[i].transpose() << endl;
                cout << "v0 " << v0.transpose() << endl;
                cout << "refPSeq " << refPSeq[i].transpose() << endl;
                cout << endl;
            }else if( i == takeoffFrame || i == endRMCFrame ){
                cout << "time " << i*dt << endl;
                cout << "f1 " << f1.transpose() << endl;
                cout << "refCMSeq " << refCMSeq[i].transpose() << endl;
                cout << "v1 " << v1.transpose() << endl;
                cout << "refPSeq " << refPSeq[i].transpose() << endl;
                cout << endl;
            }

        }else{// 非制御期
            int nextFrame = std::min(i + 1, motion->numFrames() - 1);
            refPSeq[i] = (refCMSeq[nextFrame] - refCMSeq[i]) /dt;
        }

        ofs << i*dt ;
        ofs <<  " " << (refCMSeq[i]/m).transpose();// 重心 2,3,4
        ofs <<  " " << refPSeq[i].transpose();// 運動量 5,6,7
        ofs <<  " " << refLSeq[i].transpose();// 角運動量 8,9,10
        ofs << endl;

    }// end motion loop

    // MessageView::instance()->putln(ss.str());
    ofs.close();
}

void RMControlPlugin::loadRefPLSeq( BodyMotionItem* motionItem ,const PoseSeqPtr poseSeq,
                                        const Vector3d initP, const Vector3d endP, const Vector3d initL, const Vector3d endL, 
                                        Vector3Seq& refPSeq, Vector3Seq& refLSeq){

    // BodyItem* bodyItem = motionItem->findOwnerItem<BodyItem>(true);// motionItemからbodyItemを見つける
    const BodyMotionPtr motion = motionItem->motion();

    stringstream ss,fnamess;
    // fnamess << "/home/kunio/Dropbox/log/choreonoid/refPL_" << motion->frameRate() << ".dat";
    fnamess << mPoseSeqPath.parent_path().string() << "/" << getBasename(mPoseSeqPath) << "_RMC_refPL_" << motion->frameRate() << "fps.dat";
    ofstream ofs(fnamess.str().c_str());
    ofs << "time refCMx refCMy refCMz refPx refPy refPz" << endl;
    const double dt = 1.0/motion->frameRate();
    const double g = 9.80665;
    const double m = mBody->mass();

    // Vector3Seq refCMSeq(motion->numFrames());
    refPSeq = Vector3Seq(motion->numFrames());
    refLSeq = Vector3Seq(motion->numFrames());

    // 目標重心位置
    Vector3SeqPtr refCMSeq = motionItem->findSubItem<Vector3SeqItem>("refCM")->seq();

    // 目標運動量 motion loop
    for(int i = 0; i < motion->numFrames(); ++i){
        refLSeq[i] = Vector3d::Zero();

        int nextFrame = std::min( i + 1, motion->numFrames() - 1 );
        refPSeq[i] = ( refCMSeq->at(nextFrame) - refCMSeq->at(i) ) / dt;

        ofs << i*dt ;
        ofs <<  " " << ( refCMSeq->at(i)/m ).transpose();// 重心 2,3,4
        ofs <<  " " << refPSeq[i].transpose();// 運動量 5,6,7
        ofs << endl;

    }// end motion loop

    MessageView::instance()->putln(ss.str());
    ofs.close();
}

// 分解運動量制御
void RMControlPlugin::RMControl(){
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
    int select[] = {1,2,3,4,5};
    int numSelects = sizeof(select)/sizeof(select[0]);
    MatrixXd S(numSelects,6);
    for(int i = 0; i < numSelects; ++i){
        S.row(i) = MatrixXd::Identity(6,6).row(select[i]-1);
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
    if( !footLinkInfos.isValid() ){
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
    // Item* pChildItem = bodyItems[0]->childItem();
    for(int i = 0; i < 1; ++i){
        // for(Item* pChildItem = bodyItems[0]->childItem(); pChildItem; pChildItem = pChildItem->nextItem()){
        // PoseSeqItem* pPoseSeqItem = dynamic_cast<PoseSeqItem*>(pChildItem);
        PoseSeqItemPtr pPoseSeqItem = ItemTreeView::mainInstance()->selectedItems<PoseSeqItem>()[0];
        mPoseSeqPath = boost::filesystem::path(pPoseSeqItem->filePath());

        if( pPoseSeqItem ){
            cout << "  PoseSeqName: " << pPoseSeqItem->name() << endl;

            BodyMotionItem* bodyMotionItem = pPoseSeqItem->bodyMotionItem();
            BodyMotionPtr motion = bodyMotionItem->motion();

            // BodyMotion作成
            BodyMotionGenerationBar* bmgb = BodyMotionGenerationBar::instance();// インスタンス生成
            PoseProvider* provider = pPoseSeqItem->interpolator().get();
            bmgb->shapeBodyMotion(mBody, provider, bodyMotionItem, true);
            cout << " Generated motion" << endl;


            fnamess.str("");
            // fnamess << "/home/kunio/Copy/Documents/log/choreonoid/RMControl_"  << motion->frameRate() << ".dat";
            fnamess << mPoseSeqPath.parent_path().string() << "/" << getBasename(mPoseSeqPath) << "_RMC_PL_" << motion->frameRate() << "fps.dat";
            ofstream ofs( fnamess.str().c_str() );
            if( !ofs ){ cerr << "File Open Error" << endl; return;}
            ofs << "time CMx CMy CMz Px Py Pz Lx Ly Lz" << endl;

            double dt = 1.0/motion->frameRate();

            // 目標運動量軌道作成
            Vector3Seq refPSeq,refLSeq;
            Vector3d initP,endP,initL,endL;
            // generateRefPLSeq( bodyMotionItem, pPoseSeqItem->poseSeq(), initP, endP, initL, endL, refPSeq, refLSeq );
            loadRefPLSeq( bodyMotionItem, pPoseSeqItem->poseSeq(), initP, endP, initL, endL, refPSeq, refLSeq );
            cout << " Generated ref P/L" << endl;


            // 運動量ヤコビアンが正しいかどうかの確認
            motion->frame(0) >> *mBody;// 腰座標、q更新 body更新

            VectorXd dq_(mBody->numJoints());
            MultiValueSeq::Frame q = motion->jointPosSeq()->frame(0);
            for(int k=0; k < motion->numJoints(); ++k){
                Link* joint = mBody->joint(k);
                joint->q() = q[k];
                joint->dq() = 0.1;
                // joint->dq() = 0;
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
            MatrixXd M__,H__,M__tmp,H__tmp;
            calcCMJacobian(mBody, NULL, M__tmp);// 運動量ヤコビアン、角運動量ヤコビアン(重心基準)
            calcAngularMomentumJacobian(mBody, NULL, H__tmp);
            M__ = mBody->mass() * M__tmp.block( 0,0, 3, mBody->numJoints() );
            H__ = H__tmp.block( 0,0, 3, mBody->numJoints() );
            Vector3 L__,P__,L_tmp;
            Vector3d r__ = mBody->centerOfMass() - mBody->rootLink()->p();
            P__ = mBody->mass() * ( mBody->rootLink()->v() - r__.cross( mBody->rootLink()->w() ) ) + M__ * dq_;
            calcSubMass( mBody->rootLink(), mSubMasses);
            Matrix3 I__ = mSubMasses[mBody->rootLink()->index()].Iw;
            // Matrix3 I__ = body->rootLink()->subIw;
            L_tmp = I__ * mBody->rootLink()->w() + H__ * dq_;
            L__ = mBody->centerOfMass().cross(P__) + L_tmp; // 原点回りに変換

            cout << "correct P " << P_world.transpose() << endl;
            cout << "my P " << P__.transpose() << endl;
            cout << "correct L " << L_world.transpose() << endl;
            cout << "my L " << L__.transpose() << endl;
            // ss << "root w " << body->rootLink()->w().transpose() << endl;



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
                // for(int currentFrame = 0; currentFrame < 3; ++currentFrame){

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


                // 腰座標、q更新  (body更新)
                motion->frame(currentFrame) >> *mBody;

                // dq,ddq更新
                MultiValueSeq::Frame q0 = motion->jointPosSeq()->frame(prevFrame);
                MultiValueSeq::Frame q1 = motion->jointPosSeq()->frame(currentFrame);
                MultiValueSeq::Frame q2 = motion->jointPosSeq()->frame(nextFrame);
                for(int k=0; k < motion->numJoints(); ++k){
                    Link* joint = mBody->joint(k);
                    joint->q() = q1[k];
                    joint->dq() = (q2[k] - q1[k]) / dt;
                    joint->ddq() = (q2[k] - 2.0 * q1[k] + q0[k]) / (dt * dt);
                }

                // rootLink v,x更新
                SE3 nextWaistSE3 = motion->linkPosSeq()->frame(nextFrame)[0];
                Vector3d v_ = nextWaistSE3.translation();
                Matrix3d R_ = nextWaistSE3.rotation().toRotationMatrix();
                v_ -= mBody->rootLink()->p(); v_ /= dt;
                R_ *= mBody->rootLink()->R().inverse();
                AngleAxis aa = AngleAxis(R_);
                Vector3d w_ = aa.axis() * aa.angle()/dt;

                mBody->rootLink()->v() = v_;
                mBody->rootLink()->w() = w_;
                // mBody->link(BASE_LINK)->v() = v_;
                // mBody->link(BASE_LINK)->w() = w_;

                mBody->calcForwardKinematics(true,true);// 状態更新
                calcSubMass(mBody->rootLink(), mSubMasses);
                // cout << "mSubMasses" << endl;
                // for(int i=0; i < mBody->numJoints(); ++i){
                //     Link* joint = mBody->joint(i);
                //     cout << mSubMasses[joint->jointId()].Iw << endl;
                // }

                // bodyItems[0]->calcForwardKinematics(true,true);
                // ss << "root v from motion " << v_.transpose() << endl;
                // ss << "root w from motion " << w_.transpose() << endl;


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

                // Matrix3d I_ = mBody->rootLink()->subIw;
                // vector<SubMassInertia> subMassInertias(mBody->numLinks());
                // calcSubMassInertia(mBody->rootLink(), mSubMassInertias);
                Matrix3d I_ = mSubMasses[mBody->rootLink()->index()].Iw;


                int dof = 0;// free関節の数
                for(int i = 0; i < jointConstraintVec.size(); ++i)if(jointConstraintVec[i] == Free)++dof;
            
                // cout << " Finished Step 2" << endl;

                // step 3 y計算
                // cout << " step3";
                // 目標足先速度
                VectorXd xil(6);// xil
                // xil.block(0,0, 3,1) = mBody->link("LLEG_JOINT5")->v();
                // xil.block(3,0, 3,1) = mBody->link("LLEG_JOINT5")->w();
                xil.block(0,0, 3,1) = lv;
                xil.block(3,0, 3,1) = lw;
                VectorXd xir(6);// xir
                // xir.block(0,0, 3,1) = mBody->link("RLEG_JOINT5")->v();
                // xir.block(3,0, 3,1) = mBody->link("RLEG_JOINT5")->w();
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
                // cout << "refM " << refM.transpose() << endl;

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

                MatrixXd Ainv = pinv(A);// 正方行列でない場合inverseは使えない
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
                mBody->rootLink()->p() += xib.block(0,0, 3,1) * dt;
                Vector3d omega = xib.block(3,0, 3,1);
                if( omega.norm() != 0 ) mBody->rootLink()->R() = mBody->rootLink()->R() * AngleAxisd(omega.norm()*dt, omega.normalized());


                // // BaseLink更新
                // mBody->link(BASE_LINK)->p += mBody->link(BASE_LINK)->v() * dt;
                // Vector3d omega = mBody->link(BASE_LINK)->w();
                // if( omega.norm() != 0 )mBody->link(BASE_LINK)->R() = mBody->link(BASE_LINK)->R() * AngleAxisd(omega.norm()*dt, omega.normalized());

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

                // ss << endl;
                // ss << "leg vel " << mBody->link("LLEG_JOINT5")->v().transpose() << endl;
                // ss << "leg w " << mBody->link("LLEG_JOINT5")->w().transpose() << endl;
                // ss << "rleg vel " << mBody->link("RLEG_JOINT5")->v().transpose() << endl;
                // ss << "rleg w " << mBody->link("RLEG_JOINT5")->w().transpose() << endl;


                // MultiValueSeq::Frame q = motion->jointPosSeq()->frame(nextFrame);
                // ss << "q " ;
                // for(int i = 0; i < mBody->numJoints(); ++i)ss << q[i] << " " ;
                // ss << endl;


                // // matrixファイル書き出し
                // static bool writeFlg = true;
                // bool nanFlg = false;
                // double nowt = currentFrame * dt;
                // for(int i = 0; i < xib.size(); ++i)if(isnan(abs(xib[i])))nanFlg = true;
                // for(int i = 0; i < dq.size(); ++i)if(isnan(abs(dq[i])))nanFlg = true;
                // // if(nanFlg && writeFlg){
                // // if(true){
                // if (nowt == 1.1 || nowt == 1.5 || nowt == 2){
                //   writeFlg = false;
                //   MatrixXd Hlleg = H.block(0,mLFootLink->jointId(), 3,mJpl->numJoints());
                //   MatrixXd Hrleg = H.block(0,mRFootLink->jointId(), 3,mJpr->numJoints());
                //   MatrixXd matrixList[] = {A,A_,Jl,Jr,Fl,Fr,M,H,Mb,Mfree,Hb,Hfree,Ml,Mr,Hl,Hr,MHl,MHr,Hlleg,Hrleg,I_,S,xib,xil,xir,dqfree};
                //   string nameList[] = {"A", "A_", "Jl", "Jr", "Fl", "Fr", "M", "H", "Mb", "Mfree", "Hb", "Hfree",
                //                        "Ml", "Mr", "Hl", "Hr", "MHl", "MHr", "Hlleg", "Hrleg", "I", "S", "xib", "xil", "xir", "dqfree"};
                //   stringstream matrixss;
                //   matrixss << "/home/kunio/matricies_" << nowt;
                //   ofstream matfs(matrixss.str().c_str());
                //   for(int i = 0; i < 26; ++i){
                //     matfs << "# name: " << nameList[i] <<endl;
                //     matfs << "# type: matrix" << endl;
                //     matfs << "# rows: " << matrixList[i].rows() << endl;
                //     matfs << "# columns: " << matrixList[i].cols() << endl;
                //     matfs << matrixList[i] << endl;
                //   }
                //   matfs.close();
                // }
                // cout << " Finished writing matrix files" << endl;


                // 計画後重心軌道をファイル書き出し
                motion->frame(currentFrame) >> *mBody;// 腰・q更新
                mBody->calcForwardKinematics();
                v_ = xib.block(0,0, 3,1);
                w_ = xib.block(3,0, 3,1);
                MatrixXd M_,H_,M_tmp,H_tmp;
                calcCMJacobian(mBody, NULL, M_tmp);
                calcAngularMomentumJacobian(mBody, NULL, H_tmp);// 運動量ヤコビアン、角運動量ヤコビアン(重心基準)
                M_ = mBody->mass() * M_tmp.block( 0,0, 3, mBody->numJoints() );
                H_ = H_tmp.block( 0,0, 3, mBody->numJoints() );
                calcSubMass(mBody->rootLink(), mSubMasses);// リンク先重心・慣性行列更新
                Vector3d L_,P_;
                Vector3d r_ = mBody->calcCenterOfMass() - mBody->rootLink()->p();
                // cout << "M:" << M << endl;
                // cout << "H:" << H << endl;
                // cout << "Iw:" << mSubMasses[mBody->rootLink()->index()].Iw << endl;
                // cout << "mwc:" << mSubMasses[mBody->rootLink()->index()].mwc << endl;
                // cout << "m:" << mSubMasses[mBody->rootLink()->index()].m << endl;
                P_ = mBody->mass() * ( v_ - r_.cross( w_ ) ) + M_ * dq;
                L_ = mSubMasses[mBody->rootLink()->index()].Iw * w_ + H_ * dq;
                ofs << " "  << mBody->calcCenterOfMass().transpose();// 重心 2,3,4
                ofs << " " << P_.transpose() << " " << L_.transpose();// 運動量 5,6,7 角運動量 8,9,10
                ofs << endl;
            
            }// end motion loop

            ofs.close();

        }
    }// end poseSeqItem loop



    cout << "Finished RMControl" << endl;
}

CNOID_IMPLEMENT_PLUGIN_ENTRY(RMControlPlugin)

