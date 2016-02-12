/**
   @author Kunio Kojima
*/
#include "MultiContactStabilizerPlugin.h"

using namespace boost;
using namespace cnoid;
using namespace std;
using namespace hrp;

void MultiContactStabilizerPlugin::generateSeq()
{
    stringstream ss;
    ss << mPoseSeqPath.stem().string() << "_MCS_initPL_" << frameRate << "fps.dat";
    ofstream ofs( ((filesystem::path) mPoseSeqPath.parent_path() / ss.str()).string().c_str() );
    ofs << "time initCMx initCMy initCMz initPx initPy initPz initLx initLy initLz" << endl;

    Vector3SeqPtr initCMSeqPtr = mBodyMotionItem->motion()->getOrCreateExtraSeq<Vector3Seq>("initCM");
    Vector3SeqPtr initPSeqPtr = mBodyMotionItem->motion()->getOrCreateExtraSeq<Vector3Seq>("initP");
    Vector3SeqPtr initLSeqPtr = mBodyMotionItem->motion()->getOrCreateExtraSeq<Vector3Seq>("initL");
    for(int i = 0; i < numFrames; ++i){
        updateBodyState(body, motion, i);
        body->calcForwardKinematics(true,true);

        Vector3d P,L,CM;
        CM = body->calcCenterOfMass();
        body->calcTotalMomentum(P,L);
        L -= CM.cross(P);// convert to around CoM

        initCMSeqPtr->at(i) =  CM;
        initPSeqPtr->at(i) = P;
        initLSeqPtr->at(i) = L;

        ofs << i*dt << " " << CM.transpose() <<  " " << P.transpose() << " " << L.transpose() << " " << endl;
    }
    setSubItem("initCM", initCMSeqPtr, mBodyMotionItem);
    setSubItem("initP", initPSeqPtr, mBodyMotionItem);
    setSubItem("initL", initLSeqPtr, mBodyMotionItem);

    ofs.close();
}

void MultiContactStabilizerPlugin::generateContactConstraintParamVec(std::vector<ContactConstraintParam>& ccParamVec, const std::set<Link*>& contactLinkCantidateSet, PoseSeq::iterator poseIter, const PoseSeqPtr& poseSeqPtr)
{
    for(std::set<Link*>::iterator linkIter = contactLinkCantidateSet.begin(); linkIter != contactLinkCantidateSet.end(); ++linkIter){
        int linkIdx = (*linkIter)->index();
        int contactState = getPrevContactState(poseIter, poseSeqPtr, linkIdx);
        if(contactState < 2){// 接触フラグが0か1 要改良
            ContactConstraintParam ccParam;
            ccParam.contactState = contactState;
            ccParam.linkName = (*linkIter)->name();

            ccParam.numEquals = 0;// 接触条件ごとの等式条件は0個 Fzの制約は全接触点の合計

            // 接触点座標で変化しないローカルな値は代入できる
            ccParam.mu = 0.5;// 要改良
            ccParam.numInequals = 4;// 静止摩擦制約式数

            Vector3 edge;// 要改良 直線の係数a,b,cを代入
            edge << -1, 0, 0.05;
            ccParam.edgeVec.push_back(edge);
            edge << 1, 0, -0.05;
            ccParam.edgeVec.push_back(edge);
            edge << 0, -1, 0.1;
            ccParam.edgeVec.push_back(edge);
            edge << 0, 1, -0.1;
            ccParam.edgeVec.push_back(edge);

            ccParam.numInequals += ccParam.edgeVec.size();// cop制約式数

            ccParamVec.push_back(ccParam);
        }
    }
}

void MultiContactStabilizerPlugin::generateMultiContactStabilizerParam(MultiContactStabilizerParam& mcsParam, BodyPtr body, std::vector<ContactConstraintParam>& ccParamVec, Vector3& lastP)
{
    static Vector3 g;
    g << 0,0,9.8;

    // CM,P,L
    Vector3d CM,P,L,F;
    CM = body->calcCenterOfMass();
    body->calcTotalMomentum(P,L);
    L -= CM.cross(P);// convert to around CoM
    mcsParam.CM = CM;
    mcsParam.P = P;
    mcsParam.L = L;
    mcsParam.F = (P - lastP)/dt + body->mass()*g;

    // 接触点座標系の更新 等式と不等式数の合計
    for(std::vector<ContactConstraintParam>::iterator iter = ccParamVec.begin(); iter != ccParamVec.end(); ++iter){
        (*iter).p = body->link((*iter).linkName)->p();
        (*iter).R =  body->link((*iter).linkName)->R();
        mcsParam.numEquals += (*iter).numEquals;
        mcsParam.numInequals += (*iter).numInequals;
    }
    mcsParam.ccParamVec = ccParamVec;

    lastP = P;
}

void MultiContactStabilizerPlugin::execControl()
{
    stringstream ss,fnamess;
    MessageView::instance()->putln("MultiContactStabilizer called !");
    cout << "MultiContactStabilizer()" << endl;

    ItemList<PoseSeqItem> selectedItemList = ItemTreeView::mainInstance()->selectedItems<Item>();
    if(selectedItemList.empty()){cout << "Select PoseSeqItem!!" << endl; return;}
    BodyItem* pBodyItem = selectedItemList[0]->findOwnerItem<BodyItem>();
    body = pBodyItem->body();
    cout << "Robot:" << body->name() << endl;
    PoseSeqItem* pPoseSeqItem = selectedItemList[0];
    cout << pPoseSeqItem->name() << endl;
    mPoseSeqPath = boost::filesystem::path(pPoseSeqItem->filePath());
    cout << "PoseSeqPath: " << mPoseSeqPath << endl;

    PoseSeqPtr poseSeqPtr = pPoseSeqItem->poseSeq();
    mBodyMotionItem = pPoseSeqItem->bodyMotionItem();
    motion = mBodyMotionItem->motion();

    // BodyMotion作成
    BodyMotionGenerationBar* bmgb = BodyMotionGenerationBar::instance();
    PoseProvider* provider = pPoseSeqItem->interpolator().get();
    bmgb->shapeBodyMotion(body, provider, mBodyMotionItem, true);
    cout << "Generated motion" << endl;

    frameRate = motion->frameRate();
    dt = 1.0/frameRate;
    numFrames = motion->numFrames();

    generateSeq();

    // 接触候補セットの作成
    std::set<Link*> contactLinkCantidateSet;
    for(PoseSeq::iterator poseIter = (++poseSeqPtr->begin()); poseIter != poseSeqPtr->end(); incContactPose(poseIter,poseSeqPtr,body)){
        cout << endl << endl;
        if(!isContactStateChanging(poseIter, poseSeqPtr, body))continue;
        PosePtr curPosePtr = poseIter->get<Pose>();
        for(Pose::LinkInfoMap::iterator linkInfoIter = curPosePtr->ikLinkBegin(); linkInfoIter != curPosePtr->ikLinkEnd(); ++linkInfoIter){
            if(linkInfoIter->second.isTouching() && !linkInfoIter->second.isSlave())contactLinkCantidateSet.insert(body->link(linkInfoIter->first));//接触している且つslaveでない
        }
    }

    MultiContactStabilizer* mcs = new MultiContactStabilizer();
    mcs->m = body->mass();
    mcs->dt = dt;
    mcs->numWindows = 10;

    // モーション走査
    fnamess.str("");
    fnamess << mPoseSeqPath.stem().string() << "_MCS_refPL_" << frameRate << "fps.dat";
    ofstream ofs( ((filesystem::path) mPoseSeqPath.parent_path() / fnamess.str()).string().c_str() );
    ofs << "time refCMx refCMy refCMz refPx refPy refPz refLx refLy refLz" << endl;

    Vector3SeqPtr refCMSeqPtr = mBodyMotionItem->motion()->getOrCreateExtraSeq<Vector3Seq>("refCM");
    Vector3SeqPtr refPSeqPtr = mBodyMotionItem->motion()->getOrCreateExtraSeq<Vector3Seq>("refP");
    Vector3SeqPtr refLSeqPtr = mBodyMotionItem->motion()->getOrCreateExtraSeq<Vector3Seq>("refL");

    Vector3d lastP;
    {
        Vector3d tmpL;
        updateBodyState(body, motion, 0);
        body->calcForwardKinematics(true, true);
        body->calcCenterOfMass();
        body->calcTotalMomentum(lastP,tmpL);
    }
    for(PoseSeq::iterator frontPoseIter = (++poseSeqPtr->begin()),backPoseIter = poseSeqPtr->begin(); frontPoseIter != poseSeqPtr->end(); backPoseIter = frontPoseIter,incContactPose(frontPoseIter,poseSeqPtr,body)){
        if(!isContactStateChanging(frontPoseIter, poseSeqPtr, body)) continue;

        // 接触状態依存のパラメータのみ設定(動作軌道に依存するパラメータは後で設定)
        std::vector<ContactConstraintParam> ccParamVec;
        generateContactConstraintParamVec(ccParamVec, contactLinkCantidateSet, frontPoseIter, poseSeqPtr);

        for(int i = backPoseIter->time()/dt; i < frontPoseIter->time()/dt; ++i){
            cout << endl << "turn:" << i << endl;

            updateBodyState(body, motion, i);
            body->calcForwardKinematics(true, true);

            MultiContactStabilizerParam mcsParam = MultiContactStabilizerParam(mcs);
            // 動作軌道に依存するパラメータの設定
            generateMultiContactStabilizerParam(mcsParam, body, ccParamVec, lastP);

            if(mcs->mpcParamDeque.size() == mcs->numWindows){
                mcs->calcAugmentedMatrix();// phi,psi,W1,W2 U->x0
                mcs->setupQP();
                mcs->execQP();

                VectorXd x0(mcs->stateDim);
                x0 = mcs->x0;
                Vector3d CM,P,L;
                CM << x0[0],x0[2],0;
                P << x0[1],x0[3],0;
                L << x0[4],x0[5],0;
                refCMSeqPtr->at(i-mcs->numWindows) = CM;
                refPSeqPtr->at(i-mcs->numWindows) = P;
                refLSeqPtr->at(i-mcs->numWindows) = L;
                ofs << i*dt << " " << CM.transpose() <<  " " << P.transpose() << " " << L.transpose() << " " << endl;

                mcs->mpcParamDeque.pop_front();
            }
            ModelPreviewControllerParam mpcParam;
            mcsParam.convertToMPCParam(mpcParam);

            mcs->mcsParamDeque.push_back(mcsParam);
            mcs->mpcParamDeque.push_back(mpcParam);
        }
        // numWindows回数 Dequeの最後を取り出して最後に追加しながらproc
    }

    free(mcs);

    setSubItem("refCM", refCMSeqPtr, mBodyMotionItem);
    setSubItem("refP", refPSeqPtr, mBodyMotionItem);
    setSubItem("refL", refLSeqPtr, mBodyMotionItem);

    cout << "Finished MultiContactStabilizer" << endl;
}

CNOID_IMPLEMENT_PLUGIN_ENTRY(MultiContactStabilizerPlugin)

