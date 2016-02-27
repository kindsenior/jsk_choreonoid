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
        // L -= CM.cross(P);// convert to around CoM
        L << 0,0,0;

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

void MultiContactStabilizerPlugin::generateContactConstraintParamVec(std::vector<ContactConstraintParam*>& ccParamVec, const std::set<Link*>& contactLinkCantidateSet, PoseSeq::iterator poseIter, const PoseSeqPtr& poseSeqPtr)
{
    for(std::set<Link*>::iterator linkIter = contactLinkCantidateSet.begin(); linkIter != contactLinkCantidateSet.end(); ++linkIter){
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

void MultiContactStabilizerPlugin::generateMultiContactStabilizerParam(MultiContactStabilizerParam* mcsParam, BodyPtr body, std::vector<ContactConstraintParam*>& ccParamVec)
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

void MultiContactStabilizerPlugin::processCycle(int i, std::vector<ContactConstraintParam*>& ccParamVec)
{
    cout << endl << "##############################" << endl << "turn:" << i << endl;

    clock_t st = clock();
    double tmList[4] = {0,0,0,0};

    updateBodyState(body, motion, min(i,numFrames-1));
    body->calcForwardKinematics(true, true);

    MultiContactStabilizerParam* mcsParam = new MultiContactStabilizerParam(mcs);
    // 動作軌道に依存するパラメータの設定
    generateMultiContactStabilizerParam(mcsParam, body, ccParamVec);

    clock_t et = clock();
    tmList[0] = (double) 1000*(et-st)/CLOCKS_PER_SEC;
    st = et;

    mcsParam->convertToMPCParam();
    mcs->mpcParamDeque.push_back(mcsParam);

		et = clock();
		tmList[1] = (double) 1000*(et-st)/CLOCKS_PER_SEC;
		st = et;

    if(mcs->mpcParamDeque.size() == mcs->numWindows){
        mcs->calcAugmentedMatrix();// phi,psi,W1,W2 U->x0
        mcs->setupQP();

        et = clock();
        tmList[2] = (double) 1000*(et-st)/CLOCKS_PER_SEC;
        st = et;

        if(mcs->execQP()) failIdxVec.push_back(i - mcs->numWindows);

				et = clock();
				tmList[3] = (double) 1000*(et-st)/CLOCKS_PER_SEC;
				st = et;

        mcs->updateX0Vector();
        VectorXd x0(mcs->stateDim);
        x0 = mcs->x0;
        Vector3d CM,P,L;
        CM << x0[0],x0[2],0;
        P << x0[1],x0[3],0;
        L << x0[4],x0[5],0;
        CM /= body->mass();
        mRefCMSeqPtr->at(i - mcs->numWindows + 1) = CM;
        mRefPSeqPtr->at(i - mcs->numWindows + 1) = P;
        mRefLSeqPtr->at(i - mcs->numWindows + 1) = L;
        mOfs << (i - mcs->numWindows + 1)*dt << " " << CM.transpose() <<  " " << P.transpose() << " " << L.transpose() << " " << endl;

        mcs->mpcParamDeque.pop_front();
    }

    cout << "  FK: " << tmList[0] << "[ms]  convert: " << tmList[1] << "[ms]  setup: " << tmList[2] << "[ms]  QP: " << tmList[3]  << "[ms]" << endl;
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

    mcs = new MultiContactStabilizer();
    mcs->m = body->mass();
    mcs->dt = dt;
    mcs->numWindows = mBar->dialog->numWindowsSpin->value();
    mcs->errorCMWeight = mBar->dialog->errorCMWeightSpin->value();
    mcs->errorMomentumWeight = mBar->dialog->errorMomentumWeightSpin->value();
    mcs->errorAngularMomentumWeight = mBar->dialog->errorAngularMomentumWeightSpin->value();
    mcs->inputForceWeight = mBar->dialog->inputForceWeightSpin->value();
    mcs->inputMomentWeight = mBar->dialog->inputMomentWeightSpin->value();

    // モーション走査
    fnamess.str("");
    fnamess << mPoseSeqPath.stem().string() << "_MCS_refPL";
    if(mBar->dialog->saveParameterInFileNameCheck.isChecked()) fnamess << mBar->dialog->getParamString();
    fnamess << "_" << frameRate << "fps.dat";
    mOfs.open( ((filesystem::path) mPoseSeqPath.parent_path() / fnamess.str()).string().c_str(), ios::out );
    mOfs << "time refCMx refCMy refCMz refPx refPy refPz refLx refLy refLz" << endl;

    mRefCMSeqPtr = mBodyMotionItem->motion()->getOrCreateExtraSeq<Vector3Seq>("refCM");
    mRefPSeqPtr = mBodyMotionItem->motion()->getOrCreateExtraSeq<Vector3Seq>("refP");
    mRefLSeqPtr = mBodyMotionItem->motion()->getOrCreateExtraSeq<Vector3Seq>("refL");

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
        std::vector<ContactConstraintParam*> ccParamVec;
        generateContactConstraintParamVec(ccParamVec, contactLinkCantidateSet, frontPoseIter, poseSeqPtr);

        for(int i = backPoseIter->time()/dt; i < frontPoseIter->time()/dt; ++i){
            processCycle(i, ccParamVec);
        }
    }

    {// numWindows回数 Dequeの最後を取り出して最後に追加しながらproc
        std::vector<ContactConstraintParam*> ccParamVec;
        generateContactConstraintParamVec(ccParamVec, contactLinkCantidateSet, --poseSeqPtr->end(), poseSeqPtr);

        for(int i = numFrames - 1; i < numFrames + mcs->numWindows; ++i){
            processCycle(i, ccParamVec);
        }
    }

    free(mcs);

    setSubItem("refCM", mRefCMSeqPtr, mBodyMotionItem);
    setSubItem("refP", mRefPSeqPtr, mBodyMotionItem);
    setSubItem("refL", mRefLSeqPtr, mBodyMotionItem);

    mOfs.close();

    for(std::vector<int>::iterator iter = failIdxVec.begin(); iter != failIdxVec.end(); ++iter) cout << "Failed in " << *iter << ":(" << (*iter)*dt << " sec)" << endl;
    if(failIdxVec.empty()) cout << "All QP succeeded" << endl;
    failIdxVec.clear();

    cout << "Finished MultiContactStabilizer" << endl;
}

CNOID_IMPLEMENT_PLUGIN_ENTRY(MultiContactStabilizerPlugin)
