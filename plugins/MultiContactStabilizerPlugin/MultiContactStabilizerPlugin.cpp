/**
   @author Kunio Kojima
*/
#include "MultiContactStabilizerPlugin.h"

using namespace boost;
using namespace cnoid;
using namespace std;
using namespace qpOASES;

void MultiContactStabilizerPlugin::generateSeq()
{
    stringstream ss;
    ss << mPoseSeqPath.stem().string() << "_MCS_initPL_" << frameRate << "fps.dat";
    ofstream ofs( ((filesystem::path) mPoseSeqPath.parent_path() / ss.str()).string().c_str() );
    ofs << "time initCM initCMy initCMz initPx initPy initPz initLx initLy initLz" << endl;

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

void MultiContactStabilizerPlugin::MultiContactStabilizer()
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

    // 左右足Pathを設定
    // mJpl = getCustomJointPath( body, body->rootLink(), mLFootLink );
    // mJpr = getCustomJointPath( body, body->rootLink(), mRFootLink );

    //     BodyMotionItem* bodyMotionItem = pPoseSeqItem->bodyMotionItem();
    //     BodyMotionPtr motion = bodyMotionItem->motion();

    //     // BodyMotion作成
    //     BodyMotionGenerationBar* bmgb = BodyMotionGenerationBar::instance();// インスタンス生成
    //     PoseProvider* provider = pPoseSeqItem->interpolator().get();
    //     bmgb->shapeBodyMotion(body, provider, bodyMotionItem, true);
    //     cout << " Generated motion" << endl;

    //     // 目標運動量軌道作成
    //     Vector3Seq refPSeq,refLSeq;
    //     Vector3d initDCM,endDCM,initL,endL;// initDCMとendPを明示的に0で初期化していない
    //     generateRefPLSeq( bodyMotionItem, pPoseSeqItem->poseSeq(), initDCM, endDCM, initL, endL, refPSeq, refLSeq );
    //     // loadRefPLSeq( bodyMotionItem, pPoseSeqItem->poseSeq(), initP, endP, initL, endL, refPSeq, refLSeq );
    //     cout << " Generated ref P/L" << endl;

    //         fnamess.str("");
    //         fnamess << mPoseSeqPath.parent_path().string() << "/" << getBasename(mPoseSeqPath) << "_RMC_PL_" << motion->frameRate() << "fps_" << i << ".dat";
    //         ofstream ofs( fnamess.str().c_str() );
    //         if( !ofs ){ cerr << "File Open Error" << endl; return;}
    //         ofs << "time CMx CMy CMz Px Py Pz Lx Ly Lz" << endl;

    //         double dt = 1.0/motion->frameRate();

    //         // motion loop 計算 step 2 〜 6
    //         // for(int currentFrame = 0; currentFrame < 1; ++currentFrame){
    //         for(int currentFrame = 0; currentFrame < motion->numFrames(); ++currentFrame){
    //             ofs << currentFrame * dt;

    //             // prevFrame,nextFrame設定
    //             int prevFrame = std::max(currentFrame - 1, 0);
    //             int nextFrame = std::min(currentFrame + 1, motion->numFrames() - 1);

    //             // motion->frame(nextFrame) >> *body;
    //             // body->calcForwardKinematics();

    //             motion->frame(nextFrame) << *body;// frame更新
            
    //         }// end motion loop

    //         ofs.close();

    //     }
    // }// end poseSeqItem loop

    cout << "Finished MultiContactStabilizer" << endl;
}

CNOID_IMPLEMENT_PLUGIN_ENTRY(MultiContactStabilizerPlugin)

