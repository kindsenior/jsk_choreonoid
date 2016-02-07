/**
   @author Kunio Kojima
*/
#include "MultiContactStabilizerPlugin.h"

using namespace boost;
using namespace cnoid;
using namespace std;

void MultiContactStabilizerPlugin::MultiContactStabilizer(){
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

    PoseSeqPtr poseSeq = pPoseSeqItem->poseSeq();
    BodyMotionItem* pBodyMotionItem = pPoseSeqItem->bodyMotionItem();
    BodyMotionPtr motion = pBodyMotionItem->motion();

    // BodyMotion作成
    BodyMotionGenerationBar* bmgb = BodyMotionGenerationBar::instance();
    PoseProvider* provider = pPoseSeqItem->interpolator().get();
    bmgb->shapeBodyMotion(body, provider, mBodyMotionItem, true);
    cout << "Generated motion" << endl;

    frameRate = motion->frameRate();
    dt = 1.0/frameRate;
    numFrames = motion->numFrames();



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

