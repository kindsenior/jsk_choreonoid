/**
   @author Kunio Kojima
*/
#include "SlideFrictionControlPlugin.h"

using namespace boost;
using namespace cnoid;
using namespace std;
using namespace hrp;

bool SlideFrictionControlPlugin::initialize()
{
    mBar = new SlideFrictionControlBar(this);
    addToolBar(mBar);
    return true;
}

void cnoid::sweepControl(ofstream& ofs, SlideFrictionControl* sfc, BodyPtr& body, BodyMotionItemPtr& bodyMotionItemPtr)
{
    const double dt = sfc->dt;
    const int numFrames = bodyMotionItemPtr->motion()->numFrames()*sfc->rootController()->dt/sfc->dt;

    float avgTime = 0;
    std::vector<int> failIdxVec;

    ofs << "time refCMx refCMy refCMz refPx refPy refPz refLx refLy refLz processTime" << endl;

    Vector3SeqPtr refCMSeqPtr = bodyMotionItemPtr->motion()->getOrCreateExtraSeq<Vector3Seq>("refCM");
    Vector3SeqPtr refPSeqPtr = bodyMotionItemPtr->motion()->getOrCreateExtraSeq<Vector3Seq>("refP");
    Vector3SeqPtr refLSeqPtr = bodyMotionItemPtr->motion()->getOrCreateExtraSeq<Vector3Seq>("refL");

    for(int i=0; i < numFrames + sfc->numWindows(); ++i){
        // if(i > sfc->numWindows() + 1) goto BREAK;

        cout << endl << "##############################" << endl << "processCycle() turn:" << i << endl;
        float processedTime;
        if(sfc->processCycle(processedTime)) failIdxVec.push_back(i - sfc->numWindows());
        avgTime += processedTime;

        if(i >= sfc->numWindows()){
            VectorXd x0(sfc->stateDim);
            x0 = sfc->x0;

            Vector3d CM,P,L;
            CM << x0[0],x0[2],0;
            P << x0[1],x0[3],0;
            L << x0[4],x0[5],0;
            CM /= body->mass();
            refCMSeqPtr->at(i - sfc->numWindows()) = CM;
            refPSeqPtr->at(i - sfc->numWindows()) = P;
            refLSeqPtr->at(i - sfc->numWindows()) = L;
            ofs << (i - sfc->numWindows())*dt << " " << CM.transpose() <<  " " << P.transpose() << " " << L.transpose() << " " << processedTime << endl;
        }
    }
 // BREAK:

    setSubItem("refCM", refCMSeqPtr, bodyMotionItemPtr);
    setSubItem("refP", refPSeqPtr, bodyMotionItemPtr);
    setSubItem("refL", refLSeqPtr, bodyMotionItemPtr);

    ofs.close();

    for(std::vector<int>::iterator iter = failIdxVec.begin(); iter != failIdxVec.end(); ++iter) cout << "Failed in " << *iter << ":(" << (*iter)*dt << " sec)" << endl;
    if(failIdxVec.empty()) cout << "All QP succeeded" << endl;
    failIdxVec.clear();

    cout << "total time: " << avgTime/1000.0 << "[sec]" << endl;
    cout << "average time: " << avgTime/numFrames << "[msec]" << endl;
}

void cnoid::generatePreModelPredictiveControlParamDeque(SlideFrictionControl* sfc, BodyPtr body, const PoseSeqPtr poseSeqPtr, const BodyMotionPtr& motion, const std::set<Link*>& contactLinkCandidateSet)
{
    const int frameRate = motion->frameRate();
    const int numFrames = motion->numFrames();
    const double dt = 1.0/motion->frameRate();
    Vector3d lastMomentum, tmpL;
    updateBodyState(body, motion, 0);
    body->calcForwardKinematics(true, true);
    body->calcCenterOfMass();
    body->calcTotalMomentum(lastMomentum,tmpL);

    // cnoidのクラス(BodyMotion)からmpcParamDequeを生成
    int index = 0;
    for(PoseSeq::iterator frontPoseIter = (++poseSeqPtr->begin()),backPoseIter = poseSeqPtr->begin(); frontPoseIter != poseSeqPtr->end(); backPoseIter = frontPoseIter,incContactPose(frontPoseIter,poseSeqPtr,body)){
        if(!isContactStateChanging(frontPoseIter, poseSeqPtr, body)) continue;

        // 接触状態依存のパラメータのみ設定(動作軌道に依存するパラメータは後で設定)
        std::vector<ContactConstraintParam*> ccParamVec;
        generateContactConstraintParamVec2(ccParamVec, contactLinkCandidateSet, frontPoseIter, poseSeqPtr);

        for(int i=backPoseIter->time()*frameRate; i < frontPoseIter->time()*frameRate; ++i){
            updateBodyState(body, motion, min(i,numFrames-1));
            body->calcForwardKinematics(true, true);

            SlideFrictionControlParam* sfcParam = new SlideFrictionControlParam(index, sfc);
            // 動作軌道に依存するパラメータの設定
            generateSlideFrictionControlParam(sfcParam, lastMomentum, body, ccParamVec, dt);

            sfc->preMpcParamDeque.push_back((SlideFrictionControlParam*) sfcParam);
            ++index;
        }
    }
}

void cnoid::generateContactConstraintParamVec2(std::vector<ContactConstraintParam*>& ccParamVec, const std::set<Link*>& contactLinkCandidateSet, PoseSeq::iterator poseIter, const PoseSeqPtr& poseSeqPtr)
{
    for(std::set<Link*>::iterator linkIter = contactLinkCandidateSet.begin(); linkIter != contactLinkCandidateSet.end(); ++linkIter){
        int linkIdx = (*linkIter)->index();
        int contactState = getPrevContactState(poseIter, poseSeqPtr, linkIdx);
        if(contactState < 2){// 接触フラグが0か1 要改良
            std::vector<hrp::Vector2> vertexVec;
            hrp::Vector2 vertex;// 頂点の2次元座標を代入
            vertex << 0.1,0.05; vertexVec.push_back(vertex);
            vertex << 0.1,-0.05; vertexVec.push_back(vertex);
            vertex << -0.1,0.05; vertexVec.push_back(vertex);
            vertex << -0.1,-0.05; vertexVec.push_back(vertex);
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

void cnoid::generateSlideFrictionControlParam(SlideFrictionControlParam* sfcParam, Vector3d& lastMomentum, BodyPtr body, std::vector<ContactConstraintParam*>& ccParamVec, double dt)
{

    static Vector3 g;
    g << 0,0,9.8;

    // CM,P,L
    Vector3d CM,P,L,F;
    CM = body->calcCenterOfMass();
    body->calcTotalMomentum(P,L);
    L -= CM.cross(P);// convert to around CoM
    sfcParam->CM = CM;
    sfcParam->P = P;
    // sfcParam->L = L;
    sfcParam->L << 0,0,0;
    sfcParam->F = (P - lastMomentum)/dt + body->mass()*g;

    // 接触点座標系の更新 等式と不等式数の合計
    for(std::vector<ContactConstraintParam*>::iterator iter = ccParamVec.begin(); iter != ccParamVec.end(); ++iter){
        (*iter)->p = body->link((*iter)->linkName)->p();
        (*iter)->R =  body->link((*iter)->linkName)->R();
    }
    sfcParam->ccParamVec = ccParamVec;

    lastMomentum = P;
}

void SlideFrictionControlPlugin::execControl()
{
    stringstream ss,fnamess;
    MessageView::instance()->putln("SlideFrictionControl called !");
    cout << "SlideFrictionControl()" << endl;

    BodyItemPtr bodyItemPtr;
    PoseSeqItemPtr poseSeqItemPtr;
    PoseSeqPtr poseSeqPtr;
    if(!getSelectedPoseSeqSet(bodyItemPtr, body, poseSeqItemPtr, poseSeqPtr, mBodyMotionItemPtr, motion)) return;

    mPoseSeqPath = boost::filesystem::path(poseSeqItemPtr->filePath());
    cout << "PoseSeqPath: " << mPoseSeqPath << endl;

    // BodyMotion作成
    generateBodyMotionFromBar(body, poseSeqItemPtr, mBodyMotionItemPtr);

    frameRate = motion->frameRate();
    dt = 1.0/frameRate;
    numFrames = motion->numFrames();

    generateInitSeq(body, poseSeqItemPtr);

    // 接触候補セットの作成
    std::set<Link*> contactLinkCandidateSet;
    calcContactLinkCandidateSet(contactLinkCandidateSet, body, poseSeqPtr);

    sfc = new SlideFrictionControl();
    sfc->m = body->mass();
    sfc->dt = dt;
    sfc->setBlockVector((mBar->dialog->layout())->blockSpinVec->value());
    sfc->errorCMWeight = (mBar->dialog->layout())->errorCMWeightSpin->value();
    sfc->errorMomentumWeight = (mBar->dialog->layout())->errorMomentumWeightSpin->value();
    sfc->errorAngularMomentumWeight = (mBar->dialog->layout())->errorAngularMomentumWeightSpin->value();
    sfc->inputForceWeight = (mBar->dialog->layout())->inputForceWeightSpin->value();
    sfc->inputMomentWeight = (mBar->dialog->layout())->inputMomentWeightSpin->value();

    generatePreModelPredictiveControlParamDeque(sfc, body, poseSeqPtr, motion, contactLinkCandidateSet);

    fnamess.str("");
    fnamess << mPoseSeqPath.stem().string() << "_SFC_refPL";
    fnamess << mBar->dialog->layout()->getParamString();
    fnamess << "_" << frameRate << "fps.dat";
    mOfs.open( ((filesystem::path) mPoseSeqPath.parent_path() / fnamess.str()).string().c_str(), ios::out );
    mOfs << "time refCMx refCMy refCMz refPx refPy refPz refLx refLy refLz processTime" << endl;

    sweepControl(mOfs, sfc, body, mBodyMotionItemPtr);

    cout << "Finished SlideFrictionControl" << endl;
}

CNOID_IMPLEMENT_PLUGIN_ENTRY(SlideFrictionControlPlugin)
