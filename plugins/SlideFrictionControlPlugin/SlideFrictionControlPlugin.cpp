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

void cnoid::sweepControl(boost::filesystem::path poseSeqPath ,std::string paramStr, SlideFrictionControl* sfc, BodyPtr& body, BodyMotionItemPtr& bodyMotionItemPtr, const std::set<Link*>& contactLinkCandidateSet)
{
    stringstream fnamess; fnamess.str("");
    fnamess << poseSeqPath.stem().string() << "_SFC_refPL" << paramStr << "_" << (int) 1/sfc->rootController()->dt << "fps.dat";
    ofstream refPLOfs;
    refPLOfs.open(((filesystem::path) poseSeqPath.parent_path() / fnamess.str()).string().c_str(), ios::out);

    const double dt = sfc->dt;
    const int cycle = sfc->dt*bodyMotionItemPtr->motion()->frameRate();
    const int numFrames = bodyMotionItemPtr->motion()->numFrames()*sfc->rootController()->dt/sfc->dt;

    float avgTime = 0;
    std::vector<int> failIdxVec;

    refPLOfs << "time refCMx refCMy refCMz refPx refPy refPz refLx refLy refLz processTime" << endl;

    Vector3SeqPtr refCMSeqPtr = bodyMotionItemPtr->motion()->getOrCreateExtraSeq<Vector3Seq>("refCM");
    Vector3SeqPtr refPSeqPtr = bodyMotionItemPtr->motion()->getOrCreateExtraSeq<Vector3Seq>("refP");
    Vector3SeqPtr refLSeqPtr = bodyMotionItemPtr->motion()->getOrCreateExtraSeq<Vector3Seq>("refL");
    Vector3SeqPtr refZmpSeqPtr = bodyMotionItemPtr->motion()->getOrCreateExtraSeq<Vector3Seq>("ZMP");

    fnamess.str("");
    fnamess << poseSeqPath.stem().string() << "_SFC_contact_" << sfc->dt << "dt_" << (int) 1/sfc->rootController()->dt << "fps.dat";
    ofstream contactOfs;
    contactOfs.open(((filesystem::path) poseSeqPath.parent_path() / fnamess.str()).string().c_str(), ios::out);
    contactOfs << "time lpx lpy lpz lvx lvy lvz lwx lwy lwz rpx rpy rpz rvx rvy rvz rwx rwy rwz" << endl;

    fnamess.str("");
    fnamess << poseSeqPath.stem().string() << "_SFC_wrench" << paramStr << "_" << (int) 1/sfc->rootController()->dt << "fps.dat";
    ofstream wrenchOfs;
    wrenchOfs.open(((filesystem::path) poseSeqPath.parent_path() / fnamess.str()).string().c_str(), ios::out);
    wrenchOfs << "time lfx lfy lfz lnx lny lnz rfx rfy rfz rnx rny rnz" << endl;

    fnamess.str("");
    fnamess << poseSeqPath.stem().string() << "_SFC_inputPL_" << sfc->dt << "dt_" << (int) 1/sfc->rootController()->dt << "fps.dat";
    ofstream inputPLOfs;
    inputPLOfs.open(((filesystem::path) poseSeqPath.parent_path() / fnamess.str()).string().c_str(), ios::out);
    inputPLOfs << "time CMx CMy CMz Px Py Pz Lx Ly Lz Fx Fy Fz" << endl;

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
            L << x0[4],x0[5],x0[6];
            CM /= body->mass();
            int motionIdx = (i - sfc->numWindows())*cycle;
            refCMSeqPtr->at(motionIdx) = CM;
            refPSeqPtr->at(motionIdx) = P;
            refLSeqPtr->at(motionIdx) = L;
            refPLOfs << (i - sfc->numWindows())*dt << " " << CM.transpose() <<  " " << P.transpose() << " " << L.transpose() << " " << processedTime << endl;

            SlideFrictionControlParam sfcParam = *(sfc->prevMpcParam);
            inputPLOfs << (i - sfc->numWindows())*dt << " " << sfcParam.CM.transpose() << " "<< sfcParam.P.transpose() << " " << sfcParam.L.transpose() << " " << sfcParam.F.transpose() << endl;

            contactOfs << (i - sfc->numWindows())*dt;
            wrenchOfs << (i - sfc->numWindows())*dt;
            std::vector<ContactConstraintParam*> ccParamVec = sfcParam.ccParamVec;
            VectorXd u0 = sfc->u0;
            double Fz = 0;
            Vector3d zmp = Vector3d::Zero();
            for(std::set<Link*>::iterator linkIter = contactLinkCandidateSet.begin(); linkIter != contactLinkCandidateSet.end(); ++linkIter){
                int colIdx = 0;
                for(std::vector<ContactConstraintParam*>::iterator iter = ccParamVec.begin(); iter != ccParamVec.end(); ++iter){
                    ContactConstraintParam* ccParam = *iter;
                    int inputDim = ccParam->inputDim;
                    if((*linkIter)->name() == ccParam->linkName){
                        contactOfs << " "<< ccParam->p.transpose() << " " << ccParam->v.transpose() << " " << ccParam->w.transpose();
                        VectorXd u = ccParam->inputForceConvertMat*u0.segment(colIdx,inputDim);
                        wrenchOfs << " " << u.transpose();

                        //calc ZMP 足は床上である前提
                        Vector3d f = ccParam->R*u.head(3);
                        Vector3d n = ccParam->R*u.tail(3);
                        Fz += f.z();
                        zmp.x() += -n.y()+ccParam->p.x()*f.z();
                        zmp.y() +=  n.x()+ccParam->p.y()*f.z();
                        goto END;
                    }
                    colIdx += inputDim;
                }
                contactOfs << " 0 0 0  0 0 0  0 0 0";
                wrenchOfs << " 0 0 0  0 0 0";
            END:
                ;
            }
            zmp /= Fz;
            refZmpSeqPtr->at(motionIdx) = zmp;
            contactOfs << endl;
            wrenchOfs << endl;
        }
    }
 // BREAK:

    setSubItem("refCM", refCMSeqPtr, bodyMotionItemPtr);
    setSubItem("refP", refPSeqPtr, bodyMotionItemPtr);
    setSubItem("refL", refLSeqPtr, bodyMotionItemPtr);
    setSubItem("ZMP", refZmpSeqPtr, bodyMotionItemPtr);

    refPLOfs.close();
    contactOfs.close();
    wrenchOfs.close();
    inputPLOfs.close();

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
    body->calcForwardKinematics();// use end effector's velocity
    Vector3d lastCM = body->calcCenterOfMass();
    // body->calcTotalMomentum(lastMomentum,tmpL);
    lastMomentum << 0,0,0;

    // cnoidのクラス(BodyMotion)からmpcParamDequeを生成
    int index = 0;
    for(PoseSeq::iterator frontPoseIter = (++poseSeqPtr->begin()),backPoseIter = poseSeqPtr->begin(); frontPoseIter != poseSeqPtr->end(); backPoseIter = frontPoseIter,incContactPose(frontPoseIter,poseSeqPtr,body)){
        if(!isContactStateChanging(frontPoseIter, poseSeqPtr, body)) continue;

        // 接触状態依存のパラメータのみ設定(動作軌道に依存するパラメータは後で設定)
        std::vector<ContactConstraintParam*> ccParamVec;
        generateContactConstraintParamVec2(ccParamVec, contactLinkCandidateSet, frontPoseIter, poseSeqPtr);

        for(int i=backPoseIter->time()*frameRate; i < frontPoseIter->time()*frameRate; ++i){
            updateBodyState(body, motion, min(i,numFrames-1), contactLinkCandidateSet);
            body->calcForwardKinematics();// use end effector's velocity

            SlideFrictionControlParam* sfcParam = new SlideFrictionControlParam(index, sfc);
            // 動作軌道に依存するパラメータの設定
            generateSlideFrictionControlParam(sfcParam, lastCM, lastMomentum, body, ccParamVec, dt);

            sfc->preMpcParamDeque.push_back((SlideFrictionControlParam*) sfcParam);
            ++index;
        }
    }
}

void cnoid::generateContactConstraintParamVec2(std::vector<ContactConstraintParam*>& ccParamVec, const std::set<Link*>& contactLinkCandidateSet, PoseSeq::iterator poseIter, const PoseSeqPtr& poseSeqPtr)
{
    cout << "generateContactConstraintParamVec2( ~" << poseIter->time() << "[sec] )" << endl;
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
            if(contactState == 0){// static contact
                // ccParamVec.push_back(new SimpleContactConstraintParam((*linkIter)->name(), vertexVec));
                // ccParamVec.push_back(new StaticContactConstraintParam((*linkIter)->name(), vertexVec, 0.5));// 摩擦係数 要改良
                // ccParamVec.push_back(new DistributedForceContactConstraintParam((*linkIter)->name(), vertexVec, 2,2));
                ccParamVec.push_back(new DistributedForceStaticContactConstraintParam((*linkIter)->name(), vertexVec, 2,2, 0.5));// 摩擦係数 要改良
                cout << " " << (*linkIter)->name() << ": Static" << endl;
            }else if(contactState == 1){// slide contact
                // ccParamVec.push_back(new SlideContactConstraintParam((*linkIter)->name(), vertexVec, 0.5, getPrevDirection(poseIter, poseSeqPtr, linkIdx)));
                ccParamVec.push_back(new DistributedForceSlideContactConstraintParam((*linkIter)->name(), vertexVec, 2,2, 0.5));
                cout << " " << (*linkIter)->name() << ": Slide" << endl;
            }
        }
    }
    cout << endl << endl;
}

void cnoid::generateSlideFrictionControlParam(SlideFrictionControlParam* sfcParam,Vector3d& lastCM, Vector3d& lastMomentum, BodyPtr& body, std::vector<ContactConstraintParam*>& ccParamVec, double dt)
{

    static Vector3 g;
    g << 0,0,9.8;

    // CM,P,L
    Vector3d CM,P,L,F;
    CM = body->calcCenterOfMass();
    // body->calcTotalMomentum(P,L);
    P = (CM - lastCM)/dt;// overwrite P
    P << 0,0,0;// overwrite P
    // L -= CM.cross(P);// convert to around CoM
    // sfcParam->CM = CM;
    sfcParam->CM << 0.065,0,0;
    sfcParam->CM += CM;
    sfcParam->P = P;
    // sfcParam->L = L;
    sfcParam->L << 0,0,0;
    sfcParam->F = (P - lastMomentum)/dt + body->mass()*g;

    // 接触点座標系の更新 等式と不等式数の合計
    for(std::vector<ContactConstraintParam*>::iterator iter = ccParamVec.begin(); iter != ccParamVec.end(); ++iter){
        (*iter)->p = body->link((*iter)->linkName)->p();
        (*iter)->R = body->link((*iter)->linkName)->R();
        // (*iter)->v = body->link((*iter)->linkName)->v();
        // (*iter)->w = body->link((*iter)->linkName)->w();
        (*iter)->v << 0,body->link((*iter)->linkName)->v().y(),0;// overwrite
        (*iter)->w << 0,0,body->link((*iter)->linkName)->w().z();// overwrite

        if(typeid(**iter) == typeid(SimpleContactConstraintParam)){
            sfcParam->ccParamVec.push_back(new SimpleContactConstraintParam(dynamic_cast<SimpleContactConstraintParam*>(*iter)));
        }else if(typeid(**iter) == typeid(StaticContactConstraintParam)){
            sfcParam->ccParamVec.push_back(new StaticContactConstraintParam(dynamic_cast<StaticContactConstraintParam*>(*iter)));
        }else if(typeid(**iter) == typeid(SlideContactConstraintParam)){
            sfcParam->ccParamVec.push_back(new SlideContactConstraintParam(dynamic_cast<SlideContactConstraintParam*>(*iter)));
        }else if(typeid(**iter) == typeid(DistributedForceContactConstraintParam)){
            sfcParam->ccParamVec.push_back(new DistributedForceContactConstraintParam(dynamic_cast<DistributedForceContactConstraintParam*>(*iter)));
        }else if(typeid(**iter) == typeid(DistributedForceStaticContactConstraintParam)){
            sfcParam->ccParamVec.push_back(new DistributedForceStaticContactConstraintParam(dynamic_cast<DistributedForceStaticContactConstraintParam*>(*iter)));
        }else if(typeid(**iter) == typeid(DistributedForceSlideContactConstraintParam)){
            sfcParam->ccParamVec.push_back(new DistributedForceSlideContactConstraintParam(dynamic_cast<DistributedForceSlideContactConstraintParam*>(*iter)));
        }else{
            cout << "Warning! ContactConstraintParam's pointer Copy Constructor called" << endl;
            sfcParam->ccParamVec.push_back(new ContactConstraintParam(dynamic_cast<ContactConstraintParam*>(*iter)));
        }
    }

    lastCM = CM;
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
    sfc->errorYawAngularMomentumWeight = (mBar->dialog->layout())->errorYawAngularMomentumWeightSpin->value();
    sfc->inputForceWeight = (mBar->dialog->layout())->inputForceWeightSpin->value();
    sfc->inputMomentWeight = (mBar->dialog->layout())->inputMomentWeightSpin->value();
    sfc->inputYawMomentWeight = (mBar->dialog->layout())->inputYawMomentWeightSpin->value();

    generatePreModelPredictiveControlParamDeque(sfc, body, poseSeqPtr, motion, contactLinkCandidateSet);

    sweepControl(mPoseSeqPath, mBar->dialog->layout()->getParamString(), sfc, body, mBodyMotionItemPtr, contactLinkCandidateSet);

    cout << "Finished SlideFrictionControl" << endl;
}

CNOID_IMPLEMENT_PLUGIN_ENTRY(SlideFrictionControlPlugin)
