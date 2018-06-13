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

void cnoid::loadExtraSeq(boost::filesystem::path poseSeqPath ,std::string paramStr, SlideFrictionControl* sfc, BodyPtr& body, BodyMotionItemPtr& bodyMotionItemPtr, const std::set<Link*>& contactLinkCandidateSet)
{
    cout << "loadExtraSeq()" << endl;
    cout << "contactLinkCandidateSet: ";
    for(auto link : contactLinkCandidateSet) cout << " " << link->name();
    cout << endl;

    stringstream fnamess;
    fnamess << poseSeqPath.stem().string() << "_SFC_refPL" << paramStr << "_" << (int) 1/sfc->rootController()->dt << "fps.dat";
    ifstream refPLIfs;
    refPLIfs.open(((filesystem::path) poseSeqPath.parent_path() / fnamess.str()).string().c_str(), ios::in);

    fnamess.str("");
    fnamess << poseSeqPath.stem().string() << "_SFC_wrench" << paramStr << "_" << (int) 1/sfc->rootController()->dt << "fps.dat";
    ifstream wrenchIfs;
    wrenchIfs.open(((filesystem::path) poseSeqPath.parent_path() / fnamess.str()).string().c_str(), ios::in);
    cout << " file: " << fnamess.str() << endl;

    fnamess.str("");
    fnamess << poseSeqPath.stem().string() << "_SFC_contact_" << sfc->dt << "dt_" << (int) 1/sfc->rootController()->dt << "fps.dat";
    ifstream contactIfs;
    contactIfs.open(((filesystem::path) poseSeqPath.parent_path() / fnamess.str()).string().c_str(), ios::out);

    BodyMotionPtr motion = bodyMotionItemPtr->motion();
    const double dt = ((double)1)/motion->frameRate();

    Vector3SeqPtr refCMSeqPtr = bodyMotionItemPtr->motion()->getOrCreateExtraSeq<Vector3Seq>("refCM");
    Vector3SeqPtr refPSeqPtr = bodyMotionItemPtr->motion()->getOrCreateExtraSeq<Vector3Seq>("refP");
    Vector3SeqPtr refLSeqPtr = bodyMotionItemPtr->motion()->getOrCreateExtraSeq<Vector3Seq>("refL");
    Vector3SeqPtr refZmpSeqPtr = bodyMotionItemPtr->motion()->getOrCreateExtraSeq<Vector3Seq>("ZMP");
    MultiValueSeqPtr refWrenchesSeqPtr = bodyMotionItemPtr->motion()->getOrCreateExtraSeq<MultiValueSeq>("wrenches");
    refWrenchesSeqPtr->setNumParts(6*4,true);

    std::vector<std::vector<string>> limbKeysVec{{"rleg"},{"lleg"},{"rarm"},{"larm"}};

    const int cycle = sfc->dt*bodyMotionItemPtr->motion()->frameRate();
    string PLStr, wrenchStr;
    getline(refPLIfs,PLStr);// first row is columnhead
    getline(wrenchIfs,wrenchStr);
    for(int i=0; getline(refPLIfs,PLStr),getline(wrenchIfs,wrenchStr); ++i){
        (BodyMotion::ConstFrame) motion->frame(i*cycle) >> *body;
        body->calcForwardKinematics();

        double Fz = 0;
        Vector3d zmp = Vector3d::Zero();
        std::map<string, VectorXd> wrenchMap;
        stringstream PLSs(PLStr), wrenchSs(wrenchStr);
        VectorXd wrench(6);
        Vector3d p,P,L,CM;
        double tmp;
        PLSs >> tmp;// time column
        wrenchSs >> tmp;
        for(int j=0; j<3; ++j) PLSs >> CM(j);
        for(int j=0; j<3; ++j) PLSs >> P(j);
        for(int j=0; j<3; ++j) PLSs >> L(j);
        for(auto contactLink : contactLinkCandidateSet){
            for(int j=0; j<6; ++j) wrenchSs >> wrench(j);
            p = body->link(contactLink->name())->p();
            // Vector3d f = body->link(contactLink->name())->R()*wrench.head(3);// wrench is local
            // Vector3d n = body->link(contactLink->name())->R()*wrench.tail(3);
            Vector3d f = body->link(contactLink->name())->R()*wrench.head(3);// world
            Vector3d n = body->link(contactLink->name())->R()*wrench.tail(3);

            Vector3 soleOffset = Vector3(0.05,0,0);//COP offset
            n += (body->link(contactLink->name())->R()*soleOffset).cross(f);

            wrench.head(3) << f;// world
            wrench.tail(3) << n;

            wrenchMap[contactLink->name()] = wrench;

            Fz += f.z();
            zmp.x() += -n.y()+p.x()*f.z();// 遊脚の場合は発散する
            zmp.y() +=  n.x()+p.y()*f.z();
        }
        refCMSeqPtr->at(i*cycle) = CM;
        refPSeqPtr->at(i*cycle) = P;
        refLSeqPtr->at(i*cycle) = L;
        zmp /= Fz;
        refZmpSeqPtr->at(i*cycle) = zmp;

        // refWrenchsSeq
        {
            MultiValueSeq::Frame wrenches = refWrenchesSeqPtr->frame(i*cycle);
            int idx = 0;
            for(auto limbKeys : limbKeysVec){
                VectorXd wrenchVec = VectorXd::Zero(6);
                for(auto wrench : wrenchMap){
                    bool isFound = true;
                    string linkName = wrench.first;
                    std::transform(linkName.begin(), linkName.end(), linkName.begin(), ::tolower);// convert to lower case
                    for(auto key : limbKeys){// check all keys
                        if(linkName.find(key) == std::string::npos){
                            isFound = false;
                            break;// break keys loop
                        }
                    }// isFound is true when all keys found
                    if(isFound){
                        wrenchVec = wrench.second;
                        break;// break wrenchMap loop
                    }
                }
                for(int j=0; j<6; ++j) wrenches[idx+j] = wrenchVec[j];
                idx += 6;
            }
            refWrenchesSeqPtr->frame(i*cycle) = wrenches;
        }
    }
    setSubItem("refCM", refCMSeqPtr, bodyMotionItemPtr);
    setSubItem("refP", refPSeqPtr, bodyMotionItemPtr);
    setSubItem("refL", refLSeqPtr, bodyMotionItemPtr);
    setSubItem("wrenches", refWrenchesSeqPtr, bodyMotionItemPtr);
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

    Vector3SeqPtr refCMSeqPtr = bodyMotionItemPtr->motion()->getOrCreateExtraSeq<Vector3Seq>("refCM");// 上書き確認
    Vector3SeqPtr refPSeqPtr = bodyMotionItemPtr->motion()->getOrCreateExtraSeq<Vector3Seq>("refP");
    Vector3SeqPtr refLSeqPtr = bodyMotionItemPtr->motion()->getOrCreateExtraSeq<Vector3Seq>("refL");
    Vector3SeqPtr refZmpSeqPtr = bodyMotionItemPtr->motion()->getOrCreateExtraSeq<Vector3Seq>("ZMP");
    MultiValueSeqPtr refWrenchesSeqPtr = bodyMotionItemPtr->motion()->getOrCreateExtraSeq<MultiValueSeq>("wrenches");
    refWrenchesSeqPtr->setNumParts(6*4,true);

    std::vector<std::vector<string>> limbKeysVec{{"rleg"},{"lleg"},{"rarm"},{"larm"}};

    fnamess.str("");
    fnamess << poseSeqPath.stem().string() << "_SFC_contact_" << sfc->dt << "dt_" << (int) 1/sfc->rootController()->dt << "fps.dat";
    ofstream contactOfs;
    contactOfs.open(((filesystem::path) poseSeqPath.parent_path() / fnamess.str()).string().c_str(), ios::out);
    // contactOfs << "time lpx lpy lpz lvx lvy lvz lwx lwy lwz rpx rpy rpz rvx rvy rvz rwx rwy rwz" << endl;
    contactOfs << "time";
    std::vector<string> contactStringVec{"px","py","pz", "vx","vy","vz", "wx","wy","wz"};
    for(auto link : contactLinkCandidateSet){
        string linkName = link->name();
        for(auto str :contactStringVec) contactOfs << " " << linkName << str;
    }
    contactOfs << endl;

    fnamess.str("");
    fnamess << poseSeqPath.stem().string() << "_SFC_wrench" << paramStr << "_" << (int) 1/sfc->rootController()->dt << "fps.dat";
    ofstream wrenchOfs;
    wrenchOfs.open(((filesystem::path) poseSeqPath.parent_path() / fnamess.str()).string().c_str(), ios::out);
    // wrenchOfs << "time lfx lfy lfz lnx lny lnz rfx rfy rfz rnx rny rnz" << endl;
    std::vector<string> wrenchStringVec{"fx","fy","fz", "nx","ny","nz"};
    wrenchOfs << "time";
    for(auto link : contactLinkCandidateSet){
        string linkName = link->name();
        for(auto str :wrenchStringVec) wrenchOfs << " " << linkName << str;
    }
    wrenchOfs << endl;

    fnamess.str("");
    fnamess << poseSeqPath.stem().string() << "_SFC_inputPL_" << sfc->dt << "dt_" << (int) 1/sfc->rootController()->dt << "fps.dat";
    ofstream inputPLOfs;
    inputPLOfs.open(((filesystem::path) poseSeqPath.parent_path() / fnamess.str()).string().c_str(), ios::out);
    inputPLOfs << "time inputCMx inputCMy inputCMz inputPx inputPy inputPz inputLx inputLy inputLz inputFx inputFy inputFz" << endl;

    for(int i=0; i < numFrames + sfc->numWindows(); ++i){
        // if(i > sfc->numWindows() + 1) goto BREAK;

        cout << endl << "##############################" << endl << "processCycle() turn:" << i  << "(" << (i-sfc->numWindows())*dt << " sec)"<< endl;
        float processedTime;
        if(sfc->processCycle(processedTime)) failIdxVec.push_back(i - sfc->numWindows());
        avgTime += processedTime;

        if(i >= sfc->numWindows()){
            VectorXd x0(sfc->stateDim);
            x0 = sfc->x0;

            int motionIdx = (i - sfc->numWindows())*cycle;
            Vector3d CM,P,L;
            CM << x0[0]/body->mass(),x0[2]/body->mass(),refCMSeqPtr->at(motionIdx).z(); // keep z coordinate
            P << x0[1],x0[3],refPSeqPtr->at(motionIdx).z(); // keep z coordinate
            L << x0[4],x0[5],x0[6];
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
            int numContacts = 0;
            Vector3d zmp = Vector3d::Zero();
            std::map<string, VectorXd> wrenchMap;
            for(std::set<Link*>::iterator linkIter = contactLinkCandidateSet.begin(); linkIter != contactLinkCandidateSet.end(); ++linkIter){
                int colIdx = 0;
                for(std::vector<ContactConstraintParam*>::iterator iter = ccParamVec.begin(); iter != ccParamVec.end(); ++iter){
                    ContactConstraintParam* ccParam = *iter;
                    int inputDim = ccParam->inputDim;
                    if((*linkIter)->name() == ccParam->linkName){
                        ++numContacts;
                        contactOfs << " "<< ccParam->p.transpose() << " " << ccParam->v.transpose() << " " << ccParam->w.transpose();
                        VectorXd u = ccParam->inputForceConvertMat*u0.segment(colIdx,inputDim);
                        wrenchOfs << " " << u.transpose();// local

                        //calc ZMP 足は床上である前提
                        Vector3d f = ccParam->R*u.head(3);
                        Vector3d n = ccParam->R*u.tail(3);
                        Fz += f.z();
                        zmp.x() += -n.y()+ccParam->p.x()*f.z();
                        zmp.y() +=  n.x()+ccParam->p.y()*f.z();
                        zmp.z() += ccParam->p.z();

                        Vector3 soleOffset = Vector3(0.05,0,0);// COP offset
                        n += (ccParam->R*soleOffset).cross(f);

                        //wrench
                        VectorXd wrench(6);
                        wrench.head(3) << f;// world
                        wrench.tail(3) << n;
                        // wrench.head(3) = u.head(3);// local
                        // wrench.tail(3) = u.tail(3);
                        wrenchMap[(*linkIter)->name()] = wrench;
                        goto END;
                    }
                    colIdx += inputDim;
                }
                contactOfs << " 0 0 0  0 0 0  0 0 0";
                wrenchOfs << " 0 0 0  0 0 0";
                wrenchMap[(*linkIter)->name()] = VectorXd::Zero(6);
            END:
                ;
            }
            zmp.segment(0,2)/= thresh(Fz,1e-5);
            zmp(2) /= numContacts;
            refZmpSeqPtr->at(motionIdx) = zmp;
            contactOfs << endl;
            wrenchOfs << endl;

            // refWrenchsSeq: limbKeysVec's limb order(for sequence file) is not same with contactLinkCandidateSet
            {
                cout << "wrenches in " << (i - sfc->numWindows())*dt << " sec :";
                MultiValueSeq::Frame wrenches = refWrenchesSeqPtr->frame(motionIdx);
                int idx = 0;
                for(auto limbKeys : limbKeysVec){
                    VectorXd wrenchVec = VectorXd::Zero(6);
                    for(auto wrench : wrenchMap){
                        bool isFound = true;
                        string linkName = wrench.first;
                        std::transform(linkName.begin(), linkName.end(), linkName.begin(), ::tolower);// convert to lower case
                        for(auto key : limbKeys){// check all keys
                            if(linkName.find(key) == std::string::npos){
                                isFound = false;
                                break;// break keys loop
                            }
                        }// isFound is true when all keys found
                        if(isFound){
                            wrenchVec = wrench.second;
                            cout << "  " << linkName << " " << wrench.second.transpose();
                            break;// break wrenchMap loop
                        }
                    }
                    for(int i=0; i<6; ++i) wrenches[idx+i] = wrenchVec[i];
                    idx += 6;
                }
                refWrenchesSeqPtr->frame(motionIdx) = wrenches;
                cout << endl;
            }
        }
    }
    {// for final frame's SeqPtr and log file
        int finalMotionIdx = numFrames*cycle;
        refCMSeqPtr->at(finalMotionIdx) = refCMSeqPtr->at(finalMotionIdx-cycle);
        refPSeqPtr->at(finalMotionIdx) = refPSeqPtr->at(finalMotionIdx-cycle);
        refLSeqPtr->at(finalMotionIdx) = refLSeqPtr->at(finalMotionIdx-cycle);
        refZmpSeqPtr->at(finalMotionIdx) = refZmpSeqPtr->at(finalMotionIdx-cycle);
        // refWrenchesSeqPtr->frame(finalMotionIdx) = refWrenchesSeqPtr->frame(finalMotionIdx-cycle); // cannot copy frame
        for(int j=0; j<refWrenchesSeqPtr->getNumParts(); ++j) refWrenchesSeqPtr->frame(finalMotionIdx)[j] = refWrenchesSeqPtr->frame(finalMotionIdx-cycle)[j];

        double finalTime = numFrames*dt;
        refPLOfs << finalTime << " " << refCMSeqPtr->at(finalMotionIdx).transpose() << " " << refPSeqPtr->at(finalMotionIdx).transpose() << " " << refLSeqPtr->at(finalMotionIdx).transpose() << " 0" << endl;
        const SlideFrictionControlParam* const finalMpcParam = sfc->mpcParamDeque[0];
        inputPLOfs << finalTime << " " << finalMpcParam->CM.transpose() << " " << finalMpcParam->P.transpose() << " " << finalMpcParam->L.transpose() << " " << finalMpcParam->F.transpose() << endl;
        // contactOfs, wrenchOfs
        contactOfs << finalTime; wrenchOfs << finalTime;
        std::vector<ContactConstraintParam*> ccParamVec = finalMpcParam->ccParamVec;
        for(std::set<Link*>::iterator linkIter = contactLinkCandidateSet.begin(); linkIter != contactLinkCandidateSet.end(); ++linkIter){
            int colIdx = 0;
            for(std::vector<ContactConstraintParam*>::iterator iter = ccParamVec.begin(); iter != ccParamVec.end(); ++iter){
                ContactConstraintParam* ccParam = *iter;
                int inputDim = ccParam->inputDim;
                if((*linkIter)->name() == ccParam->linkName){
                    contactOfs << " "<< ccParam->p.transpose() << " " << ccParam->v.transpose() << " " << ccParam->w.transpose();
                    VectorXd u = ccParam->inputForceConvertMat*(sfc->u0).segment(colIdx,inputDim);
                    wrenchOfs << " " << u.transpose();// local
                    goto END2;
                }
                colIdx += inputDim;
            }
            contactOfs << " 0 0 0  0 0 0  0 0 0";
            wrenchOfs << " 0 0 0  0 0 0";
        END2:
            ;
        }
        // contact log file is not used in loadExtraSeq
    }
 // BREAK:

    setSubItem("refCM", refCMSeqPtr, bodyMotionItemPtr);
    setSubItem("refP", refPSeqPtr, bodyMotionItemPtr);
    setSubItem("refL", refLSeqPtr, bodyMotionItemPtr);
    setSubItem("ZMP", refZmpSeqPtr, bodyMotionItemPtr);
    setSubItem("wrenches", refWrenchesSeqPtr, bodyMotionItemPtr);

    refPLOfs.close();
    contactOfs.close();
    wrenchOfs.close();
    inputPLOfs.close();

    for(std::vector<int>::iterator iter = failIdxVec.begin(); iter != failIdxVec.end(); ++iter) cout << "\x1b[31m" << "Failed in " << *iter << ":(" << (*iter)*dt << " sec)" << "\x1b[m" << endl;
    if(failIdxVec.empty()) cout << "All QP succeeded" << endl;
    failIdxVec.clear();

    cout << "total time: " << avgTime/1000.0 << "[sec]" << endl;
    cout << "average time: " << avgTime/numFrames << "[msec]" << endl;
}

namespace {
void generateContactConstraintParamVec(std::vector<ContactConstraintParam*>& ccParamVec, const SlideFrictionControl* const sfc, const std::set<Link*>& contactLinkCandidateSet, PoseSeq::iterator poseIter, const PoseSeqPtr& poseSeqPtr)
{
    cout << "\x1b[34m" << "generateContactConstraintParamVec( ~" << poseIter->time() << "[sec] )" << "\x1b[m" << endl;
    ccParamVec.clear();
    for(std::set<Link*>::iterator linkIter = contactLinkCandidateSet.begin(); linkIter != contactLinkCandidateSet.end(); ++linkIter){
        int linkIdx = (*linkIter)->index();
        int contactState = getPrevContactState(poseIter, poseSeqPtr, linkIdx);
        // if(contactState < 2){// 接触フラグが0か1 要改良
        if(true){
            std::vector<hrp::Vector2> vertexVec;
            hrp::Vector2 vertex;// 頂点の2次元座標を代入 (要 足の形状取得)
            // vertex << 0.1,0.05; vertexVec.push_back(vertex);
            // vertex << 0.1,-0.05; vertexVec.push_back(vertex);
            // vertex << -0.1,0.05; vertexVec.push_back(vertex);
            // vertex << -0.1,-0.05; vertexVec.push_back(vertex);
            // vertex << 0.15,0.08; vertexVec.push_back(vertex);
            // vertex << 0.15,-0.08; vertexVec.push_back(vertex);
            // vertex << -0.1,0.08; vertexVec.push_back(vertex);
            // vertex << -0.1,-0.08; vertexVec.push_back(vertex);
            // vertex << 0.13,0.065; vertexVec.push_back(vertex);
            // vertex << 0.13,-0.065; vertexVec.push_back(vertex);
            // vertex << -0.1,0.065; vertexVec.push_back(vertex);
            // vertex << -0.1,-0.065; vertexVec.push_back(vertex);
            // vertex << 0.12,0.06; vertexVec.push_back(vertex);
            // vertex << 0.12,-0.06; vertexVec.push_back(vertex);
            // vertex << -0.09,0.06; vertexVec.push_back(vertex);
            // vertex << -0.09,-0.06; vertexVec.push_back(vertex);

            // for JAXON_BLUE
            // vertex << 0.16,0.05; vertexVec.push_back(vertex);
            // vertex << 0.16,-0.05; vertexVec.push_back(vertex);
            // vertex << -0.07,0.05; vertexVec.push_back(vertex);
            // vertex << -0.07,-0.05; vertexVec.push_back(vertex);
            // side mergine
            hrp::Vector2 soleOffset = Vector2(0.05,0);
            // vertex << 0.16,0.03; vertex -= soleOffset; vertexVec.push_back(vertex);
            // vertex << 0.16,-0.03; vertex -= soleOffset; vertexVec.push_back(vertex);
            // vertex << -0.07,0.03; vertex -= soleOffset; vertexVec.push_back(vertex);
            // vertex << -0.07,-0.03; vertex -= soleOffset; vertexVec.push_back(vertex);
            vertex << 0.14,0.03; vertex -= soleOffset; vertexVec.push_back(vertex);
            vertex << 0.14,-0.03; vertex -= soleOffset; vertexVec.push_back(vertex);
            vertex << -0.05,0.03; vertex -= soleOffset; vertexVec.push_back(vertex);
            vertex << -0.05,-0.03; vertex -= soleOffset; vertexVec.push_back(vertex);

            std::vector<hrp::Vector2> smallVertexVec;
            vertex << 0.08,0.03; smallVertexVec.push_back(vertex);
            vertex << 0.08,-0.03; smallVertexVec.push_back(vertex);
            vertex << -0.05,0.03; smallVertexVec.push_back(vertex);
            vertex << -0.05,-0.03; smallVertexVec.push_back(vertex);

            if(contactState == 0){// static contact
                // ccParamVec.push_back(new SimpleContactConstraintParam((*linkIter)->name(), vertexVec));
                // ccParamVec.push_back(new StaticContactConstraintParam((*linkIter)->name(), vertexVec, 0.5));// 摩擦係数 要改良
                ccParamVec.push_back(new StaticContactConstraintParam((*linkIter)->name(), vertexVec, 0.6));// 摩擦係数 要改良
                // ccParamVec.push_back(new StaticContactConstraintParam((*linkIter)->name(), smallVertexVec, 0.6));// 摩擦係数 要改良
                // ccParamVec.push_back(new DistributedForceContactConstraintParam((*linkIter)->name(), vertexVec, 2,2));
                // ccParamVec.push_back(new DistributedForceStaticContactConstraintParam((*linkIter)->name(), vertexVec, sfc->numXDivisions,sfc->numYDivisions, 0.5));// 摩擦係数 要改良
                // ccParamVec.push_back(new DistributedForceStaticContactConstraintParam((*linkIter)->name(), vertexVec, 1,1, 0.5));// 摩擦係数 要改良
                // ccParamVec.push_back(new DistributedForceStaticContactConstraintParam((*linkIter)->name(), vertexVec, 2,2, 0.5));// 摩擦係数 要改良
                // ccParamVec.push_back(new DistributedForceStaticContactConstraintParam((*linkIter)->name(), vertexVec, 3,3, 0.5));// 3x3y 摩擦係数 要改良
                // ccParamVec.push_back(new DistributedForceStaticContactConstraintParam((*linkIter)->name(), vertexVec, 4,2, 0.5));// 4x2y 摩擦係数 要改良
                cout << " " << (*linkIter)->name() << ": Static" << endl;
            }else if(contactState == 1){// slide contact
                // ccParamVec.push_back(new SlideContactConstraintParam((*linkIter)->name(), vertexVec, 0.5, getPrevDirection(poseIter, poseSeqPtr, linkIdx)));
                ccParamVec.push_back(new DistributedForceSlideContactConstraintParam((*linkIter)->name(), vertexVec, sfc->numXDivisions,sfc->numYDivisions, 0.6));
                // ccParamVec.push_back(new DistributedForceSlideContactConstraintParam((*linkIter)->name(), vertexVec, sfc->numXDivisions,sfc->numYDivisions, 0.5));
                // ccParamVec.push_back(new DistributedForceSlideContactConstraintParam((*linkIter)->name(), vertexVec, 1,1, 0.5));
                // ccParamVec.push_back(new DistributedForceSlideContactConstraintParam((*linkIter)->name(), vertexVec, 2,2, 0.5));
                // ccParamVec.push_back(new DistributedForceSlideContactConstraintParam((*linkIter)->name(), vertexVec, 3,3, 0.5));//3x3y
                // ccParamVec.push_back(new DistributedForceSlideContactConstraintParam((*linkIter)->name(), vertexVec, 4,2, 0.5));//4x2y
                cout << " " << (*linkIter)->name() << ": Slide" << endl;
            }else if(contactState == 3){// float
                // ccParamVec.push_back(new FloatConstraintParam((*linkIter)->name(), vertexVec));
                cout << " " << (*linkIter)->name() << ": Float" << endl;
            }else{
                cout << " " << "\x1b[31m" << (*linkIter)->name() << " is static float and this is not supported" << "\x1b[m" << endl;
            }
        }
    }
    cout << endl << endl;
}

void generateSlideFrictionControlParam(SlideFrictionControlParam* sfcParam, Vector3d CM, Vector3d P, Vector3d L, Vector3d F, BodyPtr& body, std::vector<ContactConstraintParam*>& ccParamVec, std::vector<ContactConstraintParam*>& prevCcParamVec, double dt)
{

    static Vector3 g;
    g << 0,0,9.8;

    // CM,P,L
    // Vector3d CM,P,L,F;
    // Vector3d L;
    // CM = body->calcCenterOfMass();
    // body->calcTotalMomentum(P,L);
    // P << 0,0,0;// overwrite P impossible in jumping motion
    // L -= CM.cross(P);// convert to around CoM
    // sfcParam->CM = CM;
    // sfcParam->CM << 0.065,0,0;// move root link foward 65mm for JAXON
    // sfcParam->CM = CM + body->rootLink()->R()*Vector3(0.05,0,0);
    // sfcParam->CM = CM + body->rootLink()->R()*Vector3(0.065,0,0);// for rotate-left-slide
    sfcParam->CM = CM + body->rootLink()->R()*Vector3(0,0,0);
    // sfcParam->CM += CM;
    sfcParam->P = P;
    // sfcParam->P << 0,0,P.z();// overwrite only xy coordinates
    sfcParam->L = L;
    // sfcParam->L << 0,0,0;
    // sfcParam->F = body->mass()*g;
    sfcParam->F = F;

    // 接触点座標系の更新 等式と不等式数の合計
    for(std::vector<ContactConstraintParam*>::iterator iter = ccParamVec.begin(); iter != ccParamVec.end(); ++iter){
        bool isPrevstatic = false;
        if(prevCcParamVec.size() != 0){
            for(auto ccParam : prevCcParamVec){
                if((*iter)->linkName == ccParam->linkName){
                    if(typeid(*ccParam) == typeid(SimpleContactConstraintParam)
                       || typeid(*ccParam) == typeid(StaticContactConstraintParam)
                       || typeid(*ccParam) == typeid(DistributedForceContactConstraintParam)
                       || typeid(*ccParam) == typeid(DistributedForceStaticContactConstraintParam)){
                        isPrevstatic = true;
                        break;
                    }
                }
            }
        }
        // (*iter)->p = body->link((*iter)->linkName)->p();
        Vector3 soleOffset = Vector3(0.05,0,-0.1);
        (*iter)->p = body->link((*iter)->linkName)->p()+body->link((*iter)->linkName)->R()*soleOffset;
        (*iter)->R = body->link((*iter)->linkName)->R();
        // (*iter)->v = body->link((*iter)->linkName)->v();
        // (*iter)->w = body->link((*iter)->linkName)->w();
        // (*iter)->v << 0,body->link((*iter)->linkName)->v().y(),0;// overwrite x,z->0
        (*iter)->v << body->link((*iter)->linkName)->v().x(),body->link((*iter)->linkName)->v().y(),0;// overwrite z->0
        (*iter)->w << 0,0,body->link((*iter)->linkName)->w().z();// overwrite r,p->0
        // (*iter)->w << 0,0,0;// overwrite r,p,y->0
        (*iter)->v += (*iter)->w.cross(soleOffset);

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
            DistributedForceSlideContactConstraintParam* ccParam = new DistributedForceSlideContactConstraintParam(dynamic_cast<DistributedForceSlideContactConstraintParam*>(*iter));
            if(isPrevstatic){
                cout << "slideDF -> staticDF" << endl;
                DistributedForceStaticContactConstraintParam* staticCcParam
                    = new DistributedForceStaticContactConstraintParam(ccParam->linkName, ccParam->vertexVec, ccParam->rows(),ccParam->cols(), ccParam->muTrans);
                staticCcParam->p = (*iter)->p;
                staticCcParam->R = (*iter)->R;
                staticCcParam->v = (*iter)->v;
                staticCcParam->w = (*iter)->w;
                sfcParam->ccParamVec.push_back(staticCcParam);
            }else{
                sfcParam->ccParamVec.push_back(ccParam);
            }
        }else if(typeid(**iter) == typeid(FloatConstraintParam)){
            sfcParam->ccParamVec.push_back(new FloatConstraintParam(dynamic_cast<FloatConstraintParam*>(*iter)));
        }else{
            cout << "\x1b[31m" << "Warning! ContactConstraintParam's pointer Copy Constructor called" << "\x1b[m" << endl;
            sfcParam->ccParamVec.push_back(new ContactConstraintParam(dynamic_cast<ContactConstraintParam*>(*iter)));
        }
    }

}

}

void cnoid::generateVerticalTrajectory(BodyPtr& body, const PoseSeqItemPtr& poseSeqItem, const std::set<Link*> contactLinkCandidateSet, const std::vector<double>& takeoffPhaseRatioVec, const std::vector<double>& landingPhaseRatioVec)
{
    cout << "generateVerticalTrajectory()" << endl;

    PoseSeqPtr poseSeq = poseSeqItem->poseSeq();
    BodyMotionItemPtr bodyMotionItem = poseSeqItem->bodyMotionItem();
    const int frameRate = bodyMotionItem->motion()->frameRate();
    const int numFrames = bodyMotionItem->motion()->numFrames();
    const double dt = 1.0/frameRate;
    const double m = body->mass();
    const double g = 9.80665;

    // initial trajectories
    Vector3SeqPtr initCMSeqPtr = bodyMotionItem->findSubItem<Vector3SeqItem>("initCM")->seq();
    Vector3SeqPtr initPSeqPtr = bodyMotionItem->findSubItem<Vector3SeqItem>("initP")->seq();
    Vector3SeqPtr initLSeqPtr = bodyMotionItem->findSubItem<Vector3SeqItem>("initL")->seq();

    Vector3SeqPtr refCMSeqPtr = bodyMotionItem->motion()->getOrCreateExtraSeq<Vector3Seq>("refCM");
    Vector3SeqPtr refPSeqPtr = bodyMotionItem->motion()->getOrCreateExtraSeq<Vector3Seq>("refP");
    Vector3SeqPtr refLSeqPtr = bodyMotionItem->motion()->getOrCreateExtraSeq<Vector3Seq>("refL");

    Vector3d takeoffdCM, landingdCM;
    double jumpTime;
    for(PoseSeq::iterator frontPoseIter = (++poseSeq->begin()),backPoseIter = poseSeq->begin(); frontPoseIter != poseSeq->end(); incContactPose(frontPoseIter,poseSeq,body)){
        if(!isContactStateChanging(frontPoseIter, poseSeq, body)) continue;

        bool isTakeoff = true, isJumping = true, isLanding = true;
        for(auto linkIter : contactLinkCandidateSet){
            int linkIdx = linkIter->index();
            // 静止遊脚はない前提
            // 0:静止接触 1:滑り接触 (2:静止遊脚) 3:遊脚
            if(getNextContactState(frontPoseIter, poseSeq, linkIdx) != 3) isTakeoff &= false;
            if(getPrevContactState(frontPoseIter, poseSeq, linkIdx) != 3) isJumping &= false;
            if(getPrevContactState(backPoseIter,  poseSeq, linkIdx) != 3) isLanding &= false;
        }

        int jumpFrameOffset = 1;
        PoseSeq::iterator takeoffIter,landingIter;
        // CubicSplineInterpolator interpolator = CubicSplineInterpolator();// spline
        AccelerationInterpolator interpolator = AccelerationInterpolator();// acc
        if(isTakeoff || isLanding){// takeoff and landing phases
            cout << " \x1b[34m" << backPoseIter->time() << "[sec] -> " << frontPoseIter->time() << "[sec]: include takeoff or landing phase\x1b[m" << endl;

            PoseSeq::iterator startPoseIter, endPoseIter;
            double startTime, endTime;
            int startFrame, endFrame;
            Vector3d startCM, endCM, startP, endP; // use simple model momentum calculated by UtilPlugin
            if(isTakeoff){// takeoff phase is necessary and must be first in if branching for setting takeoffdCM
                takeoffIter = frontPoseIter;
                landingIter = frontPoseIter; incContactPose(landingIter,poseSeq,body);
                jumpTime = landingIter->time() - takeoffIter->time() + jumpFrameOffset*dt;// add jumpFrameOffset
                Vector3d takeoffCM = initCMSeqPtr->at(takeoffIter->time()*frameRate), landingCM = initCMSeqPtr->at(landingIter->time()*frameRate + jumpFrameOffset);// add jumpFrameOffset
                takeoffdCM = (landingCM - takeoffCM) / jumpTime;
                landingdCM = takeoffdCM;
                takeoffdCM.z() = 0.5 * g * jumpTime + (landingCM.z() - takeoffCM.z()) / jumpTime;
                landingdCM.z() = takeoffdCM.z() - g * jumpTime;

                startPoseIter = frontPoseIter; --startPoseIter;
                endPoseIter = frontPoseIter;
                startFrame = startPoseIter->time()*frameRate; endFrame = endPoseIter->time()*frameRate;
                startTime = startFrame*dt; endTime = endFrame*dt;
                startCM = initCMSeqPtr->at(startFrame); endCM = initCMSeqPtr->at(endFrame);
                startP = initPSeqPtr->at(startFrame);   endP = initPSeqPtr->at(endFrame); // use simple model momentum calculated by UtilPlugin

                // interpolator.calcCoefficients(startCM,startP/m, endCM,takeoffdCM, endTime - startTime);// spline
                interpolator.calcCoefficients(startCM,startP/m,Vector3d::Zero(), endCM,takeoffdCM,Vector3(0,0,-g), endTime - startTime, 0,takeoffPhaseRatioVec);// acc
                cout << " \x1b[34m" << startTime << "[sec](" << startFrame << ") -> " << endTime << "[sec](" << endFrame << "): takeoff phase\x1b[m" << endl;
                cout << "  startCM: " << startCM.transpose() << endl << "  startdCM:   " << startP.transpose()/m << endl;
                cout << "  endCM:   " << endCM.transpose()   << endl << "  takeoffdCM: " << takeoffdCM.transpose() << endl;
                cout << "  jumpTime: " << jumpTime << " [sec]" << endl;
            }else{// landing phase
                int delayOffset = 2;// forward-difference delay + median-filter delay = 1 + 1 = 2
                startPoseIter = backPoseIter;
                endPoseIter = backPoseIter; ++endPoseIter;
                startFrame = startPoseIter->time()*frameRate + jumpFrameOffset; endFrame = min((int)(endPoseIter->time()*frameRate) + delayOffset, numFrames-1);// add jumpFrameOffset for fz differential or delay
                startTime = startFrame*dt; endTime = endFrame*dt;// add delay

                startCM = initCMSeqPtr->at(startFrame); endCM = initCMSeqPtr->at(endFrame);
                startP = initPSeqPtr->at(startFrame);   endP = initPSeqPtr->at(endFrame); // use simple model momentum calculated by UtilPlugin

                // interpolator.calcCoefficients(startCM,landingdCM, endCM,endP/m, endTime - startTime);// spline
                interpolator.calcCoefficients(startCM,landingdCM,Vector3(0,0,-g), endCM,endP/m,Vector3d::Zero(), endTime - startTime, 0,landingPhaseRatioVec);// acc
                cout << " \x1b[34m" << startTime << "[sec](" << startFrame << ") -> " << endTime << "[sec](" << endFrame << "): landing phase\x1b[m" << endl;
                cout << "  startCM: " << startCM.transpose() << endl << "  landingdCM: " << landingdCM.transpose() << endl;
                cout << "  endCM:   " << endCM.transpose()   << endl << "  enddCM:     " << endP.transpose()/m << endl;
            }

            int copyStartFrame = backPoseIter->time()*frameRate, copyEndFrame = frontPoseIter->time()*frameRate;
            if(isLanding) copyStartFrame += jumpFrameOffset;
            // for(int i=backPoseIter->time()*frameRate; i < frontPoseIter->time()*frameRate; ++i){
            cout << "  set init value: " << copyStartFrame << " -> " << copyEndFrame << endl;
            for(int i=copyStartFrame; i < copyEndFrame; ++i){
                refCMSeqPtr->at(i) = initCMSeqPtr->at(i);
                refPSeqPtr->at(i) = initPSeqPtr->at(i);
                // refLSeqPtr->at(i) = Vector3d(0,0,0);
                refLSeqPtr->at(i) = initLSeqPtr->at(i);
            }
            // for(int i=startPoseIter->time()*frameRate; i < endPoseIter->time()*frameRate; ++i){
            // for(int i=startPoseIter->time()*frameRate; i < endFrame; ++i){ // use endFrame including delay
            for(int i=startFrame; i < endFrame; ++i){ // use startFrame and endFrame including delay
                Vector3d CM, P;
                // cubic-spline
                CM = interpolator.x(i*dt - startTime); // overwrite
                P = m*interpolator.dx(i*dt - startTime); // overwrite

                // // minjerk
                // CM.z() = (a[0] + a[1]*dT + a[2]*dT2 + a[3]*dT3 + a[4]*dT4 + a[5]*dT5).z(); // only overwrite z coordinate
                // P.z() = m*(a[1] + 2*a[2]*dT + 3*a[3]*dT2 + 4*a[4]*dT3 + 5*a[5]*dT4).z();

                refCMSeqPtr->at(i) = CM;
                refPSeqPtr->at(i) = P;
                // refLSeqPtr->at(i) = Vector3d(0,0,0);
                refLSeqPtr->at(i) = initLSeqPtr->at(i);
            }
        }else if(isJumping){// jumping phases
            // cout << " \x1b[34m" << backPoseIter->time() << "[sec] -> " << frontPoseIter->time() << "[sec]: jumping phase\x1b[m" << endl;
            int startFrame = backPoseIter->time()*frameRate, endFrame = frontPoseIter->time()*frameRate + jumpFrameOffset;// add  jumpFrameOffset for fz differential
            double startTime = startFrame*dt, endTime = endFrame*dt;// add delay
            Vector3d startCM = initCMSeqPtr->at(startFrame), endCM = initCMSeqPtr->at(endFrame);
            Vector3d startP = initPSeqPtr->at(startFrame),   endP = initPSeqPtr->at(endFrame); // use simple model momentum calculated by UtilPlugin
            double jumptime = endTime - startTime;
            cout << " \x1b[34m" << startTime << "[sec](" << startFrame << ") -> " << endTime << "[sec](" << endFrame << "): jumping phase\x1b[m" << endl;
            // for(int i=backPoseIter->time()*frameRate; i < frontPoseIter->time()*frameRate; ++i){
            for(int i=startFrame; i < endFrame; ++i){
                double t = i*dt;
                Vector3d CM = initCMSeqPtr->at(i), P = initPSeqPtr->at(i);
                refCMSeqPtr->at(i) = startCM + (endCM - startCM) * (t - startTime) /jumptime; // xy
                refCMSeqPtr->at(i).z() = - 0.5 * g * (t - startTime) * (t - endTime)
                    + (endCM.z()* (t - startTime) - startCM.z()* (t - endTime)) / jumptime; //z
                refPSeqPtr->at(i) = m*takeoffdCM; // xy
                refPSeqPtr->at(i).z() = - m * g * (t - startTime) + m*takeoffdCM.z(); // z
                // refLSeqPtr->at(i) = Vector3d(0,0,0);
                refLSeqPtr->at(i) = initLSeqPtr->at(i);
            }
        }else{// other phases
            cout << " \x1b[34m" << backPoseIter->time() << "[sec] -> " << frontPoseIter->time() << "[sec]: normal phase\x1b[m" << endl;
            for(int i=backPoseIter->time()*frameRate; i < frontPoseIter->time()*frameRate; ++i){
                refCMSeqPtr->at(i) = initCMSeqPtr->at(i);
                refPSeqPtr->at(i) = initPSeqPtr->at(i);
                // refLSeqPtr->at(i) = Vector3d(0,0,0);
                refLSeqPtr->at(i) = initLSeqPtr->at(i);
            }
        }

        backPoseIter = frontPoseIter;
    }
    refCMSeqPtr->at(numFrames-1) = refCMSeqPtr->at(numFrames-2);// final frame
    refPSeqPtr->at(numFrames-1) = refPSeqPtr->at(numFrames-2);
    refLSeqPtr->at(numFrames-1) = refLSeqPtr->at(numFrames-2);

    {
        boost::filesystem::path poseSeqPath = boost::filesystem::path(poseSeqItem->filePath());
        stringstream fnamess; fnamess.str("");
        fnamess << poseSeqPath.stem().string() << "_SFC_prePL" << "_" << frameRate << "fps.dat";
        ofstream ofs;
        ofs.open(((filesystem::path) poseSeqPath.parent_path() / fnamess.str()).string().c_str(), ios::out);
        ofs << "time preCMx preCMy preCMz prePx prePy prePz preLx preLy preLz processTime" << endl;
        for(int i=0; i<numFrames; ++i){
            ofs << i*dt << " " << refCMSeqPtr->at(i).transpose() << " " << refPSeqPtr->at(i).transpose() << " " << refLSeqPtr->at(i).transpose() << endl;
        }
        ofs.close();
    }

    setSubItem("refCM", refCMSeqPtr, bodyMotionItem);
    setSubItem("refP", refPSeqPtr, bodyMotionItem);
    setSubItem("refL", refLSeqPtr, bodyMotionItem);

}

void cnoid::generatePreModelPredictiveControlParamDeque(SlideFrictionControl* sfc, BodyPtr body, const PoseSeqItemPtr& poseSeqItem, const std::set<Link*>& contactLinkCandidateSet)
{
    cout << "generatePreModelPredictiveControlParamDeque()" << endl;

    const PoseSeqPtr poseSeqPtr = poseSeqItem->poseSeq();
    const BodyMotionItemPtr motionItem = poseSeqItem->bodyMotionItem();
    const BodyMotionPtr motion = motionItem->motion();
    const int frameRate = motion->frameRate();
    const int numFrames = motion->numFrames();
    const double dt = 1.0/motion->frameRate();
    const Vector3d gVec = Vector3(0,0,9.80665);

    updateBodyState(body, motion, 0, contactLinkCandidateSet);
    body->calcForwardKinematics();// use end effector's velocity

    Vector3SeqPtr refCMSeqPtr = motionItem->findSubItem<Vector3SeqItem>("refCM")->seq();
    Vector3SeqPtr refPSeqPtr = motionItem->findSubItem<Vector3SeqItem>("refP")->seq();
    Vector3SeqPtr refLSeqPtr = motionItem->findSubItem<Vector3SeqItem>("refL")->seq();

    // cnoidのクラス(BodyMotion)からmpcParamDequeを生成
    int index = 0;
    std::vector<ContactConstraintParam*> prevCcParamVec;
    ::generateContactConstraintParamVec(prevCcParamVec, sfc, contactLinkCandidateSet, ++poseSeqPtr->begin(), poseSeqPtr);
    for(PoseSeq::iterator frontPoseIter = (++poseSeqPtr->begin()),backPoseIter = poseSeqPtr->begin(); frontPoseIter != poseSeqPtr->end(); incContactPose(frontPoseIter,poseSeqPtr,body)){
        if(!isContactStateChanging(frontPoseIter, poseSeqPtr, body)) continue;

        // 接触状態依存のパラメータのみ設定(動作軌道に依存するパラメータは後で設定)
        std::vector<ContactConstraintParam*> ccParamVec;
        ::generateContactConstraintParamVec(ccParamVec, sfc, contactLinkCandidateSet, frontPoseIter, poseSeqPtr);

        for(int i=backPoseIter->time()*frameRate; i < frontPoseIter->time()*frameRate; ++i){
        // for(int i=backPoseIter->time()*frameRate+1; i <= frontPoseIter->time()*frameRate; ++i){
            updateBodyState(body, motion, min(i,numFrames-1), contactLinkCandidateSet);
            body->calcForwardKinematics();// use end effector's velocity

            SlideFrictionControlParam* sfcParam = new SlideFrictionControlParam(index, sfc);
            Vector3d P = refPSeqPtr->at(i);
            Vector3d L = refLSeqPtr->at(i);
            // 動作軌道に依存するパラメータの設定
            if(i == backPoseIter->time()*frameRate){ // 接触状態の変わり目だけ別処理
                // ::generateSlideFrictionControlParam(sfcParam, refCMSeqPtr->at(i), P, thresh((P - refPSeqPtr->at(max(i-1,0)))/dt+body->mass()*gVec, 0.0001, Vector3d::Zero(3)), body, ccParamVec, prevCcParamVec, dt);
                // forward-difference for jump
                ::generateSlideFrictionControlParam(sfcParam, refCMSeqPtr->at(i), P, L, thresh((refPSeqPtr->at(min(i+1,numFrames-1))-P)/dt+body->mass()*gVec, 0.0001, Vector3d::Zero(3)), body, ccParamVec, prevCcParamVec, dt);
                // ::generateSlideFrictionControlParam(sfcParam, refCMSeqPtr->at(i), P, (P - refPSeqPtr->at(max(i-1,0)))/dt+body->mass()*gVec, body, ccParamVec, prevCcParamVec, dt);
            }else{
                std::vector<ContactConstraintParam*> dummyCcparamVec;
                // ::generateSlideFrictionControlParam(sfcParam, refCMSeqPtr->at(i), P, thresh((P - refPSeqPtr->at(max(i-1,0)))/dt+body->mass()*gVec, 0.0001, Vector3d::Zero(3)), body, ccParamVec, dummyCcparamVec, dt);
                // forward-difference for jump
                ::generateSlideFrictionControlParam(sfcParam, refCMSeqPtr->at(i), P, L, thresh((refPSeqPtr->at(min(i+1,numFrames-1))-P)/dt+body->mass()*gVec, 0.0001, Vector3d::Zero(3)), body, ccParamVec, dummyCcparamVec, dt);
                // ::generateSlideFrictionControlParam(sfcParam, refCMSeqPtr->at(i), P, (P - refPSeqPtr->at(max(i-1,0)))/dt+body->mass()*gVec, body, ccParamVec, dummyCcparamVec, dt);
            }

            sfc->preMpcParamDeque.push_back((SlideFrictionControlParam*) sfcParam);
            ++index;
        }

        prevCcParamVec = ccParamVec;
        backPoseIter = frontPoseIter;
    }
}

void SlideFrictionControlPlugin::execControl()
{
    stringstream ss,fnamess;
    MessageView::instance()->putln("SlideFrictionControl called !");
    cout << "\x1b[31m" << "SlideFrictionControl()" << "\x1b[m" << endl;

    BodyItemPtr bodyItemPtr;
    PoseSeqItemPtr poseSeqItemPtr;
    PoseSeqPtr poseSeqPtr;
    if(!getSelectedPoseSeqSet(bodyItemPtr, body, poseSeqItemPtr, poseSeqPtr, mBodyMotionItemPtr, motion)) return;

    mPoseSeqPath = boost::filesystem::path(poseSeqItemPtr->filePath());
    cout << "PoseSeqPath: " << mPoseSeqPath << endl;

    std::vector<Link*> endEffectorLinkVec;
    if(!getEndEffectorLinkVector(endEffectorLinkVec, body)) return;

    generateOptionalData(body, poseSeqItemPtr, endEffectorLinkVec);

    // BodyMotion作成
    generateBodyMotionFromBar(body, poseSeqItemPtr, mBodyMotionItemPtr);

    frameRate = motion->frameRate();
    dt = 1.0/frameRate;
    numFrames = motion->numFrames();

    generateInitSeq(body, poseSeqItemPtr, endEffectorLinkVec);

    // 接触候補セットの作成
    std::set<Link*> contactLinkCandidateSet;
    calcContactLinkCandidateSet(contactLinkCandidateSet, body, poseSeqPtr);

    SlideFrictionControlSetupLayout* layout = mBar->dialog->layout();

    // 垂直方向の軌道生成
    generateVerticalTrajectory(body, poseSeqItemPtr, contactLinkCandidateSet, layout->takeoffPhaseRatioSpinArray->value(),layout->landingPhaseRatioSpinArray->value());

    sfc = new SlideFrictionControl();
    sfc->m = body->mass();
    sfc->dt = dt;
    sfc->setBlockVector(layout->blockSpinVec->value());
    sfc->errorCMWeight = layout->errorCMWeightSpin->value();
    sfc->errorMomentumWeight = layout->errorMomentumWeightSpin->value();
    sfc->errorAngularMomentumWeight = layout->errorAngularMomentumWeightSpin->value();
    sfc->errorYawAngularMomentumWeight = layout->errorYawAngularMomentumWeightSpin->value();
    sfc->inputForceWeight = layout->inputForceWeightSpin->value();
    sfc->inputMomentWeight = layout->inputMomentWeightSpin->value();
    sfc->inputYawMomentWeight = layout->inputYawMomentWeightSpin->value();
    sfc->numXDivisions = layout->xDivisionNumSpin->value();
    sfc->numYDivisions = layout->yDivisionNumSpin->value();

    generatePreModelPredictiveControlParamDeque(sfc, body, poseSeqItemPtr, contactLinkCandidateSet);

    sweepControl(mPoseSeqPath, mBar->dialog->layout()->getParamString(), sfc, body, mBodyMotionItemPtr, contactLinkCandidateSet);

    cout << "\x1b[31m" << "Finished SlideFrictionControl" << "\x1b[m" << endl;
}

CNOID_IMPLEMENT_PLUGIN_ENTRY(SlideFrictionControlPlugin)
