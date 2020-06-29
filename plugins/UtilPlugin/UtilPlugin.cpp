/**
   @author Kunio Kojima
*/

#include "UtilPlugin.h"

using namespace boost;
using namespace cnoid;
using namespace std;

boost::filesystem::path cnoid::getLogPath(const boost::filesystem::path& poseSeqPath)
{
    return poseSeqPath.parent_path() / "log" / poseSeqPath.stem();
}

// 特定のリンクの次のポーズを求める
PoseSeq::iterator cnoid::getNextPose(PoseSeq::iterator poseIter, PoseSeqPtr poseSeq, int linkId)
{
    // std::cout << "getNextPose(" << poseIter->time() << " s linkId:" << linkId << ")";
    PoseSeq::iterator iter;
    for(iter = (++poseIter); iter != poseSeq->end(); ++iter){
        // std::cout << " " << iter->time() << "sec";
        Pose::LinkInfo* linkInfo = iter->get<Pose>()->ikLinkInfo(linkId);
        if(linkInfo){
            // std::cout << " link" << linkId << "'s next pose time:" << iter->time() << std::endl;
            return iter;
        }

    }
    --iter;
    // std::cout << " link" << linkId << "'s contact pose not found. return pose time:" << iter->time() << std::endl;
    return iter;
}

// 特定リンクの前のポーズを求める
PoseSeq::iterator cnoid::getPrevPose(PoseSeq::iterator poseIter, PoseSeqPtr poseSeq, int linkId)
{
    // std::cout << "getPrevPose(" << poseIter->time() << " s linkId:" << linkId << ")";
    PoseSeq::iterator iter;
    for(iter = (--poseIter); iter != (--poseSeq->begin()); --iter){
        // std::cout << " " << iter->time() << "sec";
        Pose::LinkInfo* linkInfo = iter->get<Pose>()->ikLinkInfo(linkId);
        if(linkInfo){
            // std::cout << " link" << linkId << "'s prev pose time:" << iter->time() << std::endl;
            return iter;
        }

    }
    ++iter;
    // std::cout << " link" << linkId << "'s contact pose not found. return pose time:" << iter->time() << std::endl;
    return iter;
}

namespace {
void getContactPoints(std::vector<Vector3d>& contactPointVec, const PosePtr pose1, const PosePtr pose2, const int linkIdx)
{
    std::vector<Vector3d> contactPointVec1, contactPointVec2;
    reduceConvexHullToQuadrangle(contactPointVec1, pose1->ikLinkInfo(linkIdx)->contactPoints());
    reduceConvexHullToQuadrangle(contactPointVec2, pose2->ikLinkInfo(linkIdx)->contactPoints());

    // filter near points between current pose and previous pose
    auto nearPointFilterFunc = [](const std::vector<Vector3d>& operandPointVec, const std::vector<Vector3d>& filterPointVec, std::vector<Vector3d>& resultPointVec){
        std::copy_if(operandPointVec.begin(), operandPointVec.end(), std::back_inserter(resultPointVec),
                     [&](Vector3d operandPoint){
                         return filterPointVec.end() != std::find_if(filterPointVec.begin(), filterPointVec.end(),
                                                                     [&](Vector3d filterPoint){
                                                                         return (operandPoint - filterPoint).norm() < 0.01;
                                                                     });
                     });
    };
    std::vector<Vector3d> tmpContactPointVec;
    nearPointFilterFunc(contactPointVec1, contactPointVec2, tmpContactPointVec);
    nearPointFilterFunc(contactPointVec2, contactPointVec1, tmpContactPointVec);

    // reduce the number of points to 4
    reduceConvexHullToQuadrangle(contactPointVec, tmpContactPointVec);
    if(contactPointVec.size() != 4 && contactPointVec.size() != 0) std::cout << "\x1b[31m" << "!!! The number of Term's contact points is not 0 or 4 !!!" << "\x1b[m" << std::endl;

    std::cout << " contact vertices: "; for(auto point : contactPointVec) cout << " [" << point.transpose() << "]"; cout << std::endl;
}

// 2つのPose間の接触状態を2進数で表す
// 0:静止接触 1:滑り接触 (2:静止遊脚) 3:遊脚
int getContactState(const PosePtr pose1, const PosePtr pose2, const int linkId, const std::vector<Vector3d>& contactPointVec)
{
    // std::cout << "getContactState()";
    int state = 1;// 2^0

    // check slide (stop: 0, slide: 1)*2^0
    const double deltaPos = 0.01, deltaAngle = 0.02;// 要検討 10[mm] 0.02[rad]=1.15[deg]
    const Pose::LinkInfo *linkInfo1 = pose1->ikLinkInfo(linkId), *linkInfo2 = pose2->ikLinkInfo(linkId);
    std::vector<Vector3d> tmpVec;
    if(contactPointVec.size() == 0) reduceConvexHullToQuadrangle(tmpVec, linkInfo1->contactPoints());// use linkInfo1 and reduce to quadrangle
    const std::vector<Vector3d>& contactPointVec_ =  contactPointVec.size() == 0 ? tmpVec : contactPointVec;
    cout << "  slide check";
    for(auto contactPoint : contactPointVec_){
        double diff = ((linkInfo1->p + linkInfo1->R*contactPoint) - (linkInfo2->p + linkInfo2->R*contactPoint)).norm();
        if(diff < deltaPos){
            state = 0;// set to static
            cout << "  posdiff = " << diff << " < " << deltaPos << " : Static contact!";
            break;
        }else{
            cout << "  posdiff = " << diff << " >= " << deltaPos << ", ";
        }
    }
    cout << endl;

    // check contact (contact: 0, swing: 1)*2^1
    if(!linkInfo1->isTouching() || !linkInfo2->isTouching()){
        cout << "  pose1: " << linkInfo1->isTouching() << "  " << " pose2: " << linkInfo2->isTouching() << endl;
        state += 2;// 2^1
    }

    // std::cout << " >>state:" << state << std::endl;
    return state;
}

Vector3d getDirection(const PosePtr pose1, const PosePtr pose2, const int linkId)
{
    Vector3d ret = pose2->ikLinkInfo(linkId)->p - pose1->ikLinkInfo(linkId)->p;
    ret /= ret.norm();
    // std::cout << "getDirection()>>direction:" << ret.transpose() << std::endl;
    return ret;
}

}

// calc COM, Momentum, Angular Momentum
void cnoid::generateInitSeq(BodyPtr body, PoseSeqItemPtr& poseSeqItemPtr, std::vector<Link*>& endEffectorLinkVec)
{
    cout << "generateInitSeq()" << endl;

    boost::filesystem::path logPath = getLogPath( boost::filesystem::path(poseSeqItemPtr->filePath()) );
    boost::filesystem::create_directory( logPath.parent_path().string().c_str() );
    BodyMotionItemPtr bodyMotionItemPtr = poseSeqItemPtr->bodyMotionItem();
    BodyMotion& motion = *(bodyMotionItemPtr->motion());
    const int frameRate = motion.frameRate();
    const double dt = 1.0/frameRate;
    const double m = body->mass();
    const int numJoints = body->numJoints();

    stringstream ss;
    ss << logPath.string() << "_initCM_" << frameRate << "fps.dat";
    ofstream ofs( ss.str().c_str() );
    ofs << "time initCMx initCMy initCMz initPx initPy initPz initLx initLy initLz" << endl;

    ss.str("");
    ss << logPath.string() << "_initEE_" << frameRate << "fps.dat";
    ofstream eeOfs( ss.str().c_str() );
    eeOfs << "time";
    std::vector<string> columnHeadKeyVec={"px","py","pz", "vx","vy","vz", "wx","wy","wz"};
    for(auto link : endEffectorLinkVec) for(auto columnHeadKey : columnHeadKeyVec) eeOfs << " init" << link->name() << columnHeadKey;
    eeOfs << endl;

    Vector3Seq& initCMSeq = *(bodyMotionItemPtr->motion()->getOrCreateExtraSeq<Vector3Seq>("initCM"));
    Vector3Seq& initPSeq = *(bodyMotionItemPtr->motion()->getOrCreateExtraSeq<Vector3Seq>("initP"));
    Vector3Seq& initLSeq = *(bodyMotionItemPtr->motion()->getOrCreateExtraSeq<Vector3Seq>("initL"));
    MultiValueSeq& initdqSeq = *(bodyMotionItemPtr->motion()->getOrCreateExtraSeq<MultiValueSeq>("initdq"));
    initdqSeq.setNumParts(numJoints,true);
    int numFrames = motion.numFrames();

    cout << " numFrames: " << numFrames << endl;
    MedianFilter filter = MedianFilter(3,3);// sync with delay in generateVerticalTrajectory()
    for(int i=0; i < numFrames; ++i){
        updateBodyState(body, motion, i);
        body->calcForwardKinematics(true,true);

        Vector3d P,L,CM;
        CM = body->calcCenterOfMass();
        body->calcTotalMomentum(P,L);
        L -= CM.cross(P);// convert to around CoM
        // L << 0,0,0; // overwrite L to 0,0,0

        initCMSeq.at(i) = CM;
        // calculate momentum in simple model
        P = filter.update(m*(CM - initCMSeq.at(max(i-1,0)))/dt); //use median filter
        initPSeq.at(i) = P;
        initLSeq.at(i) = L;

        MultiValueSeq::Frame frame = initdqSeq.frame(i);
        for(int j=0; j<numJoints; ++j) frame[j] = body->link(j)->dq();

        ofs << i*dt << " " << CM.transpose() <<  " " << P.transpose() << " " << L.transpose() << " " << endl;
        eeOfs << i*dt;
        for(auto link : endEffectorLinkVec){
            eeOfs << " " << link->p().transpose() << " " << link->v().transpose() << " " << link->w().transpose();
        }
        eeOfs << endl;
    }
    setSubItem("initCM", initCMSeq, bodyMotionItemPtr);
    setSubItem("initP", initPSeq, bodyMotionItemPtr);
    setSubItem("initL", initLSeq, bodyMotionItemPtr);
    setSubItem("initdq", initdqSeq, bodyMotionItemPtr);

    ofs.close();
    eeOfs.close();
}

void cnoid::calcContactLinkCandidateSet(std::set<Link*>& contactLinkCandidateSet, BodyPtr body, const PoseSeqPtr& poseSeqPtr)
{
    cout << endl << "calcContactLinkCandidateSet()" << endl;
    for(PoseSeq::iterator poseIter = (++poseSeqPtr->begin()); poseIter != poseSeqPtr->end(); incContactPose(poseIter,poseSeqPtr,body)){
        if(!isContactStateChanging(poseIter, poseSeqPtr, body))continue;
        PosePtr curPosePtr = poseIter->get<Pose>();
        for(Pose::LinkInfoMap::iterator linkInfoIter = curPosePtr->ikLinkBegin(); linkInfoIter != curPosePtr->ikLinkEnd(); ++linkInfoIter){
            //接触している且つslaveでない
            if(linkInfoIter->second.isTouching() && !linkInfoIter->second.isSlave()) contactLinkCandidateSet.insert(body->link(linkInfoIter->first));
        }
        cout << endl;
    }
    cout << endl << " Contact Link Candidates:";
    for(std::set<Link*>::iterator iter = contactLinkCandidateSet.begin(); iter != contactLinkCandidateSet.end(); ++iter){
        cout << " " << (*iter)->name();
    }
    cout << endl << endl;
}

// どちらの足でもいいので次の接触ポーズにイテレータを進める
// end()を返すこともある
void cnoid::incContactPose(PoseSeq::iterator& poseIter, const PoseSeqPtr poseSeq, const BodyPtr body)
{
    std::cout << "incContactPose()";
    LeggedBodyHelperPtr lgh = getLeggedBodyHelper(body);
    PoseSeq::iterator iter;
    for(iter = (++poseIter); iter != poseSeq->end(); ++iter){// PoseSeq走査
        // std::cout << " " << iter->time();
        PosePtr pose = iter->get<Pose>();
        for(Pose::LinkInfoMap::iterator linkInfoIter = pose->ikLinkBegin(); linkInfoIter != pose->ikLinkEnd(); ++linkInfoIter){// LinkInfoMap走査
            for(int i = 0; i < lgh->numFeet(); ++i){
                if(lgh->footLink(i)->index() == linkInfoIter->first // リンク番号比較
                    && linkInfoIter->second.isTouching()){// 接触確認
                    poseIter = iter;
                    std::cout << " >>next contact pose is link:" << linkInfoIter->first << " time:" << poseIter->time() << std::endl;
                    return;
                }
            }
        }

    }
    poseIter = iter;
    std::cout << " next contact pose not found. return pose time:" << poseIter->time() << std::endl;
    return;
}

void cnoid::incContactPose(PoseSeq::iterator& poseIter, const PoseSeqPtr poseSeq, const Link* link)
{
    std::cout << "incContactPose(poseIter, poseSeq ,link)";
    PoseSeq::iterator iter;
    for(iter = (++poseIter); iter != poseSeq->end(); ++iter){// PoseSeq走査
        PosePtr pose = iter->get<Pose>();
        for(Pose::LinkInfoMap::iterator linkInfoIter = pose->ikLinkBegin(); linkInfoIter != pose->ikLinkEnd(); ++linkInfoIter){// LinkInfoMap走査
            if(link->index() == linkInfoIter->first // リンク番号比較
               && linkInfoIter->second.isTouching()){// 接触確認
                poseIter = iter;
                std::cout << " >>next contact pose is link:" << linkInfoIter->first << " time:" << poseIter->time() << std::endl;
                return;
            }
        }

    }
    poseIter = iter;
    std::cout << " next contact pose not found. return pose time:" << poseIter->time() << std::endl;
    return;
}

bool cnoid::getNextTermContactPoints(std::vector<Vector3d>& contactPointVec, const PoseSeq::iterator poseIter, const PoseSeqPtr poseSeq, const int linkIdx)
{
    std::cout << "getNextTermContactPoints(" << poseIter->time() << " s linkIdx:" << linkIdx << ")" << std::endl ;
    PoseSeq::iterator nextPoseIter = getNextPose( poseIter, poseSeq, linkIdx );
    if(poseIter == nextPoseIter){std::cout << " this is final pose" << std::endl; return false;}
    ::getContactPoints(contactPointVec, getPrevPose( nextPoseIter, poseSeq, linkIdx )->get<Pose>(), nextPoseIter->get<Pose>(), linkIdx );
    return true;
}

bool cnoid::getPrevTermContactPoints(std::vector<Vector3d>& contactPointVec, const PoseSeq::iterator poseIter, const PoseSeqPtr poseSeq, const int linkIdx)
{
    std::cout << "getPrevTermContactPoints(" << poseIter->time() << " s linkIdx:" << linkIdx << ")" << std::endl ;
    PoseSeq::iterator prevPoseIter = getPrevPose( poseIter, poseSeq, linkIdx );
    if(poseIter == prevPoseIter){std::cout << " this is first pose" << std::endl; return false;}
    ::getContactPoints(contactPointVec, prevPoseIter->get<Pose>(), getNextPose( prevPoseIter, poseSeq, linkIdx )->get<Pose>(), linkIdx );
    return true;
}

// poseIterが最後のポーズの時は-1を返す
int cnoid::getNextContactState(const PoseSeq::iterator poseIter, const PoseSeqPtr poseSeq, const int linkId, const std::vector<Vector3d>& contactPointVec)
{
    std::cout << "getNextContactState(" << poseIter->time() << " s linkId:" << linkId << ")" << std::endl ;
    PoseSeq::iterator nextPoseIter = getNextPose( poseIter, poseSeq, linkId );
    if(poseIter == nextPoseIter){std::cout << " this is final pose" << std::endl; return -1;}
    return ::getContactState( getPrevPose( nextPoseIter, poseSeq, linkId )->get<Pose>(), nextPoseIter->get<Pose>(), linkId, contactPointVec );
}

int cnoid::getPrevContactState(const PoseSeq::iterator poseIter, const PoseSeqPtr poseSeq, const int linkId, const std::vector<Vector3d>& contactPointVec)
{
    std::cout << "getPrevContactState(" << poseIter->time() << " s linkId:" << linkId << ")" << std::endl ;
    PoseSeq::iterator prevPoseIter = getPrevPose( poseIter, poseSeq, linkId );
    if(poseIter == prevPoseIter){std::cout << " this is first pose" << std::endl; return -1;}
    return ::getContactState( prevPoseIter->get<Pose>(), getNextPose( prevPoseIter, poseSeq, linkId )->get<Pose>(), linkId, contactPointVec );
}

// 特定の接触状態のLinkSetを取得
void cnoid::getNextTargetContactLinkSet(std::set<Link*>& linkSet, BodyPtr& body, const int contactState, const std::set<Link*>& contactLinkCandidateSet, const PoseSeq::iterator poseIter, const PoseSeqPtr poseSeq)
{
    for(std::set<Link*>::iterator candidateLinkIter = contactLinkCandidateSet.begin(); candidateLinkIter != contactLinkCandidateSet.end(); ++candidateLinkIter){
        int linkIdx = (*candidateLinkIter)->index();
        if(contactState == getNextContactState(poseIter, poseSeq, linkIdx)) linkSet.insert(body->link(linkIdx));
    }
}

Vector3d cnoid::getPrevDirection(const PoseSeq::iterator poseIter, const PoseSeqPtr poseSeq, const int linkId)
{
    std::cout << "getPrevDirection(" << poseIter->time() << " s linkid:" << linkId << ")" << std::endl;
    PoseSeq::iterator prevPoseIter = getPrevPose(poseIter, poseSeq, linkId);
    if(poseIter == prevPoseIter){std::cout << " this is first pose" << std::endl; return Vector3d::Zero();}
    return getDirection(prevPoseIter->get<Pose>(), getNextPose(prevPoseIter, poseSeq, linkId)->get<Pose>(), linkId);
}

// 足の接触状態しか見ていない lgh要改良
bool cnoid::isContactStateChanging(PoseSeq::iterator poseIter, PoseSeqPtr poseSeq, BodyPtr body)
{
    std::cout << "isContactStateChanging(" << poseIter->time() <<  " s)" << std::endl;

    PosePtr prevPose,curPose,nextPose;
    PoseSeq::iterator prevIter,nextIter;
    curPose = poseIter->get<Pose>();

    LeggedBodyHelperPtr lbh = getLeggedBodyHelper(body);
    for(Pose::LinkInfoMap::iterator linkInfoIter = curPose->ikLinkBegin(); linkInfoIter != curPose->ikLinkEnd(); ++linkInfoIter){
        std::cout << "  linkID:" << linkInfoIter->first << " link name:" << body->link(linkInfoIter->first)->name() << " isTouching:" << linkInfoIter->second.isTouching() << std::endl;
        for(int i = 0; i < lbh->numFeet(); ++i){
            if(lbh->footLink(i)->index() == linkInfoIter->first && linkInfoIter->second.isTouching()){// 足先リンクで且つ接触している
                int prevState,nextState;
                if((prevState = getPrevContactState(poseIter, poseSeq, linkInfoIter->first)) != (nextState = getNextContactState(poseIter, poseSeq, linkInfoIter->first))){// 前後の接触状態比較
                    std::cout << " " << "\x1b[33m" << body->link(linkInfoIter->first)->name() << "'s state changing at " << poseIter->time() << " s: " << prevState << "->" << nextState << "\x1b[m" << std::endl;
                    return true;
                }else{
                    std::cout << " " << body->link(linkInfoIter->first)->name() << "'s state is not changing or not touching at " << poseIter->time() << std::endl;
                }
            }
        }
    }

    std::cout << " state not changing or no foot ikLink or no contact link" << std::endl;
    return false;
}

bool cnoid::isContactStateChanging(PoseSeq::iterator poseIter, PoseSeqPtr poseSeq, const Link* link)
{
    std::cout << "isContactStateChanging(" << poseIter->time() <<  " s, " << link->name() << ")" << std::endl;

    PosePtr prevPose,curPose,nextPose;
    PoseSeq::iterator prevIter,nextIter;
    curPose = poseIter->get<Pose>();

    for(Pose::LinkInfoMap::iterator linkInfoIter = curPose->ikLinkBegin(); linkInfoIter != curPose->ikLinkEnd(); ++linkInfoIter){
        std::cout << "  linkID:" << linkInfoIter->first << " link name:" << link->name() << " isTouching:" << linkInfoIter->second.isTouching() << std::endl;
        if(link->index() == linkInfoIter->first && linkInfoIter->second.isTouching()){// 足先リンクで且つ接触している
            int prevState,nextState;
            if((prevState = getPrevContactState(poseIter, poseSeq, linkInfoIter->first)) != (nextState = getNextContactState(poseIter, poseSeq, linkInfoIter->first))){// 前後の接触状態比較
                std::cout << " " << "\x1b[33m" << link->name() << "'s state changing at " << poseIter->time() << " s: " << prevState << "->" << nextState << "\x1b[m" << std::endl;
                return true;
            }else{
                std::cout << " " << link->name() << "'s state is not changing or not touching at " << poseIter->time() << std::endl;
            }
        }
    }

    std::cout << link->name() << "'s state not changing" << std::endl;
    return false;
}

void UtilPlugin::getFootLink(Link** plFootLink, Link** prFootLink, const BodyPtr& body)
{
  cout << "getFootLink()" << endl;

  // yamlから直接読み込み
  // YAMLReader parser;
  // parser.load("/home/kunio/Dropbox/choreonoid/models/HRP2JSK/hrp2jsk.yaml");
  // MappingPtr info = parser.document()->toMapping();
  // Listing& linkGroupList = *info->findListing("footLinks");
  // ValueNode& node = footLinkList[0]; Mapping& group = *node.toMapping();

  // bodyのinfoからlisting
  // Listing& footLinkInfos = *body->info()->findListing("footLinks");
  // if(!footLinkInfos.isValid()){
  //   cout << "Please load model from yaml file" << endl;
  //   return;
  // }
  // Link* rFootLink = body->link( (*footLinkInfos[0].toMapping())["link"].toString() );
  // Link* lFootLink = body->link( (*footLinkInfos[1].toMapping())["link"].toString() );

  // getLeggedBodyHelperを使う
  LeggedBodyHelperPtr legged = getLeggedBodyHelper(body);

  if(legged->isValid()){
    *prFootLink = body->link(legged->footLink(0)->index());
    *plFootLink = body->link(legged->footLink(1)->index());
    cout << "Foot Links: " << (*plFootLink)->name() << " " << (*prFootLink)->name() << endl;
  }else{
    cout << "Please load model from yaml file" << endl;
  }

  cout << "Finished setting foot links" << endl;
}

bool cnoid::getSelectedPoseSeqSet(BodyItemPtr& bodyItemPtr, BodyPtr& body,
                                  PoseSeqItemPtr& poseSeqItemPtr, PoseSeqPtr& poseSeqPtr,
                                  BodyMotionItemPtr& bodyMotionItem)
{
    ItemList<PoseSeqItem> selectedItemList = ItemTreeView::mainInstance()->selectedItems<Item>();
    if(selectedItemList.empty()){cout << "Select PoseSeqItem!!" << endl; return false;}
    bodyItemPtr = selectedItemList[0]->findOwnerItem<BodyItem>();
    body = bodyItemPtr->body();
    cout << "Robot:" << body->name() << endl;

    poseSeqItemPtr = selectedItemList[0];
    cout << poseSeqItemPtr->name() << endl;

    poseSeqPtr = poseSeqItemPtr->poseSeq();
    bodyMotionItem = poseSeqItemPtr->bodyMotionItem();

    return true;
}

void cnoid::generateBodyMotionFromBar(BodyPtr& body, const PoseSeqItemPtr& poseSeqItemPtr, const BodyMotionItemPtr& bodyMotionItemPtr)
{
    BodyMotionGenerationBar* bmgb = BodyMotionGenerationBar::instance();
    PoseProvider* provider = poseSeqItemPtr->interpolator().get();
    bmgb->shapeBodyMotion(body, provider, bodyMotionItemPtr, true);
    cout << "Generated motion" << endl;
}

void cnoid::generateOptionalData(BodyPtr& body, const PoseSeqItemPtr& poseSeqItemPtr, const std::vector<Link*>& linkVec)
{
    cout << endl << "generateOptionalData()" << endl;

    PoseSeqPtr poseSeqPtr = poseSeqItemPtr->poseSeq();
    BodyMotionItemPtr bodyMotionItemPtr = poseSeqItemPtr->bodyMotionItem();
    BodyMotion& motion = *(bodyMotionItemPtr->motion());

    int linkNum = linkVec.size();
    int frameRate = motion.frameRate();
    MultiValueSeq& optionalDataSeq = *(motion.getOrCreateExtraSeq<MultiValueSeq>("optionaldata"));
    optionalDataSeq.setNumParts(linkNum*2,true);

    int j = 0;
    for(auto link: linkVec){
        for(PoseSeq::iterator frontPoseIter = (++poseSeqPtr->begin()),backPoseIter = poseSeqPtr->begin(); frontPoseIter != poseSeqPtr->end(); incContactPose(frontPoseIter,poseSeqPtr,link)){
            if(!isContactStateChanging(frontPoseIter, poseSeqPtr, link)) continue;
            int contactState = getPrevContactState(frontPoseIter, poseSeqPtr, link->index());
            int optionalDataState;
            if(contactState == 0) optionalDataState = 1;// 0:静止接触
            if(contactState == 1) optionalDataState = -1;// 1:滑り接触
            if(contactState > 1)  optionalDataState = 0;

            for(int i=backPoseIter->time()*frameRate; i < frontPoseIter->time()*frameRate; ++i){
                MultiValueSeq::Frame frame = optionalDataSeq.frame(i);
                frame[j] = optionalDataState;
                frame[j+linkNum] = frontPoseIter->time() - i/(double)frameRate;
            }
            cout << endl;
            backPoseIter = frontPoseIter;
        }
        ++j;
    }

    // final frame is the same with the previous frame
    {
        MultiValueSeq::Frame finalFrame = optionalDataSeq.frame(motion.numFrames()-1);
        MultiValueSeq::Frame preFinalFrame = optionalDataSeq.frame(motion.numFrames()-2);
        for(int j=0; j<linkNum*2; ++j) finalFrame[j] = preFinalFrame[j];
    }
    cout << endl << endl;

    setSubItem("optionaldata", optionalDataSeq, bodyMotionItemPtr);
}

// 接触力によるトルクのみ計算 リンクの慣性力によるトルクは含まれていない
void cnoid::generateTorque(BodyPtr& body, const PoseSeqItemPtr& poseSeqItemPtr, const std::vector<Link*>& linkVec)
{
    cout << endl << "generateTorque()" << endl;

    PoseSeqPtr poseSeqPtr = poseSeqItemPtr->poseSeq();
    BodyMotionItemPtr bodyMotionItemPtr = poseSeqItemPtr->bodyMotionItem();
    BodyMotion& motion = *(bodyMotionItemPtr->motion());

    MultiValueSeq& torqueSeq = *(motion.getOrCreateExtraSeq<MultiValueSeq>("torque"));
    torqueSeq.setNumParts(body->numJoints(),true);

    MultiValueSeq& wrenchSeq = *(bodyMotionItemPtr->findSubItem<MultiValueSeqItem>("wrenches")->seq());

    const int wrenchDim = 6;
    std::vector<std::shared_ptr<JointPath>> jointPathVec;
    for(auto link : linkVec){
        jointPathVec.push_back(std::make_shared<JointPath>(body->rootLink(), link));
    }
    for(int i=0; i<motion.numFrames(); ++i){
        updateBodyState(body, motion, i);
        body->calcForwardKinematics();

        MultiValueSeq::Frame torqueFrame = torqueSeq.frame(i);
        MultiValueSeq::Frame wrenchFrame = wrenchSeq.frame(i);
        int idxOffset = 0;
        for(auto jointPathPtr : jointPathVec){
            VectorXd wrench(wrenchDim);
            for(int j=0; j<wrenchDim; ++j) wrench(j) = wrenchFrame[j+idxOffset];

            MatrixXd J;
            jointPathPtr->calcJacobian(J);
            VectorXd tau = -J.transpose()*wrench;
            for(int j=0; j<jointPathPtr->numJoints(); ++j) torqueFrame[jointPathPtr->joint(j)->jointId()] += tau(j);

            idxOffset += wrenchDim;
        }
    }
    cout << endl << endl;

    setSubItem("torque", torqueSeq, bodyMotionItemPtr);
}

bool cnoid::getEndEffectorLinkVector(std::vector<Link*>& endEffectorLinkVec, BodyPtr& body)
{
    cout << endl << "generateEndEffectorLinkVector( " << body->name() << " )" << endl;
    const Listing& endEffectorNodes = *body->info()->findListing("endEffectors");
    if(endEffectorNodes.isValid() && endEffectorNodes.size() != 0){
        cout << " End effector link vector contains";
        for(int i=0; i<endEffectorNodes.size(); ++i){
            endEffectorLinkVec.push_back(body->link(endEffectorNodes[i].toString()));
            cout << " " << endEffectorNodes[i].toString();
        }cout << " in this order" << endl;
        return true;
    }else{
        cout << " Please set endEffectorNodes in yaml file" << endl << endl;
        return false;
    }
}

void cnoid::updateBodyState(BodyPtr& body, const BodyMotion& motion, const int currentFrame, const std::set<Link*>& linkSet)
{
    int prevFrame = max(currentFrame-1, 0);
    int nextFrame = min(currentFrame+1, motion.numFrames()-1);

    const double dt = 1.0/motion.frameRate();

    // 特定リンクのvとwの更新
    if(linkSet.size() != 0){
        (BodyMotion::ConstFrame) motion.frame(prevFrame) >> *body;
        body->calcForwardKinematics();
        std::map<std::string, SE3> SE3Map;
        for(std::set<Link*>::iterator iter = linkSet.begin(); iter != linkSet.end(); ++iter){
            string linkName = (*iter)->name();
            SE3 se3;
            se3.translation() = body->link(linkName)->p();
            se3.rotation() = body->link(linkName)->R();
            SE3Map[linkName] = se3;
        }
        (BodyMotion::ConstFrame) motion.frame(currentFrame) >> *body;
        body->calcForwardKinematics();
        for(std::set<Link*>::iterator iter = linkSet.begin(); iter != linkSet.end(); ++iter){
            string linkName = (*iter)->name();
            SE3 se3 = SE3Map[linkName];
            body->link(linkName)->v() = (body->link(linkName)->p() - se3.translation())/dt;
            Matrix3d R = body->link(linkName)->R() * se3.rotation().toRotationMatrix().inverse();
            AngleAxis aa = AngleAxis(R);
            body->link(linkName)->w() = aa.axis() * aa.angle()/dt;
        }
    }

    (BodyMotion::ConstFrame) motion.frame(currentFrame) >> *body;

		Vector3d v,w;
		VectorXd dq,ddq;
		calcDifferential(motion, currentFrame, v, w, dq, ddq);

    // dq,ddq更新
    for(int k=0; k < motion.numJoints(); ++k){
        Link* joint = body->joint(k);
        // joint->q() = q[k];
        joint->dq() = dq[k];
        joint->ddq() = ddq[k];
    }

		// rootLink v,x更新
    body->rootLink()->v() = v;
    body->rootLink()->w() = w;
}

void cnoid::calcDifferential(const BodyMotion& motion, const int currentFrame, Vector3d& v, Vector3d& w, VectorXd&dq, VectorXd& ddq)
{
    int prevFrame = max(currentFrame-1, 0);
    int nextFrame = min(currentFrame+1, motion.numFrames()-1);

    const double dt = 1.0/motion.frameRate();

    // dq,ddq計算
    dq = VectorXd(motion.numJoints());
    ddq = VectorXd(motion.numJoints());
    MultiValueSeq::Frame q0 = motion.jointPosSeq()->frame(prevFrame);
    MultiValueSeq::Frame q1 = motion.jointPosSeq()->frame(currentFrame);
    MultiValueSeq::Frame q2 = motion.jointPosSeq()->frame(nextFrame);
    for(int k=0; k < motion.numJoints(); ++k){
        // Link* joint = mBody->joint(k);
        // joint->q() = q1[k];
        dq[k] = (q2[k] - q1[k]) / dt;
        ddq[k] = (q2[k] - 2.0 * q1[k] + q0[k]) / (dt * dt);
    }

    // rootLink v,x計算
    SE3 currentWaistSE3 = motion.linkPosSeq()->frame(currentFrame)[0];
    SE3 nextWaistSE3 = motion.linkPosSeq()->frame(nextFrame)[0];
    v = nextWaistSE3.translation() - currentWaistSE3.translation();
    v /= dt;
    // Matrix3d R = nextWaistSE3.rotation().toRotationMatrix() * currentWaistSE3.rotation().toRotationMatrix().inverse(); // old and wrong
    Matrix3d R = currentWaistSE3.rotation().toRotationMatrix().inverse() * nextWaistSE3.rotation().toRotationMatrix();// R(t)^T * R(t+dt)
    AngleAxis aa = AngleAxis(R);
    // w = aa.axis() * aa.angle()/dt; // old and wrongx
    w = currentWaistSE3.rotation().toRotationMatrix() * aa.axis() * aa.angle()/dt;// w is in world frame
}

void cnoid::calcTotalMomentum(Vector3d& P, Vector3d& L, BodyPtr& body, const VectorXd& dq)
{
    MatrixXd M,H;
    calcCMJacobian(body, NULL, M);// 運動量ヤコビアン、角運動量ヤコビアン(重心基準)
    calcAngularMomentumJacobian(body, NULL, H);

    VectorXd velocityVec(6+dq.size());
    velocityVec << dq, body->rootLink()->v(), body->rootLink()->w();
    P = body->mass() * M * velocityVec;
    L = H * velocityVec;
}

Matrix3d cnoid::D(Vector3d r)
{
    Matrix3d r_cross;
    r_cross <<
        0.0,  -r(2), r(1),
        r(2),    0.0,  -r(0),
        -r(1), r(0),    0.0;
    return r_cross.transpose() * r_cross;
}


void cnoid::setSubItem(std::string seqName, const Vector3Seq& seq, BodyMotionItem* pBodyMotionItem)
{
    Vector3SeqItemPtr seqItemPtr = pBodyMotionItem->findSubItem<Vector3SeqItem>(seqName);
    if(!seqItemPtr){
        seqItemPtr = new Vector3SeqItem(std::make_shared<Vector3Seq>(seq));
        seqItemPtr->setName(seqName);
        pBodyMotionItem->addSubItem(seqItemPtr);
    }else{
        seqItemPtr->seq() = std::make_shared<Vector3Seq>(seq);
    }
}

void cnoid::setSubItem(std::string seqName, const MultiValueSeq& seq, BodyMotionItem* pBodyMotionItem)
{
    MultiValueSeqItemPtr seqItemPtr = pBodyMotionItem->findSubItem<MultiValueSeqItem>(seqName);
    if(!seqItemPtr){
        seqItemPtr = new MultiValueSeqItem(std::make_shared<MultiValueSeq>(seq));
        seqItemPtr->setName(seqName);
        pBodyMotionItem->addSubItem(seqItemPtr);
    }else{
        seqItemPtr->seq() = std::make_shared<MultiValueSeq>(seq);
    }
}

double cnoid::thresh(double x, double thresh, double target){return fabs(x) > thresh ? x : target;}
VectorXd cnoid::thresh(VectorXd x, double thresh, VectorXd target){return x.norm() > thresh ? x : (x.size() == target.size() ? target : VectorXd::Constant(x.size(),x[0]));}

CNOID_IMPLEMENT_PLUGIN_ENTRY(UtilPlugin)
