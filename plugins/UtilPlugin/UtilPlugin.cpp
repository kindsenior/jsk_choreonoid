/**
   @author Kunio Kojima
*/

#include "UtilPlugin.h"

using namespace boost;
using namespace cnoid;
using namespace std;

// 2つのPose間の接触状態を2進数で表す
// 0:静止接触 1:滑り接触 (2:静止遊脚) 3:遊脚
int cnoid::getContactState( const PosePtr pose1, const PosePtr pose2, const int linkId ){
    std::cout << "getContactState()";
    int state = 0;

    // 滑り判定(静止0 滑り1)
    double delta = 0.005;// 要検討 5[mm]
    // std::cout << pose1->ikLinkInfo(linkInfoIter->first)->p.transpose() << " " << pose2->ikLinkInfo(linkInfoIter->first)->p.transpose() << std::endl;
    if( (pose1->ikLinkInfo(linkId)->p - pose2->ikLinkInfo(linkId)->p).norm() > delta ){
        state += 1;// 2^0
    }

    // 遊脚判定(接触0 非接触1)
    if( !pose1->ikLinkInfo(linkId)->isTouching() || !pose2->ikLinkInfo(linkId)->isTouching() ){
        state += 2;// 2^1
    }

    std::cout << " >>state:" << state << std::endl;
    return state;
}

// どちらの足でもいいので次の接触ポーズにイテレータを進める
// end()を返すこともある
void cnoid::incContactPose( PoseSeq::iterator& poseIter, const PoseSeqPtr poseSeq, const BodyPtr body ){
    std::cout << "incContactPose()";
    LeggedBodyHelperPtr lgh = getLeggedBodyHelper(body);
    PoseSeq::iterator iter;
    for( iter = (++poseIter); iter != poseSeq->end(); ++iter ){// PoseSeq走査
        // std::cout << " " << iter->time();
        PosePtr pose = iter->get<Pose>();
        for( Pose::LinkInfoMap::iterator linkInfoIter = pose->ikLinkBegin(); linkInfoIter != pose->ikLinkEnd(); ++linkInfoIter ){// LinkInfoMap走査
            for( int i = 0; i < lgh->numFeet(); ++i ){
                if( lgh->footLink(i)->index() == linkInfoIter->first // リンク番号比較
                    && linkInfoIter->second.isTouching() ){// 接触確認
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

// 特定のリンクの次のポーズを求める
PoseSeq::iterator cnoid::getNextPose( PoseSeq::iterator poseIter, PoseSeqPtr poseSeq, int linkId ){
    std::cout << "getNextPose(" << poseIter->time() << "[sec] linkId:" << linkId << ")";
    PoseSeq::iterator iter;
    for( iter = (++poseIter); iter != poseSeq->end(); ++iter ){
        // std::cout << " " << iter->time();
        Pose::LinkInfo* linkInfo = iter->get<Pose>()->ikLinkInfo(linkId);
        if( linkInfo ){
            std::cout << " link" << linkId << "'s next pose time:" << iter->time() << std::endl;
            return iter;
        }

    }
    --iter;
    std::cout << " link" << linkId << "'s contact pose not found. return pose time:" << iter->time() << std::endl;
    return iter;
}

// 特定リンクの前のポーズを求める
PoseSeq::iterator cnoid::getPrevPose( PoseSeq::iterator poseIter, PoseSeqPtr poseSeq, int linkId ){
    std::cout << "getPrevPose(" << poseIter->time() << "[sec] linkId:" << linkId << ")";
    PoseSeq::iterator iter;
    for( iter = (--poseIter); iter != (--poseSeq->begin()); --iter ){
        // std::cout << " " << iter->time();
        Pose::LinkInfo* linkInfo = iter->get<Pose>()->ikLinkInfo(linkId);
        if( linkInfo ){
            std::cout << " link" << linkId << "'s prev pose time:" << iter->time() << std::endl;
            return iter;
        }

    }
    ++iter;
    std::cout << " link" << linkId << "'s contact pose not found. return pose time:" << iter->time() << std::endl;
    return iter;
}

// poseIterが最後のポーズの時は-1を返す
int cnoid::getNextContactState( const PoseSeq::iterator poseIter, const PoseSeqPtr poseSeq, const int linkId ){
    std::cout << "getNextContactState(" << poseIter->time() << "[sec] linkId:" << linkId << ")" << std::endl ;
    PoseSeq::iterator nextPoseIter = getNextPose( poseIter, poseSeq, linkId );
    if( poseIter == nextPoseIter ){std::cout << " this is final pose" << std::endl; return -1;}
    return getContactState( getPrevPose( nextPoseIter, poseSeq, linkId )->get<Pose>(), nextPoseIter->get<Pose>(), linkId );
}

int cnoid::getPrevContactState( const PoseSeq::iterator poseIter, const PoseSeqPtr poseSeq, const int linkId ){
    std::cout << "getPrevContactState(" << poseIter->time() << "[sec] linkId:" << linkId << ")" << std::endl ;
    PoseSeq::iterator prevPoseIter = getPrevPose( poseIter, poseSeq, linkId );
    if( poseIter == prevPoseIter ){std::cout << " this is first pose" << std::endl; return -1;}
    return getContactState( prevPoseIter->get<Pose>(), getNextPose( prevPoseIter, poseSeq, linkId )->get<Pose>(), linkId );
}

Vector3d cnoid::getDirection(const PosePtr pose1, const PosePtr pose2, const int linkId)
{
    Vector3d ret = pose2->ikLinkInfo(linkId)->p - pose1->ikLinkInfo(linkId)->p;
    ret /= ret.norm();
    std::cout << "getDirection()>>direction:" << ret.transpose() << std::endl;
    return ret;
}

Vector3d cnoid::getPrevDirection(const PoseSeq::iterator poseIter, const PoseSeqPtr poseSeq, const int linkId)
{
    std::cout << "getPrevDirection(" << poseIter->time() << "[sec] linkid:" << linkId << ")" << std::endl;
    PoseSeq::iterator prevPoseIter = getPrevPose(poseIter, poseSeq, linkId);
    if( poseIter == prevPoseIter ){std::cout << " this is first pose" << std::endl; return Vector3d::Zero();}
    return getDirection(prevPoseIter->get<Pose>(), getNextPose(prevPoseIter, poseSeq, linkId)->get<Pose>(), linkId);
}

// 足の接触状態しか見ていない lgh要改良
bool cnoid::isContactStateChanging( PoseSeq::iterator poseIter, PoseSeqPtr poseSeq, BodyPtr body ){
    std::cout << "isContactStateChanging(" << poseIter->time() <<  "[sec])" << std::endl;

    PosePtr prevPose,curPose,nextPose;
    PoseSeq::iterator prevIter,nextIter;
    curPose = poseIter->get<Pose>();

    LeggedBodyHelperPtr lbh = getLeggedBodyHelper(body);
    for( Pose::LinkInfoMap::iterator linkInfoIter = curPose->ikLinkBegin(); linkInfoIter != curPose->ikLinkEnd(); ++linkInfoIter ){
        std::cout << "  linkID:" << linkInfoIter->first << " link name:" << body->link(linkInfoIter->first)->name() << " " << linkInfoIter->second.isTouching() << std::endl;
        for( int i = 0; i < lbh->numFeet(); ++i ){
            if( lbh->footLink(i)->index() == linkInfoIter->first && linkInfoIter->second.isTouching() ){// 足先リンクで且つ接触している
                if( getPrevContactState( poseIter, poseSeq, linkInfoIter->first ) != getNextContactState( poseIter, poseSeq, linkInfoIter->first ) ){// 前後の接触状態比較
                    std::cout << " " << body->link(linkInfoIter->first)->name() << "'s state changing at " << poseIter->time() << std::endl;
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

void UtilPlugin::getFootLink( Link** plFootLink, Link** prFootLink, const BodyPtr& body ){
  cout << "getFootLink()" << endl;

  // yamlから直接読み込み
  // YAMLReader parser;
  // parser.load("/home/kunio/Dropbox/choreonoid/models/HRP2JSK/hrp2jsk.yaml");
  // MappingPtr info = parser.document()->toMapping();
  // Listing& linkGroupList = *info->findListing("footLinks");
  // ValueNode& node = footLinkList[0]; Mapping& group = *node.toMapping();

  // bodyのinfoからlisting
  // Listing& footLinkInfos = *body->info()->findListing("footLinks");
  // if( !footLinkInfos.isValid() ){
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
  }else{
    cout << "Please load model from yaml file" << endl;
  }

  cout << "Finished setting foot links" << endl;
}

void cnoid::updateBodyState( BodyPtr& body, const BodyMotionPtr& motion, const int currentFrame ){
    int prevFrame = max(currentFrame-1, 0);
    int nextFrame = min(currentFrame+1, motion->numFrames()-1);

    const double dt = 1.0/motion->frameRate();

    motion->frame(currentFrame) >> *body;

		Vector3d v,w;
		VectorXd dq,ddq;
		calcDifferential(motion, currentFrame, v, w, dq, ddq);

    // dq,ddq更新
    for(int k=0; k < motion->numJoints(); ++k){
        Link* joint = body->joint(k);
        // joint->q() = q[k];
        joint->dq() = dq[k];
        joint->ddq() = ddq[k];
    }

		// rootLink v,x更新
    body->rootLink()->v() = v;
    body->rootLink()->w() = w;
}

void cnoid::calcDifferential(const BodyMotionPtr& motion, const int currentFrame, Vector3d& v, Vector3d& w, VectorXd&dq, VectorXd& ddq){
    int prevFrame = max(currentFrame-1, 0);
    int nextFrame = min(currentFrame+1, motion->numFrames()-1);

    const double dt = 1.0/motion->frameRate();

    // dq,ddq計算
    dq = VectorXd(motion->numJoints());
    ddq = VectorXd(motion->numJoints());
    MultiValueSeq::Frame q0 = motion->jointPosSeq()->frame(prevFrame);
    MultiValueSeq::Frame q1 = motion->jointPosSeq()->frame(currentFrame);
    MultiValueSeq::Frame q2 = motion->jointPosSeq()->frame(nextFrame);
    for(int k=0; k < motion->numJoints(); ++k){
        // Link* joint = mBody->joint(k);
        // joint->q() = q1[k];
        dq[k] = (q2[k] - q1[k]) / dt;
        ddq[k] = (q2[k] - 2.0 * q1[k] + q0[k]) / (dt * dt);
    }

    // rootLink v,x計算
    SE3 currentWaistSE3 = motion->linkPosSeq()->frame(currentFrame)[0];
    SE3 nextWaistSE3 = motion->linkPosSeq()->frame(nextFrame)[0];
    v = nextWaistSE3.translation() - currentWaistSE3.translation();
    v /= dt;
    Matrix3d R = nextWaistSE3.rotation().toRotationMatrix() * nextWaistSE3.rotation().toRotationMatrix().inverse();
    AngleAxis aa = AngleAxis(R);
    w = aa.axis() * aa.angle()/dt;
}

void cnoid::calcTotalMomentum(Vector3d& P, Vector3d& L, BodyPtr& body, const Matrix3d& Iw, const VectorXd& dq){
    MatrixXd M,H,M_tmp,H_tmp;
    calcCMJacobian(body, NULL, M_tmp);// 運動量ヤコビアン、角運動量ヤコビアン(重心基準)
    calcAngularMomentumJacobian(body, NULL, H_tmp);
    M = body->mass() * M_tmp.block( 0,0, 3, body->numJoints() );
    H = H_tmp.block( 0,0, 3, body->numJoints() );

    Vector3d r = body->calcCenterOfMass() - body->rootLink()->p();
    P = body->mass() * ( body->rootLink()->v() - r.cross( body->rootLink()->w() ) ) + M * dq;
    L = Iw * body->rootLink()->w() + H * dq;
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

void cnoid::calcSubMass(Link* link, vector<SubMass>& subMasses){
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

void cnoid::setSubItem(std::string seqName, const Vector3SeqPtr& seqPtr, BodyMotionItem* pBodyMotionItem){
    Vector3SeqItemPtr seqItemPtr = pBodyMotionItem->findSubItem<Vector3SeqItem>(seqName);
    if(!seqItemPtr){
        seqItemPtr = new Vector3SeqItem(seqPtr);
        seqItemPtr->setName(seqName);
        pBodyMotionItem->addSubItem(seqItemPtr);
    }else{
        seqItemPtr->seq() = seqPtr;
    }
}

template <typename t_matrix>
t_matrix cnoid::PseudoInverse(const t_matrix& m, const double &tolerance=1.e-6)
{
  using namespace Eigen;
  typedef JacobiSVD<t_matrix> TSVD;
  unsigned int svd_opt(ComputeThinU | ComputeThinV);
  if(m.RowsAtCompileTime!=Dynamic || m.ColsAtCompileTime!=Dynamic)
  svd_opt= ComputeFullU | ComputeFullV;
  TSVD svd(m, svd_opt);
  const typename TSVD::SingularValuesType &sigma(svd.singularValues());
  typename TSVD::SingularValuesType sigma_inv(sigma.size());
  for(long i=0; i<sigma.size(); ++i)
  {
    if(sigma(i) > tolerance)
      sigma_inv(i)= 1.0/sigma(i);
    else
      sigma_inv(i)= 0.0;
  }
  return svd.matrixV()*sigma_inv.asDiagonal()*svd.matrixU().transpose();
}

void Dummy(){
    PseudoInverse<MatrixXd>(MatrixXd::Identity(1,1), 0);
};

CNOID_IMPLEMENT_PLUGIN_ENTRY(UtilPlugin)
