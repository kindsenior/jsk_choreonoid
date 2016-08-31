/**
   @author Kunio Kojima
*/
#pragma once

#include <iostream>
#include <sstream>
#include <fstream>
#include <map>

#include <unistd.h>

#include <cnoid/Plugin>
#include <cnoid/ItemTreeView>
#include <cnoid/BodyItem>
#include <cnoid/ToolBar>
#include <boost/bind.hpp>
#include <cnoid/MessageView>
#include <cnoid/src/PoseSeqPlugin/PoseSeqItem.h>
#include <cnoid/BodyMotionGenerationBar>
#include <cnoid/Vector3Seq>
#include <cnoid/LeggedBodyHelper>
#include <cnoid/src/Body/Jacobian.h>
#include <cnoid/Vector3SeqItem>
#include <cnoid/FileUtil>

namespace cnoid{

struct SubMass
{
    double m;
    Vector3 mwc;
    Matrix3d Iw;
    SubMass& operator+=(const SubMass& rhs){
        m += rhs.m;
        mwc += rhs.mwc;
        Iw += rhs.Iw;
        return *this;
    }
};

void generateInitSeq(BodyPtr body, PoseSeqItemPtr& poseSeqItemPtr);

void calcContactLinkCandidateSet(std::set<Link*>& contactLinkCandidateSet, BodyPtr body, const PoseSeqPtr& poseSeqPtr);

// 2つのPose間の接触状態を2進数で表す
// 0:静止接触 1:滑り接触 (2:静止遊脚) 3:遊脚
int getContactState(const PosePtr pose1, const PosePtr pose2, const int linkId);

// どちらの足でもいいので次の接触ポーズにイテレータを進める
// end()を返すこともある
void incContactPose(PoseSeq::iterator& poseIter, const PoseSeqPtr poseSeq, const BodyPtr body);

// 特定のリンクの次のポーズを求める
PoseSeq::iterator getNextPose(PoseSeq::iterator poseIter, PoseSeqPtr poseSeq, int linkId);

// 特定リンクの前のポーズを求める
PoseSeq::iterator getPrevPose(PoseSeq::iterator poseIter, PoseSeqPtr poseSeq, int linkId);

// poseIterが最後のポーズの時は-1を返す
int getNextContactState(const PoseSeq::iterator poseIter, const PoseSeqPtr poseSeq, const int linkId);

int getPrevContactState(const PoseSeq::iterator poseIter, const PoseSeqPtr poseSeq, const int linkId);

Vector3d getDirection(const PosePtr pose1, const PosePtr pose2, const int linkId);

Vector3d getPrevDirection(const PoseSeq::iterator poseIter, const PoseSeqPtr poseSeq, const int linkId);

bool isContactStateChanging(PoseSeq::iterator poseIter, PoseSeqPtr poseSeq, BodyPtr body);

bool getSelectedPoseSeqSet(BodyItemPtr& bodyItemPtr, BodyPtr& body,
                           PoseSeqItemPtr& poseSeqItemPtr, PoseSeqPtr& poseSeqPtr,
                           BodyMotionItemPtr& bodyMotionItem, BodyMotionPtr& motion);

void generateBodyMotionFromBar(BodyPtr& body, const PoseSeqItemPtr& poseSeqItemptr, const BodyMotionItemPtr& bodyMotionItemPtr);

void updateBodyState(BodyPtr& body, const BodyMotionPtr& motion, const int currentFrame, const std::set<Link*>& linkSet=std::set<Link*>());

void calcDifferential(const BodyMotionPtr& motion, const int currentFrame, Vector3d& v, Vector3d& w, VectorXd&dq, VectorXd& ddq);

void calcTotalMomentum(Vector3d& P, Vector3d& L, BodyPtr& body, const Matrix3d& Iw, const VectorXd& dq);

Matrix3d D(Vector3d r);

void calcSubMass(Link* link, std::vector<SubMass>& subMasses);

void setSubItem(std::string seqName, const Vector3SeqPtr& seqPtr, BodyMotionItem* pBodyMotionItem);

template <typename t_matrix>
t_matrix PseudoInverse(const t_matrix& m, const double &tolerance=1.e-6);

class UtilPlugin : public Plugin
{
public:
    UtilPlugin() : Plugin("Util")
    {
        require("Body");
        require("PoseSeq");
    }
    
    virtual bool initialize()
    {
        return true;
    }

    static void getFootLink( Link** lFootLink, Link** rFootLink, const BodyPtr& body );

};

}

/* CNOID_IMPLEMENT_PLUGIN_ENTRY(UtilPlugin) */
