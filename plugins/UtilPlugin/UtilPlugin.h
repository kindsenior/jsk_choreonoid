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
#include <cnoid/ValueTree>

#include "Filter.h"

namespace cnoid{


void generateInitSeq(BodyPtr body, PoseSeqItemPtr& poseSeqItemPtr, std::vector<Link*>& endEffectorLinkVector);

void calcContactLinkCandidateSet(std::set<Link*>& contactLinkCandidateSet, BodyPtr body, const PoseSeqPtr& poseSeqPtr);

PoseSeq::iterator getNextPose(PoseSeq::iterator poseIter, PoseSeqPtr poseSeq, int linkId);
PoseSeq::iterator getPrevPose(PoseSeq::iterator poseIter, PoseSeqPtr poseSeq, int linkId);

// どちらの足でもいいので次の接触ポーズにイテレータを進める
// end()を返すこともある
void incContactPose(PoseSeq::iterator& poseIter, const PoseSeqPtr poseSeq, const BodyPtr body);
void incContactPose(PoseSeq::iterator& poseIter, const PoseSeqPtr poseSeq, const Link* link);

// poseIterが最後のポーズの時は-1を返す
int getNextContactState(const PoseSeq::iterator poseIter, const PoseSeqPtr poseSeq, const int linkId, const std::vector<Vector3d>& contactPointVec = std::vector<Vector3d>());

int getPrevContactState(const PoseSeq::iterator poseIter, const PoseSeqPtr poseSeq, const int linkId, const std::vector<Vector3d>& contactPointVec = std::vector<Vector3d>());

void getNextTargetContactLinkSet(std::set<Link*>& linkSet, BodyPtr& body, const int contactState, const std::set<Link*>& contactLinkCandidateSet, const PoseSeq::iterator poseIter, const PoseSeqPtr poseSeq);

Vector3d getPrevDirection(const PoseSeq::iterator poseIter, const PoseSeqPtr poseSeq, const int linkId);

bool isContactStateChanging(PoseSeq::iterator poseIter, PoseSeqPtr poseSeq, BodyPtr body);
bool isContactStateChanging(PoseSeq::iterator poseIter, PoseSeqPtr poseSeq, const Link* link);

bool getSelectedPoseSeqSet(BodyItemPtr& bodyItemPtr, BodyPtr& body,
                           PoseSeqItemPtr& poseSeqItemPtr, PoseSeqPtr& poseSeqPtr,
                           BodyMotionItemPtr& bodyMotionItem);

void generateBodyMotionFromBar(BodyPtr& body, const PoseSeqItemPtr& poseSeqItemptr, const BodyMotionItemPtr& bodyMotionItemPtr);

void generateOptionalData(BodyPtr& body, const PoseSeqItemPtr& poseSeqItemPtr, const std::vector<Link*>& linkVec);
void generateTorque(BodyPtr& body, const PoseSeqItemPtr& poseSeqItemPtr, const std::vector<Link*>& linkVec);

bool getEndEffectorLinkVector(std::vector<Link*>& endEffectorLinkVector, BodyPtr& body);

void updateBodyState(BodyPtr& body, const BodyMotion& motion, const int currentFrame, const std::set<Link*>& linkSet=std::set<Link*>());

void calcDifferential(const BodyMotion& motion, const int currentFrame, Vector3d& v, Vector3d& w, VectorXd&dq, VectorXd& ddq);

void calcTotalMomentum(Vector3d& P, Vector3d& L, BodyPtr& body, const VectorXd& dq);

Matrix3d D(Vector3d r);


void setSubItem(std::string seqName, const Vector3Seq& seq, BodyMotionItem* pBodyMotionItem);
void setSubItem(std::string seqName, const MultiValueSeq& seq, BodyMotionItem* pBodyMotionItem);

double thresh(double x, double thresh, double target = 0);
VectorXd thresh(VectorXd x, double thresh, VectorXd target = VectorXd::Zero(3));

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
