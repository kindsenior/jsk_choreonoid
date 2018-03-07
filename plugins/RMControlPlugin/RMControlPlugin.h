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
#include <cnoid/YAMLReader>

#include <cnoid/src/Body/Jacobian.h>
#include <cnoid/src/PoseSeqPlugin/PoseSeqItem.h>
// #include <cnoid/src/PoseSeqPlugin/gettext.h>
#include <cnoid/src/PoseSeqPlugin/BodyMotionGenerationBar.h>
#include <cnoid/Vector3SeqItem>

#include <cnoid/PoseProvider>
#include <cnoid/BodyMotionPoseProvider>
#include <cnoid/PoseProviderToBodyMotionConverter>
#include <cnoid/BodyMotionUtil>
#include <cnoid/TimeBar>
#include <cnoid/Archive>
#include <cnoid/MenuManager>
#include <cnoid/MainWindow>
#include <cnoid/SpinBox>
// #include <cnoid/Separator>
#include <cnoid/Button>
#include <QDialog>
#include <QDialogButtonBox>
#include <boost/format.hpp>
#include <set>

#include <cnoid/FileUtil>

#include <UtilPlugin/UtilPlugin.h>
#include <UtilPlugin/Interpolator.h>
#include <UtilPlugin/InverseKinematics.h>

#include "RMControlBar.h"

namespace cnoid {

enum Constraint { Free , LLEG , RLEG };// 拘束条件

class RMControlBar;

class RMControlPlugin : public Plugin
{
protected:
    RMControlBar* mBar;

public:
    std::vector<SubMass> mSubMasses;
    BodyPtr mBody;
    JointPathPtr mJpl;
    JointPathPtr mJpr;
    Link* mLFootLink;
    Link* mRFootLink;
    boost::filesystem::path mPoseSeqPath;

    RMControlPlugin() : Plugin("RMControl")
    {
        require("Body");
        require("PoseSeq");
    }

    virtual bool initialize();

    // ロボットモデル依存の部分あり
    // 各種行列を計算
    void calcMatrixies(MatrixXd& A_, MatrixXd& Jl, MatrixXd& Jr, MatrixXd& Fl, MatrixXd& Fr,
                       MatrixXd& M, MatrixXd& H, MatrixXd& Mb, MatrixXd& Mfree, MatrixXd& Hb, MatrixXd& Hfree,
                       MatrixXd& Ml, MatrixXd& Mr, MatrixXd& Hl, MatrixXd& Hr, std::vector<Constraint>& jointConstraintVec);

    // 目標運動量・角運動量軌道生成
    // void generateRefPLSeq(BodyPtr body,BodyItem* bodyItem, const BodyMotionPtr motion,const PoseSeqPtr poseSeq,
    void generateRefPLSeq(BodyMotionItem* motionItem ,const PoseSeqPtr poseSeq,
                          const Vector3d initP, const Vector3d endP, const Vector3d initL, const Vector3d endL);

    void loadRefPLSeq(BodyMotionItem* motionItem ,const PoseSeqPtr poseSeq,
                      const Vector3d initP, const Vector3d endP, const Vector3d initL, const Vector3d endL);

    void generateRefWrenchSeq(BodyPtr& body, PoseSeqItemPtr& poseSeqItem, const std::vector<Link*>& endEffectorLinkVec);

    void modifyJumpingTrajectory(PoseSeqItemPtr& poseSeqItem, const std::set<Link*>& contactLinkCandidateSet);

    void sweepControl(boost::filesystem::path poseSeqPath ,std::string paramStr, BodyPtr& body, BodyMotionItemPtr& bodyMotionItemPtr, const std::set<Link*>& contactLinkCandidateSet);

    // 分解運動量制御
    void execControl();

};

}
