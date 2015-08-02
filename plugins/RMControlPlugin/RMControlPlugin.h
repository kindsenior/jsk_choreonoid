/**
   @author Kunio Kojima
*/

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
#include <cnoid/src/PoseSeqPlugin/gettext.h>
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

// #include <DynamicsPlugin/DynamicsPlugin.h>
#include <cnoid/FileUtil>

namespace cnoid{

    enum Constraint { Free , LLEG , RLEG };// 拘束条件

    class RMControlPlugin : public Plugin
    {
    private:
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
                // require("Dynamics");
            }
    
        virtual bool initialize()
        {
            ToolBar* bar = new ToolBar("RMControl");

            bar->addButton("RMControl")
                ->sigClicked().connect(boost::bind(&RMControlPlugin::RMControl, this));

            addToolBar(bar);

            return true;
        }

        void calcSubMass(Link* link, std::vector<SubMass>& subMasses);

        // ロボットモデル依存の部分あり
        // 各種行列を計算
        void calcMatrixies( MatrixXd& A_, MatrixXd& Jl, MatrixXd& Jr, MatrixXd& Fl, MatrixXd& Fr,
                            MatrixXd& M, MatrixXd& H, MatrixXd& Mb, MatrixXd& Mfree, MatrixXd& Hb, MatrixXd& Hfree,
                            MatrixXd& Ml, MatrixXd& Mr, MatrixXd& Hl, MatrixXd& Hr, std::vector<Constraint>& jointConstraintVec);


        // スプライン補間
        void splineInterpolation(const Vector3d f0, const Vector3d v0, const Vector3d f1, const Vector3d v1, const double tau,
                                 Vector3d& a0, Vector3d& a1, Vector3d& a2, Vector3d& a3);


        // 目標運動量・角運動量軌道生成
        // void generateRefPLSeq(BodyPtr body,BodyItem* bodyItem, const BodyMotionPtr motion,const PoseSeqPtr poseSeq,
        void generateRefPLSeq( BodyMotionItem* motionItem ,const PoseSeqPtr poseSeq,
                               const Vector3d initP, const Vector3d endP, const Vector3d initL, const Vector3d endL, 
                               Vector3Seq& refPSeq, Vector3Seq& refLSeq);

        void loadRefPLSeq( BodyMotionItem* motionItem ,const PoseSeqPtr poseSeq,
                               const Vector3d initP, const Vector3d endP, const Vector3d initL, const Vector3d endL, 
                               Vector3Seq& refPSeq, Vector3Seq& refLSeq);

        // 分解運動量制御
        void RMControl();

    };

}
