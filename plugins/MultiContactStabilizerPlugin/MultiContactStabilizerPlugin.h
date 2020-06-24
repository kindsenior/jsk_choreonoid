/**
   @author Kunio Kojima
*/
#pragma once

#include <iostream>
#include <sstream>
#include <fstream>
#include <map>
// #include <set>

#include <time.h>

// #include <unistd.h>

// #include <boost/format.hpp>
#include <boost/bind.hpp>

#include <cnoid/Plugin>
#include <cnoid/ItemTreeView>
#include <cnoid/BodyItem>
// #include <cnoid/ToolBar>
#include <cnoid/MessageView>
// #include <cnoid/YAMLReader>

// #include <cnoid/src/Body/Jacobian.h>
#include <cnoid/src/PoseSeqPlugin/PoseSeqItem.h>
// #include <cnoid/src/PoseSeqPlugin/gettext.h>
#include <cnoid/src/PoseSeqPlugin/BodyMotionGenerationBar.h>
// #include <cnoid/Vector3SeqItem>
// #include <cnoid/PoseProvider>
// #include <cnoid/BodyMotionPoseProvider>
// #include <cnoid/PoseProviderToBodyMotionConverter>
#include <cnoid/FileUtil>
// #include <cnoid/BodyMotionUtil>
// #include <cnoid/TimeBar>
// #include <cnoid/Archive>
// #include <cnoid/MenuManager>
// #include <cnoid/MainWindow>
// #include <cnoid/SpinBox>
// #include <cnoid/Button>
// #include <QDialog>
// #include <QDialogButtonBox>

// #include <DynamicsPlugin/DynamicsPlugin.h>
#include <UtilPlugin/UtilPlugin.h>

#include "MultiContactStabilizer.h"
#include "MultiContactStabilizerBar.h"

namespace cnoid{
    void generatePreModelPredictiveControlParamDeque(hrp::MultiContactStabilizer* mcs, BodyPtr body, const PoseSeqPtr poseSeqPtr, const BodyMotion& motion, const std::set<Link*>& contactLinkCandidateSet);
    void sweepControl(boost::filesystem::path logPath, std::string paramStr, hrp::MultiContactStabilizer* mcs, BodyPtr& body, BodyMotionItemPtr& bodyMotionItemPtr);

    class MultiContactStabilizerBar;

    class MultiContactStabilizerPlugin : public Plugin
    {
    private:
        std::ofstream mOfs;
        hrp::MultiContactStabilizer* mcs;
        Vector3d lastP;
        std::vector<int> failIdxVec;

    public:
        BodyPtr body;
        BodyMotionItemPtr mBodyMotionItemPtr;
        MultiContactStabilizerBar* mBar;

        boost::filesystem::path mLogPath;

        int frameRate;
        double dt;
        int numFrames;

        MultiContactStabilizerPlugin() : Plugin("MultiContactStabilizer")
        {
            require("Body");
            require("PoseSeq");
            // require("Dynamics");
        }
    
        virtual bool initialize();
        void execControl();

    };

}
