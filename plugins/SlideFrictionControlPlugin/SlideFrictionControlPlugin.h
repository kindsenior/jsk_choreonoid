/**
   @author Kunio Kojima
*/
#pragma once

#include <iostream>
#include <sstream>
#include <fstream>
#include <map>
#include <numeric>

#include <time.h>

#include <boost/bind.hpp>

#include <cnoid/Plugin>
#include <cnoid/ItemTreeView>
#include <cnoid/BodyItem>
#include <cnoid/MessageView>
#include <cnoid/src/PoseSeqPlugin/PoseSeqItem.h>
#include <cnoid/src/PoseSeqPlugin/BodyMotionGenerationBar.h>
#include <cnoid/FileUtil>
#include <QEventLoop>

#include <UtilPlugin/UtilPlugin.h>
#include <UtilPlugin/GeometricUtil.h>
#include <UtilPlugin/Interpolator.h>

#include "SlideFrictionControl.h"
#include "SlideFrictionControlBar.h"

namespace cnoid{
    void generatePreModelPredictiveControlParamDeque(hrp::SlideFrictionControl* sfc, BodyItemPtr bodyItemPtr, const PoseSeqItemPtr& poseSeqItemPtr, const std::set<Link*>& contactLinkCandidateSet);
    void generateVerticalTrajectory(BodyPtr& body, const PoseSeqItemPtr& poseSeqItem, const std::set<Link*> contactLinkCandidateSet, const std::vector<double>& takeoffPhaseRatioVec, const std::vector<double>& landingPhaseRatioVec);
    void sweepControl(boost::filesystem::path logPath ,std::string paramStr, hrp::SlideFrictionControl* sfc, BodyPtr& body, BodyMotionItemPtr& bodyMotionItemPtr, const std::set<Link*>& contactLinkCandidateSet);
    void loadExtraSeq(boost::filesystem::path logPath ,std::string paramStr, hrp::SlideFrictionControl* sfc, BodyPtr& body, BodyMotionItemPtr& bodyMotionItemPtr, const std::set<Link*>& contactLinkCandidateSet);

    class SlideFrictionControlBar;

    class SlideFrictionControlPlugin : public Plugin
    {
    private:
        hrp::SlideFrictionControl* sfc;
        std::vector<int> failIdxVec;

    public:
        BodyPtr body;
        BodyMotionItemPtr mBodyMotionItemPtr;
        SlideFrictionControlBar* mBar;

        boost::filesystem::path mLogPath;

        int frameRate;
        double dt;
        int numFrames;

        SlideFrictionControlPlugin() : Plugin("SlideFrictionControl")
        {
            require("Body");
            require("PoseSeq");
            // require("Dynamics");
        }

        virtual bool initialize();
        void execControl();

    };

}
