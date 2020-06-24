/**
   @author Kunio Kojima
*/
#pragma once

#include <iostream>
#include <sstream>
#include <fstream>
#include <map>

#include <time.h>

#include <boost/bind.hpp>

#include <cnoid/Plugin>
#include <cnoid/ItemTreeView>
#include <cnoid/BodyItem>
#include <cnoid/MessageView>
#include <cnoid/src/PoseSeqPlugin/PoseSeqItem.h>
#include <cnoid/src/PoseSeqPlugin/BodyMotionGenerationBar.h>
#include <cnoid/FileUtil>

#include <UtilPlugin/UtilPlugin.h>
#include <MultiContactStabilizerPlugin/MultiContactStabilizerPlugin.h>
#include <MultiContactStabilizerPlugin/MultiContactStabilizer.h>
#include <SlideFrictionControlPlugin/SlideFrictionControlPlugin.h>

#include "CascadedControlBar.h"

namespace cnoid{
    void interpolateExtraSeq(BodyMotionItemPtr& bodyMotionItemPtr, double T);

    class CascadedControlBar;

    class CascadedControlPlugin : public Plugin
    {
    private:
        std::ofstream mOfs;
        hrp::MultiContactStabilizer* mcs;
        Vector3d lastP;
        std::vector<int> failIdxVec;

    public:
        BodyPtr body;
        BodyMotionItemPtr mBodyMotionItemPtr;
        CascadedControlBar* mBar;

        boost::filesystem::path mLogPath;

        int frameRate;
        double dt;
        int numFrames;

        CascadedControlPlugin() : Plugin("CascadedControl")
        {
            require("Body");
            require("PoseSeq");
            require("MultiContactStabilizer");
            require("SlideFrictionControl");
            // require("Dynamics");
        }

        virtual bool initialize();
        void execControl(bool loadFlg);
    };

}
