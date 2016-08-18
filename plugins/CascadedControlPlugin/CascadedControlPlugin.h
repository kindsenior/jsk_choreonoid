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

#include "CascadedControlBar.h"

namespace cnoid{
    class CascadedControlBar;

    class CascadedControlPlugin : public Plugin
    {
    private:
        Vector3SeqPtr mRefCMSeqPtr, mRefPSeqPtr, mRefLSeqPtr;
        std::ofstream mOfs;
        hrp::MultiContactStabilizer* mcs;
        Vector3d lastP;
        std::vector<int> failIdxVec;

    public:
        BodyPtr body;
        BodyMotionItemPtr mBodyMotionItemPtr;
        BodyMotionPtr motion;
        CascadedControlBar* mBar;

        boost::filesystem::path mPoseSeqPath;

        int frameRate;
        double dt;
        int numFrames;

        CascadedControlPlugin() : Plugin("CascadedControl")
        {
            require("Body");
            require("PoseSeq");
            require("MultiContactStabilizer");
            // require("Dynamics");
        }

        virtual bool initialize();
        void execControl();
    };

}
