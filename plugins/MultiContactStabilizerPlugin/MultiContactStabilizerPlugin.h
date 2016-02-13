/**
   @author Kunio Kojima
*/
#pragma once

#include <iostream>
#include <sstream>
#include <fstream>
#include <map>
// #include <set>

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

#include "Controller.h"
#include "test.h"

namespace cnoid{

    class MultiContactStabilizerPlugin : public Plugin
    {
    public:
        BodyPtr body;
        BodyMotionItem* mBodyMotionItem;
        BodyMotionPtr motion;

        boost::filesystem::path mPoseSeqPath;
        int frameRate;
        double dt;
        int numFrames;

        MultiContactStabilizerPlugin() : Plugin("MultiContactStabilizer")
        {
            require("Body");
            require("PoseSeq");
            // require("Dynamics");
        }
    
        virtual bool initialize()
        {
            ToolBar* bar = new ToolBar("MultiContactStabilizer");

            bar->addButton("MCS")
                ->sigClicked().connect(boost::bind(&MultiContactStabilizerPlugin::execControl, this));

            bar->setVisibleByDefault(true);// 効かない?
            addToolBar(bar);

            return true;
        }

        void generateSeq();
        void generateContactConstraintParamVec(std::vector<hrp::ContactConstraintParam>& ccParamVec, const std::set<Link*>& contactLinkCantidateSet, PoseSeq::iterator poseIter, const PoseSeqPtr& poseSeqPtr);
        void generateMultiContactStabilizerParam(hrp::MultiContactStabilizerParam& mcsParam, BodyPtr body, std::vector<hrp::ContactConstraintParam>& ccParamVec);
        void execControl();

    };

}
