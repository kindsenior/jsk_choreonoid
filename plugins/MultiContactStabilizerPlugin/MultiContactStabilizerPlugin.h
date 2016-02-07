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
#include <cnoid/Button>
#include <QDialog>
#include <QDialogButtonBox>
#include <boost/format.hpp>
#include <set>

// #include <DynamicsPlugin/DynamicsPlugin.h>
#include <cnoid/FileUtil>

#include <UtilPlugin/UtilPlugin.h>

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
                ->sigClicked().connect(boost::bind(&MultiContactStabilizerPlugin::MultiContactStabilizer, this));

            bar->setVisibleByDefault(true);// 効かない?
            addToolBar(bar);

            return true;
        }

        void generateSeq();
        void MultiContactStabilizer();

    };

}
