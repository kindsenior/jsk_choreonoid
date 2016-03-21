#include <iostream>
#include <sstream>
#include <fstream>
/* #include <map> */

/* #include <unistd.h> */

#include <cnoid/Plugin>
#include <cnoid/ItemTreeView>
#include <cnoid/BodyItem>
#include <cnoid/ToolBar>
#include <boost/bind.hpp>
#include <cnoid/src/PoseSeqPlugin/PoseSeqItem.h>
#include <cnoid/src/PoseSeqPlugin/Pose.h>
/* #include <cnoid/MessageView> */
/* #include <cnoid/ItemManager> */
#include <cnoid/FileUtil>
/* #include <cnoid/BodyMotionGenerationBar> */
/* #include <cnoid/JointPath> */

/* #include <cnoid/LeggedBodyHelper> */
/* #include <cnoid/AppConfig> */

/* #include <DynamicsPlugin/DynamicsPlugin.h> */
#include <UtilPlugin/UtilPlugin.h>

namespace cnoid{

class HrpsysSequenceFileExportPlugin : public Plugin
{
public:
    
    HrpsysSequenceFileExportPlugin() : Plugin("HrpsysSequenceFileExport")
    {
        require("Body");
        /* require("Dynamics"); */
        require("Util");
    }
    
    virtual bool initialize()
    {
        ToolBar* bar = new ToolBar("HrpsysSequenceFileExportPlugin");
        bar->addButton("Export")->sigClicked().connect(boost::bind(&HrpsysSequenceFileExportPlugin::HrpsysSequenceFileExport, this));
        /* bar->setVisibleByDefault(true); */ /* 関係ない?? */

        addToolBar(bar);

        return true;
    }

    void HrpsysSequenceFileExport();

};

}
