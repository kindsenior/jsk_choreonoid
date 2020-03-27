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
#include <cnoid/src/PoseSeqPlugin/PoseSeqItem.h>
#include <cnoid/src/Body/InverseDynamics.h>
#include <cnoid/Vector3Seq>

namespace cnoid {

// zmp計算
void calcZMP(const BodyPtr& body, BodyMotion& motion, Vector3Seq& zmpSeq, const bool checkLargeDiff = false);

class DynamicsPlugin : public Plugin
{
public:
    
  DynamicsPlugin() : Plugin("Dynamics")
  {
    require("Body");
    require("PoseSeq");
    require("Util");
  }
    
  virtual bool initialize()
  {
    return true;
  }

};

}
/* CNOID_IMPLEMENT_PLUGIN_ENTRY(DynamicsPlugin) */
