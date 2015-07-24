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
#include <cnoid/Vector3Seq>
#include <cnoid/LeggedBodyHelper>

namespace cnoid{

    // 2つのPose間の接触状態を2進数で表す
    // 0:静止接触 1:滑り接触 (2:静止遊脚) 3:遊脚
    int getContactState( const PosePtr pose1, const PosePtr pose2, const int linkId );

    // どちらの足でもいいので次の接触ポーズにイテレータを進める
    // end()を返すこともある
    void incContactPose( PoseSeq::iterator& poseIter, const PoseSeqPtr poseSeq, const BodyPtr body );

    // 特定のリンクの次のポーズを求める
    PoseSeq::iterator getNextPose( PoseSeq::iterator poseIter, PoseSeqPtr poseSeq, int linkId );

    // 特定リンクの前のポーズを求める
    PoseSeq::iterator getPrevPose( PoseSeq::iterator poseIter, PoseSeqPtr poseSeq, int linkId );

    // poseIterが最後のポーズの時は-1を返す
    int getNextContactState( const PoseSeq::iterator poseIter, const PoseSeqPtr poseSeq, const int linkId );

    int getPrevContactState( const PoseSeq::iterator poseIter, const PoseSeqPtr poseSeq, const int linkId );

    bool isContactStateChanging( PoseSeq::iterator poseIter, PoseSeqPtr poseSeq, BodyPtr body );


  class UtilPlugin : public Plugin
  {
  public:
    
  UtilPlugin() : Plugin("Util")
      {
        require("Body");
        require("PoseSeq");
      }
    
    virtual bool initialize()
    {
      return true;
    }

    static void getFootLink( Link** lFootLink, Link** rFootLink, const BodyPtr& body );

  };

}

/* CNOID_IMPLEMENT_PLUGIN_ENTRY(UtilPlugin) */
