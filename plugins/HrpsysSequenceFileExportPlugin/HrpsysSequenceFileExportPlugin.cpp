/**
   @author Kunio Kojima
*/
#include "HrpsysSequenceFileExportPlugin.h"

using namespace boost;
using namespace cnoid;
using namespace std;

void HrpsysSequenceFileExportPlugin::HrpsysSequenceFileExport(){
  cout << "\x1b[31m" << "Start HrpsysSequenceFileExport" << "\x1b[m" << endl;

  // BodyItem,BodyPtr
  ItemList<BodyItem> bodyItems = ItemTreeView::mainInstance()->checkedItems<BodyItem>();
  BodyPtr body = bodyItems[0]->body();// ロボットモデルは１個のみチェック

  // 足先リンク名取得
  Link *lFootLink, *rFootLink;
  UtilPlugin::getFootLink( &lFootLink, &rFootLink, body );

  ItemList<PoseSeqItem> poseSeqItems = ItemTreeView::mainInstance()->selectedItems<Item>();

  BodyMotionItem* bodyMotionItem = poseSeqItems[0]->bodyMotionItem();
  BodyMotionPtr motion = bodyMotionItem->motion();

  string poseSeqPathString = poseSeqItems[0]->filePath();
  boost::filesystem::path poseSeqPath(poseSeqPathString);
  cout << " parent_path:" << poseSeqPath.parent_path().string() << " basename:" << getBasename(poseSeqPath) << endl;

  stringstream ss;
  ss << poseSeqPath.parent_path().string() << "/" << poseSeqItems[0]->name() << ".optionaldata";
  FILE* fp = fopen(ss.str().c_str(),"w"); if( fp == NULL ){ cout << "\x1b[31m" << "optionaldata file("<< ss.str() << ") open error" << "\x1b[m" << endl; return; }

  double frameRate = motion->frameRate();

  for(PoseSeq::iterator poseItr = poseSeqItems[0]->poseSeq()->begin(); poseItr != (--poseSeqItems[0]->poseSeq()->end()); ++poseItr){
      PoseSeq::iterator nextItr = poseItr; ++nextItr;
      // cout << poseSeqItems[0]->poseSeq()->end()->time() << endl;// iteratorのendのtimeは0

      // PosePtr取得
      PosePtr curPose = poseItr->get<Pose>();
      PosePtr nextPose = nextItr->get<Pose>();

      // for( int j = 0; j < curPose->numJoints(); ++j ){ body->joint(j)->q() =  curPose->jointPosition(j); }
      // body->calcForwardKinematics();

      // for( std::map<int, Pose::LinkInfo>::iterator linkItr = curPose->ikLinkBegin(); linkItr != curPose->ikLinkEnd(); ++linkItr ){
      //     cout << linkItr->first << ":" << linkItr->second.p << " ";
      // }cout << endl << endl;      

      int lFootState = 1, rFootState = 1;
      curPose->ikLinkInfo(lFootLink->index())->p;
      if( (curPose->ikLinkInfo(lFootLink->index())->p - nextPose->ikLinkInfo(lFootLink->index())->p).norm() > 1e-6 ) lFootState = 0;
      if( (curPose->ikLinkInfo(rFootLink->index())->p - nextPose->ikLinkInfo(rFootLink->index())->p).norm() > 1e-6 ) rFootState = 0;

      // 同じ時刻のキーポーズが連続するとダメ
      for( int i = (int) poseItr->time()*frameRate; i < nextItr->time()*frameRate; ++i ){
          fprintf(fp,"%lf %d %d 0 0   5 5 5 5\n", (double)i/frameRate, rFootState, lFootState);
      }
  }

  fclose(fp);

  cout << "\x1b[31m" << "Finished HrpsysSequenceFileExport" << "\x1b[m" << endl << endl;
}



CNOID_IMPLEMENT_PLUGIN_ENTRY(HrpsysSequenceFileExportPlugin)
