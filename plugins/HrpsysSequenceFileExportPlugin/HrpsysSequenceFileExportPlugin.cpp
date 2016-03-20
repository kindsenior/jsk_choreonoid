/**
   @author Kunio Kojima
*/
#include "HrpsysSequenceFileExportPlugin.h"

using namespace boost;
using namespace cnoid;
using namespace std;

void HrpsysSequenceFileExportPlugin::HrpsysSequenceFileExport()
{
  cout << "\x1b[31m" << "Start HrpsysSequenceFileExport" << "\x1b[m" << endl;

  // BodyItem,BodyPtr
  ItemList<BodyItem> bodyItems = ItemTreeView::mainInstance()->checkedItems<BodyItem>();
  BodyPtr body = bodyItems[0]->body();// ロボットモデルは１個のみチェック

  LeggedBodyHelperPtr lgh = getLeggedBodyHelper(body);

  PoseSeqItem* poseSeqItem = ItemTreeView::mainInstance()->selectedItems<PoseSeqItem>()[0];
  PoseSeqPtr poseSeq = poseSeqItem->poseSeq();

  BodyMotionItem* bodyMotionItem = poseSeqItem->bodyMotionItem();
  BodyMotionPtr motion = bodyMotionItem->motion();

  string poseSeqPathString = poseSeqItem->filePath();
  boost::filesystem::path poseSeqPath(poseSeqPathString);
  cout << " parent_path:" << poseSeqPath.parent_path().string() << " basename:" << getBasename(poseSeqPath) << endl;

  stringstream ss;
  ss << poseSeqPath.parent_path().string() << "/" << poseSeqItem->name() << ".optionaldata";
  FILE* fp = fopen(ss.str().c_str(),"w"); if( fp == NULL ){ cout << "\x1b[31m" << "optionaldata file("<< ss.str() << ") open error" << "\x1b[m" << endl; return; }

  double frameRate = motion->frameRate();

  for( PoseSeq::iterator prevPoseIter = poseSeq->begin(), poseIter = poseSeq->begin(); poseIter != poseSeq->end(); prevPoseIter = poseIter ){
      incContactPose( poseIter, poseSeq, body );
      cout << "phase:" << prevPoseIter->time() << "[sec]->" << poseIter->time() << "[sec]" << endl;

      std::vector<int> contactStates;
      for( int i = 0; i < lgh->numFeet(); ++i ){
          contactStates.push_back( getPrevContactState( poseIter, poseSeq, lgh->footLink(i)->index() ) );
      }

      // 同じ時刻のキーポーズが連続するとダメ
      for( int i = (int) (prevPoseIter->time()*frameRate); i < poseIter->time()*frameRate; ++i ){
          fprintf(fp,"%lf %d %d 0 0   5 5 5 5\n", (double)i/frameRate, contactStates[0] > 0 ? 0 : 1, contactStates[1] > 0 ? 0 : 1);// hrpsys-baseの実装に合わせて静止接触だけ1
      }
      cout << endl << endl;
  }

  fclose(fp);

  cout << "\x1b[31m" << "Finished HrpsysSequenceFileExport" << "\x1b[m" << endl << endl;
}



CNOID_IMPLEMENT_PLUGIN_ENTRY(HrpsysSequenceFileExportPlugin)
