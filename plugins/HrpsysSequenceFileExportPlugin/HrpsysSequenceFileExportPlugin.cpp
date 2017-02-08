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

  double dt = ((double) 1)/motion->frameRate();
  int numFrames = motion->numFrames();

  std::vector<string> extentionVec{"wrenches","optionaldata"};
  for(auto ext : extentionVec){
      stringstream fnamess;
      fnamess << poseSeqItem->name() << "." << ext;
      ofstream ofs;
      ofs.open(((filesystem::path) poseSeqPath.parent_path() / fnamess.str()).string().c_str(), ios::out);
      MultiValueSeqPtr multiValueSeqPtr = bodyMotionItem->findSubItem<MultiValueSeqItem>(ext)->seq();

      for(int i=0; i<numFrames; ++i){
          ofs << i*dt;
          MultiValueSeq::Frame frame = multiValueSeqPtr->frame(i);
          for(int j=0; j<frame.size(); ++j){
              ofs << " " << frame[j];
          }
          ofs << endl;
      }
      ofs.close();
  }

  cout << "\x1b[31m" << "Finished HrpsysSequenceFileExport" << "\x1b[m" << endl << endl;
}



CNOID_IMPLEMENT_PLUGIN_ENTRY(HrpsysSequenceFileExportPlugin)
