/**
   @author Kunio Kojima
*/
#include <iostream>
#include <sstream>

#include <unistd.h>

#include <cnoid/Plugin>
#include <cnoid/ItemTreeView>
#include <cnoid/BodyItem>
#include <cnoid/WorldItem>
#include <cnoid/ToolBar>
#include <boost/bind.hpp>
#include <cnoid/MessageView>
#include <cnoid/Archive>

#include <cnoid/BodyMotionGenerationBar>
#include <cnoid/src/PoseSeqPlugin/PoseSeqItem.h>
#include <cnoid/PoseProvider>
#include <cnoid/BodyMotionPoseProvider>
#include <cnoid/PoseProviderToBodyMotionConverter>
#include <cnoid/BodyMotionUtil>
#include <cnoid/TimeBar>
#include <cnoid/Archive>
#include <cnoid/MenuManager>
#include <cnoid/MainWindow>
#include <cnoid/SpinBox>
// #include <cnoid/Separator>
#include <cnoid/Button>
#include <QDialog>
#include <QDialogButtonBox>
#include <boost/format.hpp>
#include <set>
// #include "../../../src/PoseSeqPlugin/gettext.h"

using namespace boost;
using namespace cnoid;
using namespace std;

class RhythmBalancerPlugin : public Plugin
{
public:
    
  RhythmBalancerPlugin() : Plugin("RhythmBalancer")
  {
    require("Body");
  }
    
  virtual bool initialize()
  {
    ToolBar* bar = new ToolBar("RhythmBalancer");

    BodyMotionGenerationBar* bmgb = BodyMotionGenerationBar::instance();// インスタンス生成
    // bar->addButton("motion")
    //   ->sigClicked().connect(bind(&BodyMotionGenerationBar::generateBodyMotion, bmgb));// コールバック関数登録

    bar->addButton("sweep")
      ->sigClicked().connect(bind(&RhythmBalancerPlugin::onButtonClicked, this, false));
    bar->addButton("print")
      ->sigClicked().connect(bind(&RhythmBalancerPlugin::onButtonClicked, this, true));

    addToolBar(bar);

    return true;
  }

  void onButtonClicked(bool print_flg)
  {
    stringstream ss;

    // アイテム
    set<BodyMotionItem*> motionItems; // for avoiding overlap
    ItemList<Item> selectedItems = ItemTreeView::mainInstance()->selectedItems<Item>();
    for(size_t i=0; i < selectedItems.size(); ++i){
      // ss << selectedItems[i]->name() << " ";
    }
    // MessageView::instance()->putln(ss.str());
    // 選択したアイテムではなく特定のアイテムを取得したい時は？
    // 選択したアイテムではなく全てのアイテムを取得したい時は？

    TimeBar* tb = TimeBar::instance();;
    // tb->setTime(3);    // 時刻設定
    // ss << "time " << tb->time() << endl;    // 現在時刻取得

    // 関節・リンク情報取得・設定
    ItemList<BodyItem> bodyItems = ItemTreeView::mainInstance()->checkedItems<BodyItem>();
    // checkedItems チェックされたアイテムを取得
    if(print_flg){
      MessageView::instance()->putln("Print called !");
    
      // ItemList<PoseSeqItem> poseSeqItems = bodyItems[0]->getSubItems<PoseSeqItem>();// サブアイテムを取得
      // for( size_t i = 0; i < poseSeqItems.size(); ++i){
      //   ss << poseSeqItems[i]->name() << endl;
      // }

      for(size_t i=0; i < bodyItems.size(); ++i){
        BodyPtr body = bodyItems[i]->body();

        // ss << "baselink " << bodyItems[i]->currentBaseLink()->name() << endl;// BaseLink

        for(int j=0; j < body->numLinks(); ++j){

          // linkのメンバ変数については適宜 Link.hを参照

          // ss << "joint " << j << endl;
          // ss << body->link(j)->name() << endl;// リンク名

          // ss << body->joint(j)->q << " ";// 単位はラジアン
          // ss << "R "<< endl << body->link(j)->R << endl<< "p " << endl << body->link(j)->p << endl;// 姿勢・位置(座標は軸上)

          // ss << "dq "<<  body->link(j)->dq << "   ddq " << body->link(j)->ddq << endl;;// dq,ddq,uとかは今一不明
          // ss << "vel" << endl << body->link(j)->v << endl;
          // ss << "angle vel" << endl << body->link(j)->w << endl;
          // dq,velは一度モーションを作成して、タイムバーの時刻を設定するとできる？？angle velは不明


          // ss << body->link("RLEG_JOINT0")->q << " ";// リンク名で指定可能
          // ss << "mass " << body->link(j)->m << endl;// 質量 kg
          // ss << "inertia "<< endl << body->link(j)->I << endl;// inertia
          // body->getJointPath(body->link(0), body->link(3));
          // ss << "local cog" << endl << body->link(j)->c << endl;// リンク重心位置
          // ss << "world cog" << endl << body->link(j)->wc << endl;// リンク重心位置 calcCM()を行なってから
          // ss << "parent name " <<  body->link(2)->parent->name() << endl;// 親リンク

          // ss << endl;
        }
        // ss << "total cog" << endl << body->calcCM() << endl;// 全重心
        // ss << "tatal mass" << endl << body->totalMass() << endl;// 総重量
  
        bodyItems[i]->notifyKinematicStateChange(true);// GUIに反映 bodyItemに対して行う
        body->calcCenterOfMass();// wcを更新
        body->calcForwardKinematics(true);// 順運動学計算 ここでq,dq,ddqの変更を反映してv,w dv,dwを更新
        Vector3 L;Vector3 P;
        body->calcTotalMomentum(P,L);
        // ss << "momentum" << endl << P << endl;
        // ss << "angular momentum" << endl <<  L << endl;

        // // zmp取得
        // BodyItem* bodyItem = bodyItems[0];
        // optional<Vector3> p = bodyItem->getParticularPosition(BodyItem::ZERO_MOMENT_POINT);
        // ss << (*p)[0] << " " << (*p)[1] << " " << (*p)[2] << endl;
        // Vector3 zmp = bodyItem->zmp();
        //   ss << zmp[0] << " " << zmp[1] << " " << zmp[2] << endl;

      }
      MessageView::instance()->putln(ss.str());
      return;
    }// end prit

    MessageView::instance()->putln("sweep called !");
    // 各PoseSeqへアクセス
    ItemList<PoseSeqItem> poseSeqItems = ItemTreeView::mainInstance()->selectedItems<Item>();
    for(size_t i=0; i < poseSeqItems.size(); ++i){
      PoseSeqPtr poseSeq = poseSeqItems[i]->poseSeq();
      // ss << poseSeq->name();// 名前取得
      // ss << poseSeq->beginningTime() << " ";// 開始時刻
      // ss << poseSeq->endingTime() << " ";// 終了時刻

      BodyPtr body = bodyItems[0]->body();

      // // BodyMotionの各Frameへのアクセス
      // BodyMotionItem* bodyMotionItem = poseSeqItems[i]->bodyMotionItem();
      // BodyMotionPtr motion = bodyMotionItem->motion();
      // Vector3Seq zmpSeq = *(motion->ZMPseq());

      // ss << "frame num " << motion->numFrames() << endl; 
      // for(int j=0;j < motion->numFrames(); ++j){// zmpSeq.numFrames()でもOK

      //   // dq,ddqは毎回この操作で求めなければならない
      //   int currentFrame = j;
      //   int prevFrame = std::max(currentFrame - 1, 0);
      //   int nextFrame = std::min(currentFrame + 1, motion->numFrames());
      //   MultiValueSeq::Frame q0 = motion->jointPosSeq()->frame(prevFrame);
      //   MultiValueSeq::Frame q1 = motion->jointPosSeq()->frame(currentFrame);
      //   MultiValueSeq::Frame q2 = motion->jointPosSeq()->frame(nextFrame);
      //   double dt = 1/motion->frameRate();
      //   double dt2 = dt * dt;
      //   for(int k=0; k < motion->numJoints(); ++k){
      //     Link* joint = body->joint(k);
      //     joint->q = q1[k];
      //     joint->dq = (q2[k] - q1[k]) / dt;
      //     joint->ddq = (q2[k] - 2.0 * q1[k] + q0[k]) / dt2;
      //   }

      //   if( j % 1000 == 0){
      //     // for(int k = 0; k < body->numJoints(); ++k){
      //     //   ss << "dq " << body->joint(k)->dq << "  ddq " << body->joint(k)->ddq << endl;
      //     // }

      //     // ss << "time " << j/motion->frameRate() << endl;// フレームレート取得 motion時刻取得
      //     // body->calcCM();body->calcForwardKinematics(true);
      //     // Vector3 L;Vector3 P;
      //     // body->calcTotalMomentum(P,L);
      //     // ss << "momentum" << endl << P << endl;
      //     // ss << "angular momentum" << endl <<  L << endl;

      //     // motion → frame → motion
      //     // ss << motion->jointPosSeq()->frame(j-1)[20] << endl;
      //     // ss << motion->frame(j).motion().jointPosSeq()->frame(j-1)[20] << endl;
      //     // ss << motion->frame(j).motion().jointPosSeq()->frame(j)[20] << endl;
      //     // ss << motion->frame(j).motion().numJoints() << endl;

      //     ss << endl;
      //   }

      //   if( j % 10 == 0 ){
      //   // if( j == 100 ){
      //     // ss << "frame " << j << endl;

      //     // ss << "time " << j/motion->frameRate() << endl;// フレームレート取得 motion時刻取得
      //     motion->frame(j) >> *(bodyItems[0]->body());// motionをbodyに反映
      //     // bodyItems[0]->notifyKinematicStateChange(true);// GUIに反映 bodyItemに対して行う

      //     // BodyMotion::Frame frame = motion->frame(j);
      //     MultiValueSeq::Frame q = motion->jointPosSeq()->frame(j);// jはframe.frame()でもいい
      //     MultiSE3Seq::Frame links = motion->linkPosSeq()->frame(j);

      //     // SE3とは??
      //     // SO(3) 3次特殊直交群 eg R
      //     // so(3) 歪対称行列の集合 eg [r×]
      //     // SE(3) 同次変換の集合 (特殊ユークリッド群) eg H
      //     // se(3) ツイストによる歪対称行列の集合 eg [xi×]

      //     // zmp取得
      //     // BodyItem* bodyItem = bodyItems[0];
      //     // // optional<Vector3> p = bodyItem->getParticularPosition(BodyItem::ZERO_MOMENT_POINT);// 今一不明
      //     // // ss << (*p)[0] << " " << (*p)[1] << " " << (*p)[2] << endl;
      //     // Vector3 zmp = bodyItem->zmp();
      //     // ss << zmp.x() << " " << zmp.y() << " " << zmp.z() << endl;// 今一不明

      //     // ss << " zmp " <<  zmpSeq[j].x() << " " << zmpSeq[j].y() << " " << zmpSeq[j].z() << endl;// 目標zmp[m]

      //     // ss << " q ";
      //     // for(int k=0; k < motion->numJoints(); ++k)ss << q[k]  << " ";
      //     // ss << endl;

      //     for(int k=0; k < motion->numLinks(); ++k){// motionでは腰リンクのみ
      //       // ss << "translation" << endl << links[k].translation();
      //       // ss << "rotaotion" << links[k].rotation();// エラー
      //     }


      //     // rootLink v,x更新
      //     // SE3 waistLink0 = motion->linkPosSeq()->frame(prevFrame)[0];
      //     // SE3 waistLink1 = motion->linkPosSeq()->frame(currentFrame)[0];
      //     // SE3 waistLink2 = motion->linkPosSeq()->frame(nextFrame)[0];
      //     // body->rootLink()->v = (waistLink2.translation() - waistLink1.translation()) / dt;
      //     // Eigen::Quaternion<double,0> qua1 = waistLink1.rotation();
      //     // Eigen::Quaternion<double,0> qua2 = waistLink2.rotation();
      //      // AngleAxis aa = AngleAxis( qua1.inverse() * qua2  );
      //     // // AngleAxis aa;          // aa.fromRotationMatrix( Matrix3::Identity() );
      //     // body->rootLink()->w = aa.axis() * aa.angle()/dt;
      //     // ss << "root v before <<" << endl << body->rootLink()->v << endl;
      //     // ss << "root w before <<" << endl << body->rootLink()->w << endl;


      //     // SE3 rootH = motion->linkPosSeq()->frame(nextFrame)[0];
      //     // ss << "translation " << rootH.translation() << endl;
      //     // ss << "axis " << AngleAxis(rootH.rotation()).axis() << endl;
      //     // ss << "angle " << AngleAxis(rootH.rotation()).angle() << endl;

      //     // ss << endl;
      //   }
      // }

      // 各Poseへアクセス
      for(PoseSeq::iterator poseIter = poseSeq->begin(); poseIter != poseSeq->end(); ++poseIter){
        PosePtr pose = poseIter->get<Pose>();// PosePtr取得

        // tb->setTime(poseIter->time());
        // BodyPtr body = bodyItems[0]->body();
        // ss << body->link("RLEG_JOINT0")->dq << endl;
        // if(pose->isZmpValid()){// zmpはposeRollで有効になってないと0（初期値）のまま
        //   ss << pose->zmp().x() << " " << pose->zmp().y() << " " << pose->zmp().z() << endl;// zmp??
        // }

        // ss << poseIter->time();// pose時刻取得
        // ss << poseIter->maxTransitionTime();// pose遷移時間取得
        // for(int j = 0;j < pose->numJoints(); j++){
          // ss << pose->jointPosition(j) << " ";// 関節速度・加速度も知りたい
        // }

        // for(int j = 0;j < pose->numKeyposes(); j++){// keypose情報アクセス
        //   ss << pose->keyposeInfos[j].q << " ";// jointPosition(j)に当たるものを実装したほうがいい? keyposeInfosは本来private
        // }


        // LinkInfoは yaml で defaultIKsetup しているリンクが、jointIdの順に並んでいる
        // for(Pose::LinkInfoMap::iterator itr = pose->ikLinkBegin(); itr != pose->ikLinkEnd(); ++itr){
        //   ss << "isBaseLink " << itr->second.isBaseLink() << endl;
        //   ss << "isTouching " << itr->second.isTouching() << endl;
        // }ss << endl;
        

        // ss << endl;
      }// 各poseに対するforの終わり


    }// 各poseSeqに対するforの終わり

    // WorldItemへのアクセス
    {
        ItemList<PoseSeqItem> selectedItems = ItemTreeView::mainInstance()->selectedItems<Item>();
        // RootItem* rootItem = selectedItems[0]->findRootItem();
        // cout << "RootItem: "<<  rootItem->name() << endl;
        WorldItem* ownerItem = selectedItems[0]->findOwnerItem<WorldItem>();
        cout << "OwnerItem: "<<  ownerItem->name() << endl;// ワールドアイテム
        Item* childItem = ownerItem->childItem();//ボディーアイテム
        cout << "ChildItem: " << childItem->name() << endl;
        Item* item = ownerItem->findItem("AISTシミュレータ");// シミュレータアイテム
        cout << "Item: " << item->name() << endl;
        Archive archive;
        item->store(archive);
        double staticFriction,slipFriction;
        archive.read("staticFriction",staticFriction); archive.read("slipFriction",slipFriction);
        cout << "StaticFriction: " << staticFriction << "  SlipFriction: " << slipFriction << endl;
    }

    MessageView::instance()->putln(ss.str());
  }
};

// Item
// BodyItem
// PoseSeqItem
// BodyMotionItem

CNOID_IMPLEMENT_PLUGIN_ENTRY(RhythmBalancerPlugin)

// /PosePlugin/Pose.cpp
// Pose::store()
