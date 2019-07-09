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
#include <cnoid/PointSetItem>
#include <cnoid/TimeBar>
#include <cnoid/Archive>
#include <cnoid/MenuManager>
#include <cnoid/MainWindow>
#include <cnoid/SpinBox>
// #include <cnoid/Separator>
#include <cnoid/Button>
#include <QDialog>
#include <QDialogButtonBox>
#include <QEventLoop>
#include <boost/format.hpp>
#include <boost/thread.hpp>
#include <set>
// #include "../../../src/PoseSeqPlugin/gettext.h"

#include <UtilPlugin/UtilPlugin.h>

using namespace boost;
using namespace cnoid;
using namespace std;

class SandboxPlugin : public Plugin
{
public:
    Connection connections;
    
    SandboxPlugin() : Plugin("Sandbox")
    {
        require("Body");
    }
    
    virtual bool initialize()
    {
        ToolBar* bar = new ToolBar("Sandbox");

        BodyMotionGenerationBar* bmgb = BodyMotionGenerationBar::instance();// インスタンス生成

        bar->addButton("sweep")
            ->sigClicked().connect(bind(&SandboxPlugin::onButtonClicked, this, false));
        bar->addButton("print")
            ->sigClicked().connect(bind(&SandboxPlugin::onButtonClicked, this, true));

        addToolBar(bar);

        return true;
    }

    void onButtonClicked(bool print_flg)
    {
        cout << "onButtonClicked()" << endl;
        stringstream ss;

        ItemList<Item> selectedItems = ItemTreeView::mainInstance()->selectedItems<Item>();
        cout << "selectedItems: " << selectedItems.size() << endl;

        TimeBar* tb = TimeBar::instance();;
        // tb->setTime(3);    // 時刻設定
        // ss << "time " << tb->time() << endl;    // 現在時刻取得

        // 関節・リンク情報取得・設定
        ItemList<BodyItem> bodyItems = ItemTreeView::mainInstance()->checkedItems<BodyItem>();
        cout << "bodyItems: " << bodyItems.size() << endl;

        // checkedItems チェックされたアイテムを取得
        if(print_flg){
            MessageView::instance()->putln("Print called !");
    
            // ItemList<PoseSeqItem> poseSeqItems = bodyItems[0]->getSubItems<PoseSeqItem>();// サブアイテムを取得
            // for(size_t i = 0; i < poseSeqItems.size(); ++i){
            //   ss << poseSeqItems[i]->name() << endl;
            // }

            for(size_t i=0; i < bodyItems.size(); ++i){
                BodyPtr body = bodyItems[i]->body();
                // ss << "baselink " << bodyItems[i]->currentBaseLink()->name() << endl;// BaseLink

                // zmp取得
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
        QEventLoop eventLoop;
        // 各PoseSeqへアクセス
        ItemList<PoseSeqItem> poseSeqItems = ItemTreeView::mainInstance()->selectedItems<Item>();
        cout << poseSeqItems.size() << " poseseqs are selected" << endl;
        for(size_t i=0; i < poseSeqItems.size(); ++i){
            PoseSeqPtr poseSeq = poseSeqItems[i]->poseSeq();
            cout << "PoseSeq: " <<poseSeq->name() << endl;

            BodyPtr body = bodyItems[0]->body();

            // BodyMotionの各Frameへのアクセス
            BodyMotionItem* bodyMotionItem = poseSeqItems[i]->bodyMotionItem();
            BodyMotionPtr motion = bodyMotionItem->motion();
            const double dt = 1.0/motion->frameRate();

            for(int j=0;j < motion->numFrames(); ++j){// zmpSeq.numFrames()でもOK

              if(j % 10 == 0){
                // BodyMotion::Frame frame = motion->frame(j);
                MultiValueSeq::Frame q = motion->jointPosSeq()->frame(j);// jはframe.frame()でもいい
                MultiSE3Seq::Frame links = motion->linkPosSeq()->frame(j);

                // SE3とは??
                // SO(3) 3次特殊直交群 eg R
                // so(3) 歪対称行列の集合 eg [r×]
                // SE(3) 同次変換の集合 (特殊ユークリッド群) eg H
                // se(3) ツイストによる歪対称行列の集合 eg [xi×]

                // zmp取得
                // BodyItem* bodyItem = bodyItems[0];
                // // optional<Vector3> p = bodyItem->getParticularPosition(BodyItem::ZERO_MOMENT_POINT);// 今一不明
                // // ss << (*p)[0] << " " << (*p)[1] << " " << (*p)[2] << endl;
                // Vector3 zmp = bodyItem->zmp();
                // ss << zmp.x() << " " << zmp.y() << " " << zmp.z() << endl;// 今一不明

                // 姿勢更新
                (BodyMotion::ConstFrame) motion->frame(j) >> *body;
                bodyItems[0]->notifyKinematicStateChange(true);

                // collision取得
                const std::vector<CollisionLinkPairPtr>& collisions = bodyItems[0]->collisions();
                cout << dt*j << " sec:";
                cout << collisions.size() << endl;
                for(size_t i=0; i < collisions.size(); ++i){
                  CollisionLinkPair& collisionPair = *collisions[i];
                  cout << "  " << collisionPair.link[0]->name() << "(" << collisionPair.link[0]->jointId() << ")  " << collisionPair.link[1]->name() << " " << collisionPair.isSelfCollision();
                  for(std::vector<Collision>::iterator iter = collisionPair.collisions.begin(); iter != collisionPair.collisions.end(); ++iter){
                    // cout << " " << iter->point.transpose() << " " << iter->normal.transpose() << endl;
                  }
                }cout << endl;
                eventLoop.processEvents();
              }
            }

            // 各Poseへアクセス
            for(PoseSeq::iterator poseIter = poseSeq->begin(); poseIter != poseSeq->end(); ++poseIter){
                PosePtr pose = poseIter->get<Pose>();// PosePtr取得
                cout << poseIter->time() << "[sec]: ";

                std::string linkName = "LLEG_JOINT5";
                LinkPtr link = body->link(linkName);
                Pose::LinkInfo* linkInfo = pose->ikLinkInfo(link->index());
                if( linkInfo != 0 ){
                    cout << "Found: " << linkName << " "<< linkInfo->isTouching() << " " << linkInfo->isStationaryPoint() << endl;
                    linkInfo->clearTouching();
                }else{
                    cout << "Not found: " << linkName << endl;
                }

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

                // LinkInfoは yaml で defaultIKsetup しているリンクが、jointIdの順に並んでいる
                // for(Pose::LinkInfoMap::iterator itr = pose->ikLinkBegin(); itr != pose->ikLinkEnd(); ++itr){
                //   ss << "isBaseLink " << itr->second.isBaseLink() << endl;
                //   ss << "isTouching " << itr->second.isTouching() << endl;
                // }ss << endl;

                // ss << endl;
            }// 各poseに対するforの終わり
            poseSeqItems[i]->updateInterpolation();
        }// 各poseSeqに対するforの終わり


        // シグナル
        Signal<void()> sigTest;
        sigTest.connect(bind(&SandboxPlugin::test, this));
        sigTest();

        // スレッドでcollision取得->pointで表示
        // boost::thread thr(&SandboxPlugin::test, this);

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
            if(!item) return;
            cout << "Item: " << item->name() << endl;
            Archive archive;
            item->store(archive);
            double staticFriction,slipFriction;
            archive.read("staticFriction",staticFriction); archive.read("slipFriction",slipFriction);
            cout << "StaticFriction: " << staticFriction << "  SlipFriction: " << slipFriction << endl;
        }

        MessageView::instance()->putln(ss.str());
    }

    // collision取得
    void test()
    {
        cout << "test" << endl;
        BodyItemPtr bodyItemPtr; BodyPtr robot;
        PoseSeqItemPtr poseSeqItemPtr; PoseSeqPtr poseSeqPtr;
        BodyMotionItemPtr bodyMotionItemPtr; BodyMotionPtr motion;
        if(!getSelectedPoseSeqSet(bodyItemPtr, robot, poseSeqItemPtr, poseSeqPtr, bodyMotionItemPtr, motion)) return;
        WorldItemPtr worldItemPtr = bodyItemPtr->findOwnerItem<WorldItem>();
        const double dt = 1.0/motion->frameRate();

        PointSetItemPtr pointSetItemPtr;
        if(!(pointSetItemPtr = worldItemPtr->findItem<PointSetItem>("ContactPoint"))){
            pointSetItemPtr = new PointSetItem();
            pointSetItemPtr->setName("ContactPoint");
            worldItemPtr->addChildItem(pointSetItemPtr);
            ItemTreeView::mainInstance()->checkItem(pointSetItemPtr);// check + updateCollisions() -> segmentation fault
            cout << "Created ContactPoint" << endl;
        }

        connections = bodyItemPtr->sigCollisionsUpdated().connect(boost::bind(&SandboxPlugin::test2, this, bodyItemPtr, robot, motion, pointSetItemPtr));
        test2(bodyItemPtr, robot, motion, pointSetItemPtr);
    }

    void test2(BodyItemPtr& bodyItemPtr, BodyPtr& robot, BodyMotionPtr& motion, PointSetItemPtr& pointSetItemPtr)
    {
        static int idx = 0;
        static const double dt = 1.0/motion->frameRate();

        usleep(50*1000);

        if(idx != 0){
            pointSetItemPtr->clearAttentionPoint();
            const std::vector<CollisionLinkPairPtr>& collisions = bodyItemPtr->collisions();
            cout << dt*idx << " sec:";
            cout << collisions.size() << endl;
            for(size_t i=0; i < collisions.size(); ++i){
                CollisionLinkPair& collisionPair = *collisions[i];
                cout << " " << collisionPair.link[0]->name() << " " << collisionPair.link[1]->name() << " " << collisionPair.isSelfCollision();
                for(std::vector<Collision>::iterator iter = collisionPair.collisions.begin(); iter != collisionPair.collisions.end(); ++iter){
                    // cout << " " << iter->point.transpose() << " " << iter->normal.transpose() << endl;
                    pointSetItemPtr->addAttentionPoint(iter->point);
                }
            }cout << endl;
        }

        if(idx < motion->numFrames()){
            (BodyMotion::ConstFrame) motion->frame(idx) >> *robot;
            bodyItemPtr->notifyKinematicStateChange(true);
            ++idx;
        }else{
            idx = 0;
            connections.disconnect();// disconnect when finished
        }
    }

};

CNOID_IMPLEMENT_PLUGIN_ENTRY(SandboxPlugin)
