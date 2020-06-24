/**
   @author Kunio Kojima
*/
#include "PreviewControlPlugin.h"

using namespace boost;
using namespace cnoid;
using namespace std;

bool PreviewControlPlugin::initialize()
{
    mBar = new PreviewControlBar(this);
    addToolBar(mBar);

    return true;
}

// 球面からZ座標を計算
double PreviewControlPlugin::calcZFromSphere(const Vector3d centerPos, const Vector3d pos, const double radius)
{
    return centerPos.z() + sqrt( pow(radius,2) - pow(pos.x()-centerPos.x(), 2) - pow(pos.y()-centerPos.y(), 2) );
}

// 腰位置を可動域内に修正
void PreviewControlPlugin::modifyWaistIntoRange
(Vector3d& waistPos, const Vector3d lFootPos, const Vector3d rFootPos, const Vector3d lHipPos, const Vector3d rHipPos, const double legLength)
{
    // void modifyWaistIntoRange( Position waistPos, const Vector3d lFootPos, const Vector3d rFootPos, const Vector3d lHipPos, const Vector3d rHipPos, const double legLength ){
    // const double ratio = 1;
    const double ratio = 0.99;
    double z = waistPos.z();
    waistPos.z() = min( waistPos.z(),
                        min( waistPos.z() - lHipPos.z() + calcZFromSphere( lFootPos, lHipPos, legLength*ratio ),
                             waistPos.z() - rHipPos.z() + calcZFromSphere( rFootPos, rHipPos, legLength*ratio ) ) );
    // cout << " lhippos:" << lHipPos.z() << "  sphere z(l):" << calcZFromSphere( lFootPos, lHipPos, legLength*ratio ) << endl;
    // cout << " rhippos:" << rHipPos.z() << "  sphere z(r):" << calcZFromSphere( rFootPos, rHipPos, legLength*ratio ) << endl;
    if(waistPos.z() < z) cout << "down waist " << z-waistPos.z() << endl;
}

// 予見制御
void PreviewControlPlugin::execControl()
{
    cout << "\x1b[31m" << "Start Preview Control" << "\x1b[m" << endl;

    BodyItemPtr bodyItemPtr;
    BodyPtr body;
    PoseSeqItemPtr poseSeqItemPtr;
    PoseSeqPtr poseSeqPtr;
    BodyMotionItemPtr bodyMotionItemPtr;
    if(!getSelectedPoseSeqSet(bodyItemPtr, body, poseSeqItemPtr, poseSeqPtr, bodyMotionItemPtr)) return;
    BodyMotion& motion = *(bodyMotionItemPtr->motion());

    // 足先リンク名取得
    Link *lFootLink, *rFootLink;
    UtilPlugin::getFootLink( &lFootLink, &rFootLink, body );

    boost::filesystem::path logPath = getLogPath(boost::filesystem::path(poseSeqItemPtr->filePath()));

    Link* rootLink = body->rootLink();
    JointPath jpl, jpr;
    jpl = JointPath( rootLink, lFootLink );
    jpr = JointPath( rootLink, rFootLink );

    // 脚長さ計算
    for(int i = 0; i < jpl.numJoints(); ++i) jpl.joint(i)->q() =0;
    body->calcForwardKinematics();
    double legLength = ( jpl.joint(0)->p() - jpl.endLink()->p() ).norm();
    cout << "legLength: " << legLength << endl;

    // motion生成
    cout << "numFrames: " << motion.numFrames() << endl;
    if(motion.numFrames() == 0){
        generateBodyMotionFromBar(body, poseSeqItemPtr, bodyMotionItemPtr);
    }else{
        cout << "Need not generate motion" << endl;
    }

    double dt = 1.0/motion.frameRate(), max_tm = motion.numFrames() / (double)motion.frameRate();

    // 目標zmpを計算
    Vector3Seq& refZmpSeq = *(cnoid::getOrCreateZMPSeq(motion));
    cout << "Finished generating ref zmp seq" << endl;

    Vector3Seq initialZMPSeq;
    initialZMPSeq.setNumFrames(motion.numFrames(), true);
    calcZMP(body, motion, initialZMPSeq);// 入力動作のzmpの計算

    // 目標zmp・初期zmp・初期重心軌道書き出し
    {
        stringstream ss;
        ss << logPath.string() << "_PC";
        if(mBar->dialog->saveParameterInFileNameCheck.isChecked()) ss << mBar->dialog->getParamString();
        ss << "_" << motion.frameRate() << "fps.dat";
        ofstream ofs(ss.str().c_str());
        Vector3d lastPosVec = body->rootLink()->p();
        Vector3d lastVelVec = VectorXd::Zero(3);
        ofs << "time initZMPx initZMPy initZMPz refZMPx refZMPy refZMPz initCMx initCMy initCMz rootPosX rootPosY rootPosZ rootVelX rootVelY rootVelZ rootAccX rootAccY rootAccZ" << endl;
        for(int i = 0; i < motion.numFrames(); ++i){
            (BodyMotion::ConstFrame) motion.frame(i) >> *body;
            body->calcForwardKinematics();
            body->calcCenterOfMass();
      
            Vector3d velVec = (body->rootLink()->p() - lastPosVec)/dt;
            Vector3d accVec = (velVec - lastVelVec)/dt;
            lastPosVec = body->rootLink()->p(); lastVelVec = velVec;

            ofs << i*dt
                << " " << initialZMPSeq.at(i).transpose() << " " << refZmpSeq.at(i).transpose() << " " << body->centerOfMass().transpose()
                << " " << body->rootLink()->p().transpose() << " " << velVec.transpose() << " " << accVec.transpose() << endl;
        }
        ofs.close();
    }

    // 予見制御収束ループ
    Vector3Seq zmpSeq;
    string modeStr = mBar->dialog->controlModeCombo->currentText().toStdString();
    cout << "control mode: " << modeStr << endl;
    string dfMode = mBar->dialog->controlModeCombo->itemText(DynamicsFilter).toStdString();
    string tpMode = mBar->dialog->controlModeCombo->itemText(TrajectoryPlanning).toStdString();
    for(int loopNum = 0; loopNum < 3; ++loopNum){
        cout << "loop: " << loopNum << endl;

        zmpSeq.setNumFrames(motion.numFrames(), true);
        calcZMP(body, motion, zmpSeq , true);// 実際のzmpの計算

        // 予見制御用の実際のzmpと目標zmp、誤差zmp、時刻tmを計算
        std::queue<hrp::Vector3> ref_zmp_list;
        std::deque<double> tm_list;
        for(size_t i = 0; i < static_cast<size_t>(round(max_tm / dt)); ++i){
            // tm_list
            double tmp_tm = i * dt;
            tm_list.push_back(tmp_tm);
            // ref_zmp_list
            hrp::Vector3 ref_v;// 目標zmp
            ref_v << refZmpSeq.at(i).x(), refZmpSeq.at(i).y(), refZmpSeq.at(i).z();
        
            hrp::Vector3 v;// 実際のzmp
            v << zmpSeq.at(i).x(), zmpSeq.at(i).y(), zmpSeq.at(i).z();

            // 目標値リストに入力
            if(i+1 < static_cast<size_t>(round(max_tm / dt))){
                if(modeStr == dfMode){
                    ref_zmp_list.push(ref_v - v);
                }else if(modeStr == tpMode){
                    ref_zmp_list.push(ref_v);
                }else{
                    cout << "No such controle mode" << endl;
                    return;
                }
            }else{
                ref_zmp_list.push(ref_zmp_list.back());// 最後は1つ前の値と同じ
            }
        }
        cout << "Finished generating ref zmp list" << endl;

        Vector3Seq inputZMPSeq;
        inputZMPSeq.setNumFrames(motion.numFrames(), true);

        Vector3Seq outputCMSeq;
        outputCMSeq.setNumFrames(motion.numFrames(), true);

        Vector3Seq outputZMPSeq;
        outputZMPSeq.setNumFrames(motion.numFrames(), true);

        (BodyMotion::ConstFrame) motion.frame(0) >> *body;
        body->calcForwardKinematics();
        Vector3d CM = body->calcCenterOfMass();
        rats2::preview_dynamics_filter<rats2::extended_preview_control> df(dt, CM.z(), ref_zmp_list.front());
        double output_zmp[3], input_zmp[3];
        bool r = true;
        size_t index = 0;
        while (r) {
            hrp::Vector3 p, x;// p: current_refzmp   x: refcog
            std::vector<hrp::Vector3> qdata;
            r = df.update(p, x, qdata, ref_zmp_list.front(), qdata, !ref_zmp_list.empty());
            if (r) {
                df.get_cart_zmp(output_zmp);
                df.get_current_refzmp(input_zmp);

                outputCMSeq.at(index) = x;

                inputZMPSeq.at(index) << input_zmp[0], input_zmp[1], input_zmp[2];
                outputZMPSeq.at(index) << output_zmp[0], output_zmp[1], output_zmp[2];

                ++index;
            } else if ( !ref_zmp_list.empty() ) r = true;
            if (!ref_zmp_list.empty()){
                ref_zmp_list.pop();
            }
        }
        cout << "Finished calculating ref diff centroid" << endl;

        for(int i = 0; i < motion.numFrames(); ++i){
            Vector3d lFootPos,rFootPos, lHipPos, rHipPos, refCM;
            Matrix3 lFootR,rFootR;
            (BodyMotion::ConstFrame) motion.frame(i) >> *body;
            body->calcForwardKinematics();// 状態更新

            lFootPos = lFootLink->p(); lFootR = lFootLink->R();// 足先位置取得
            rFootPos = rFootLink->p(); rFootR = rFootLink->R();

            if(modeStr == dfMode){
                Vector3d diffCM = outputCMSeq.at(i);
                diffCM.z() = 0;
                // body->rootLink()->p() += 0.5*diffCM;// 腰位置修正 ゲインを掛けるだけでは微妙
                body->rootLink()->p() += 1.0*diffCM;// for HRP2 and JAXON_BLUE
            }else if(modeStr == tpMode){
                Vector3d CM =  outputCMSeq.at(i);
                Vector3d CM2RootLink =  body->rootLink()->p() - body->calcCenterOfMass();
                CM2RootLink.z() = 0;
                CM.z() = body->rootLink()->p().z();
                body->rootLink()->p() = CM + CM2RootLink;
            }
            body->calcForwardKinematics();// 状態更新
            lHipPos = jpl.joint(0)->p(); rHipPos = jpr.joint(0)->p();

            Vector3 tmpVec = body->rootLink()->p();
            modifyWaistIntoRange( tmpVec, lFootPos, rFootPos, lHipPos, rHipPos, legLength );
            body->rootLink()->p() = tmpVec;// 腰位置修正は要改良
            body->calcForwardKinematics();// 状態更新

            if(!jpl.calcInverseKinematics(lFootPos,lFootR)) cout << "\x1b[31m" << i*dt << " lfoot IK fail" << "\x1b[m" << endl;
            if(!jpr.calcInverseKinematics(rFootPos,rFootR)) cout << "\x1b[31m" << i*dt << " rfoot IK fail" << "\x1b[m" << endl;

            motion.frame(i) << *body;
        }
        cout << "Finished modifying waist position" << endl;

        // zmpファイル書き出し
        {
            stringstream ss;
            ss << logPath.string() << "_PC";
            if(mBar->dialog->saveParameterInFileNameCheck.isChecked()) ss << mBar->dialog->getParamString();
            ss << "_" << motion.frameRate() << "fps_" << loopNum << ".dat";
            ofstream ofs;
            ofs.open(ss.str().c_str());

            calcZMP(body, motion, zmpSeq, true);
            ofs << "time  inputZMPx inputZMPy inputZMPz outputZMPx outputZMPy outputZMPz outputCMx outputCMy outputCMz actZMPx actZMPy actZMPz actCMx actCMy actCMz rootPosX rootPosY rootPosZ" << endl;
            for(int i = 0; i < motion.numFrames(); ++i){
                (BodyMotion::ConstFrame) motion.frame(i) >> *body;
                body->calcForwardKinematics();
                body->calcCenterOfMass();
                ofs << i*dt
                    << " " << inputZMPSeq.at(i).transpose() << " " << outputZMPSeq.at(i).transpose() << " " << outputCMSeq.at(i).transpose()
                    << " " << zmpSeq.at(i).transpose() << " " << body->centerOfMass().transpose() << " " << body->rootLink()->p().transpose() << endl;
            }
            ofs.close();
        }
        cout << "Finished recording ZMP trajectory" << endl;

    }// modifying loop

    // motionに出力動作ZMPを代入
    Vector3Seq& finalZMPSeq = *(getOrCreateZMPSeq(motion));// motionのZMP
    for(int i = 0; i < motion.numFrames(); ++i){
        finalZMPSeq.at(i) = zmpSeq.at(i);
    }

    cout << "Finished assigning ZMP to motion class" << endl;

    cout << "\x1b[31m" << "Finished Preview Control" << "\x1b[m" << endl << endl;
}

CNOID_IMPLEMENT_PLUGIN_ENTRY(PreviewControlPlugin)
