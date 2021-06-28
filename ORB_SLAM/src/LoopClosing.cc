/**
* This file is part of ORB-SLAM.
*
* Copyright (C) 2014 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <http://webdiis.unizar.es/~raulmur/orbslam/>
*
* ORB-SLAM is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM. If not, see <http://www.gnu.org/licenses/>.
*/

#include "LoopClosing.h"

#include "Sim3Solver.h"

#include "Converter.h"

#include "Optimizer.h"

#include "ORBmatcher.h"

#include <ros/ros.h>

#include "Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"

namespace ORB_SLAM
{

LoopClosing::LoopClosing(Map *pMap, KeyFrameDatabase *pDB, ORBVocabulary *pVoc):
    mbResetRequested(false), mpMap(pMap), mpKeyFrameDB(pDB), mpORBVocabulary(pVoc), mLastLoopKFid(0)
{
    mnCovisibilityConsistencyTh = 3;
    mpMatchedKF = NULL;
}

void LoopClosing::SetTracker(Tracking *pTracker)
{
    mpTracker=pTracker;
}

void LoopClosing::SetLocalMapper(LocalMapping *pLocalMapper)
{
    mpLocalMapper=pLocalMapper;
}


void LoopClosing::Run()
{

    ros::Rate r(200);

    while(ros::ok())
    {
        // Check if there are keyframes in the queue
        if(CheckNewKeyFrames())
        {
            // Detect loop candidates and check covisibility consistency
            if(DetectLoop())
            {
               // Compute similarity transformation [sR|t]
               if(ComputeSim3())
               {
                   // Perform loop fusion and pose graph optimization
                   CorrectLoop();
               }
            }
        }

        ResetIfRequested();
        r.sleep();
    }
}

void LoopClosing::InsertKeyFrame(KeyFrame *pKF)
{
    boost::mutex::scoped_lock lock(mMutexLoopQueue);
    if(pKF->mnId!=0)
        mlpLoopKeyFrameQueue.push_back(pKF);
}

bool LoopClosing::CheckNewKeyFrames()
{
    boost::mutex::scoped_lock lock(mMutexLoopQueue);
    return(!mlpLoopKeyFrameQueue.empty());
}


/*******************************************************************************
 *    功能:检测是否产生了回环
 *        检测回环的步骤:
 *           1  检测上次回环发生是否离当前关键帧足够长时间    并且满足当前关键帧总数量大于10
 *           2  找出当前关键帧的共视关键帧,并找出其中的最小得分
 *           3  根据最小得分寻找回环候选帧   具体见函数DetectLoopCandidates
 *           4  在候选回环关键帧中寻找具有连续性的关键帧

 *              这里将候选回环关键帧和他的共视关键帧组成一个候选组
 *              一个组和另一个组是连续的,是指他们至少存在一个共视关键帧

 *              如果两个组之间存在足够多的帧是共视关键帧,则证明两个组之间是完全连续组,则说明发生了回环
 *              候选关键帧需要进行连续性检验的原因: 
 *              我们通过聚类相连候选帧,可以将一些得分很高但却相对独立的枕给去掉这些帧与其他帧相对没有关联，而我们知道事实上回环处会
 *              有一个时间和空间上的连续性，因此对于正确的回环来讲，这些相似性评分较高的帧是错误关键帧。
 * 
 ******************************************************************************/
bool LoopClosing::DetectLoop()
{
    {
        boost::mutex::scoped_lock lock(mMutexLoopQueue);
        mpCurrentKF = mlpLoopKeyFrameQueue.front();
        mlpLoopKeyFrameQueue.pop_front();
        // Avoid that a keyframe can be erased while it is being process by this thread
        // 设置当前关键帧不可被擦除(防止进程运行过程中,当前关键帧被擦除),当检测完回环之后重新设为可被擦除
        mpCurrentKF->SetNotErase();
    }

    //If the map contains less than 10 KF or less than 10KF have passed from last loop detection
    //如果刚刚发生了回环   上次回环之后通过的关键帧帧数不超过10  则将该关键帧添加到关键帧集中  将该关键帧设为可擦除关键帧 
    // 或者map中关键帧总共还没有10帧，则不进行闭环检测
    if(mpCurrentKF->mnId<mLastLoopKFid+10)
    {
        mpKeyFrameDB->add(mpCurrentKF);
        mpCurrentKF->SetErase();
        return false;
    }

    // Compute reference BoW similarity score
    // This is the lowest score to a connected keyframe in the covisibility graph
    // We will impose loop candidates to have a higher similarity than this
    // 当前关键帧的共视关键帧
    vector<KeyFrame*> vpConnectedKeyFrames = mpCurrentKF->GetVectorCovisibleKeyFrames();
    // 得到当前关键帧的BOW向量
    DBoW2::BowVector CurrentBowVec = mpCurrentKF->GetBowVector();
    float minScore = 1;
    // 循环每个共视关键帧  计算每个共视关键帧与当前待检测回环关键帧之间的BOW得分  并得到其中最小的得分
    for(size_t i=0; i<vpConnectedKeyFrames.size(); i++)
    {
        KeyFrame* pKF = vpConnectedKeyFrames[i];
        if(pKF->isBad())
            continue;
        // 共视关键帧的BOW向量
        DBoW2::BowVector BowVec = pKF->GetBowVector();
        //得到共视关键帧和当前关键帧的BOW向量得分
        float score = mpORBVocabulary->score(CurrentBowVec, BowVec);

        if(score<minScore) //得到最小的得分
            minScore = score;
    }

    // Query the database imposing the minimum score
    // 在关键帧数据集中查找当前关键帧的回环候选关键帧  最小得分大于minScore 
    vector<KeyFrame*> vpCandidateKFs = mpKeyFrameDB->DetectLoopCandidates(mpCurrentKF, minScore);


    // If there are no loop candidates, just add new keyframe and return false
    if(vpCandidateKFs.empty())
    {
        mpKeyFrameDB->add(mpCurrentKF);
        mvConsistentGroups.clear();
        mpCurrentKF->SetErase();
        return false;
    }

    // For each loop candidate check consistency with previous loop candidates
    // Each candidate expands a covisibility group (keyframes connected to the loop candidate in the covisibility graph)
    // A group is consistent with a previous group if they share at least a keyframe
    // We must detect a consistent loop in several consecutive keyframe to accept it
    // 步骤4：在候选帧中检测具有连续性的候选帧
    // 1、每个候选帧将与自己相连的关键帧构成一个“子候选组spCandidateGroup”，vpCandidateKFs-->spCandidateGroup
    // 2、检测“子候选组”中每一个关键帧是否存在于“连续组”，如果存在nCurrentConsistency++，则将该“子候选组”放入“当前连续组vCurrentConsistentGroups”
    // 3、如果nCurrentConsistency大于等于3，那么该”子候选组“代表的候选帧过关，进入mvpEnoughConsistentCandidates
    //筛选后得到的具有连续性的候选帧
    //  候选关键帧需要进行连续性检验的原因: 
    // 我们通过聚类相连候选帧,可以将一些得分很高但却相对独立的枕给去掉这些帧与其他帧相对没有关联，而我们知道事实上回环处会
    // 有一个时间和空间上的连续性，因此对于正确的回环来讲，这些相似性评分较高的帧是错误关键帧。
    mvpEnoughConsistentCandidates.clear();
    // ConsistentGroup数据类型为pair<set<KeyFrame*>,int>
    // ConsistentGroup.first对应每个“连续组”中的关键帧，ConsistentGroup.second为当前该连续组与其他连续组之间连续的连续组数量
    // 当前的连续组   连续的候选回环帧组
    vector<ConsistentGroup> vCurrentConsistentGroups;
    vector<bool> vbConsistentGroup(mvConsistentGroups.size(),false);

    //每一个候选回环帧
    for(size_t i=0, iend=vpCandidateKFs.size(); i<iend; i++)
    {
        KeyFrame* pCandidateKF = vpCandidateKFs[i];
        // 候选关键帧的共视关键帧以及候选关键帧构成了"子候选组"
        set<KeyFrame*> spCandidateGroup = pCandidateKF->GetConnectedKeyFrames();
        spCandidateGroup.insert(pCandidateKF);

        bool bEnoughConsistent = false;
        bool bConsistentForSomeGroup = false;
        //遍历之前的"子连续组"
        for(size_t iG=0, iendG=mvConsistentGroups.size(); iG<iendG; iG++)
        {
            //之前的子连续组
            set<KeyFrame*> sPreviousGroup = mvConsistentGroups[iG].first;

            bool bConsistent = false;
            //子连续组中的每一帧
            for(set<KeyFrame*>::iterator sit=spCandidateGroup.begin(), send=spCandidateGroup.end(); sit!=send;sit++)
            {
                //如果之前子连续组中包含"子候选组"中的帧,则说明该关键帧组与之前的组是连续的
                if(sPreviousGroup.count(*sit))
                {
                    bConsistent=true;
                    bConsistentForSomeGroup=true;
                    break;
                }
            }

            // 如果与之前的连续组是连续的  则将它加入到当前连续组中
            if(bConsistent)
            {
                int nPreviousConsistency = mvConsistentGroups[iG].second;
                int nCurrentConsistency = nPreviousConsistency + 1;
                // 如果当前连续组没有在当前连续组集中,则将其加入
                if(!vbConsistentGroup[iG])
                {
                    ConsistentGroup cg = make_pair(spCandidateGroup,nCurrentConsistency);
                    vCurrentConsistentGroups.push_back(cg);//当前连续组
                    vbConsistentGroup[iG]=true; //this avoid to include the same group more than once
                }
                // 如果与当前连续组连续的其他连续组之间连续数量大于某一阈值,则说明他有足够多的连续组,将其加入足够连续组集
                if(nCurrentConsistency>=mnCovisibilityConsistencyTh && !bEnoughConsistent)
                {
                    mvpEnoughConsistentCandidates.push_back(pCandidateKF);
                    bEnoughConsistent=true; //this avoid to insert the same candidate more than once
                }
            }
        }

        // If the group is not consistent with any previous group insert with consistency counter set to zero
        if(!bConsistentForSomeGroup)
        {
            ConsistentGroup cg = make_pair(spCandidateGroup,0);
            vCurrentConsistentGroups.push_back(cg);
        }
    }

    // Update Covisibility Consistent Groups 更新连续组
    mvConsistentGroups = vCurrentConsistentGroups;


    // Add Current Keyframe to database 添加关键帧到关键帧数据库中
    mpKeyFrameDB->add(mpCurrentKF);

    // 如果足够连续的候选组为空则将返回false,如果存在足够连续候选组,则证明发生回环
    if(mvpEnoughConsistentCandidates.empty())
    {
        mpCurrentKF->SetErase();
        return false;
    }
    else
    {
        return true;
    }

    mpCurrentKF->SetErase();// 设置当前关键帧可以被擦除,与刚进行检测时形成呼应
    return false;
}

bool LoopClosing::ComputeSim3()
{
    // For each consistent loop candidate we try to compute a Sim3

    const int nInitialCandidates = mvpEnoughConsistentCandidates.size();

    // We compute first ORB matches for each candidate
    // If enough matches are found, we setup a Sim3Solver
    ORBmatcher matcher(0.75,true);

    vector<Sim3Solver*> vpSim3Solvers;
    vpSim3Solvers.resize(nInitialCandidates);

    vector<vector<MapPoint*> > vvpMapPointMatches;
    vvpMapPointMatches.resize(nInitialCandidates);

    vector<bool> vbDiscarded;
    vbDiscarded.resize(nInitialCandidates);

    int nCandidates=0; //candidates with enough matches

    // 计算当前关键帧与之前获取到的候选回环帧匹配集合及构建Sim3Solver求解器
    for(int i=0; i<nInitialCandidates; i++)
    {
        KeyFrame* pKF = mvpEnoughConsistentCandidates[i];

        // avoid that local mapping erase it while it is being processed in this thread
        pKF->SetNotErase();

        if(pKF->isBad())
        {
            vbDiscarded[i] = true;
            continue;
        }

        int nmatches = matcher.SearchByBoW(mpCurrentKF,pKF,vvpMapPointMatches[i]);

        if(nmatches<20)
        {
            vbDiscarded[i] = true;
            continue;
        }
        else
        {
            Sim3Solver* pSolver = new Sim3Solver(mpCurrentKF,pKF,vvpMapPointMatches[i]);
            pSolver->SetRansacParameters(0.99,20,300);
            vpSim3Solvers[i] = pSolver;
        }

        nCandidates++;
    }

    bool bMatch = false;

    // Perform alternatively RANSAC iterations for each candidate
    // until one is succesful or all fail
    while(nCandidates>0 && !bMatch)
    {
        for(int i=0; i<nInitialCandidates; i++)
        {
            if(vbDiscarded[i])
                continue;

            KeyFrame* pKF = mvpEnoughConsistentCandidates[i];

            // Perform 5 Ransac Iterations
            vector<bool> vbInliers;
            int nInliers;
            bool bNoMore;

            Sim3Solver* pSolver = vpSim3Solvers[i];
            cv::Mat Scm  = pSolver->iterate(5,bNoMore,vbInliers,nInliers);

            // If Ransac reachs max. iterations discard keyframe
            if(bNoMore)
            {
                vbDiscarded[i]=true;
                nCandidates--;
            }

            // If RANSAC returns a Sim3, perform a guided matching and optimize with all correspondences
            // sim3之后继续优化求解
            if(!Scm.empty())
            {
                vector<MapPoint*> vpMapPointMatches(vvpMapPointMatches[i].size(), static_cast<MapPoint*>(NULL));
                for(size_t j=0, jend=vbInliers.size(); j<jend; j++)
                {
                    if(vbInliers[j])
                       vpMapPointMatches[j]=vvpMapPointMatches[i][j];
                }

                cv::Mat R = pSolver->GetEstimatedRotation();
                cv::Mat t = pSolver->GetEstimatedTranslation();
                const float s = pSolver->GetEstimatedScale();
                matcher.SearchBySim3(mpCurrentKF,pKF,vpMapPointMatches,s,R,t,7.5);


                g2o::Sim3 gScm(Converter::toMatrix3d(R),Converter::toVector3d(t),s);
                const int nInliers = Optimizer::OptimizeSim3(mpCurrentKF, pKF, vpMapPointMatches, gScm, 10);

                // If optimization is succesful stop ransacs and continue
                if(nInliers>=20)
                {
                    bMatch = true;
                    mpMatchedKF = pKF;
                    g2o::Sim3 gSmw(Converter::toMatrix3d(pKF->GetRotation()),Converter::toVector3d(pKF->GetTranslation()),1.0);
                    mg2oScw = gScm*gSmw;
                    mScw = Converter::toCvMat(mg2oScw);

                    mvpCurrentMatchedPoints = vpMapPointMatches;
                    break;
                }
            }
        }
    }

    if(!bMatch)
    {
        for(int i=0; i<nInitialCandidates; i++)
             mvpEnoughConsistentCandidates[i]->SetErase();
        mpCurrentKF->SetErase();
        return false;
    }

    // Retrieve MapPoints seen in Loop Keyframe and neighbors
    // 获取最佳匹配KF的共视图，并获取共视图所有关键帧所有的地图点集mvpLoopMapPoints
    vector<KeyFrame*> vpLoopConnectedKFs = mpMatchedKF->GetVectorCovisibleKeyFrames();
    vpLoopConnectedKFs.push_back(mpMatchedKF);
    mvpLoopMapPoints.clear();
    for(vector<KeyFrame*>::iterator vit=vpLoopConnectedKFs.begin(); vit!=vpLoopConnectedKFs.end(); vit++)
    {
        KeyFrame* pKF = *vit;
        vector<MapPoint*> vpMapPoints = pKF->GetMapPointMatches();
        for(size_t i=0, iend=vpMapPoints.size(); i<iend; i++)
        {
            MapPoint* pMP = vpMapPoints[i];
            if(pMP)
            {
                if(!pMP->isBad() && pMP->mnLoopPointForKF!=mpCurrentKF->mnId)
                {
                    mvpLoopMapPoints.push_back(pMP);
                    pMP->mnLoopPointForKF=mpCurrentKF->mnId;
                }
            }
        }
    }

    // Find more matches projecting with the computed Sim3
    // 根据地图点集mvpLoopMapPoints，继续搜索更多的匹配
    matcher.SearchByProjection(mpCurrentKF, mScw, mvpLoopMapPoints, mvpCurrentMatchedPoints,10);

    // If enough matches accept Loop
    int nTotalMatches = 0;
    for(size_t i=0; i<mvpCurrentMatchedPoints.size(); i++)
    {
        if(mvpCurrentMatchedPoints[i])
            nTotalMatches++;
    }

    if(nTotalMatches>=40)
    {
        for(int i=0; i<nInitialCandidates; i++)
            if(mvpEnoughConsistentCandidates[i]!=mpMatchedKF)
                mvpEnoughConsistentCandidates[i]->SetErase();
        return true;
    }
    else
    {
        for(int i=0; i<nInitialCandidates; i++)
            mvpEnoughConsistentCandidates[i]->SetErase();
        mpCurrentKF->SetErase();
        return false;
    }

}


/******************************************
 *  根据回环进行位姿矫正
 *      1. 请求局部地图线程停止,并且中止现有的全局优化进程
 *      2. 根据当前帧求得的相机位姿(相似变换矩阵)来求解矫正前和矫正后的相邻帧位姿变换矩阵(相似变换矩阵)
 *      3. 将相邻关键帧的所有地图点都根据更新后的相机位姿(相似变换矩阵)重新计算地图点世界坐标  
 *      4. 进行地图点融合   将之前匹配的(在ComputeSim3()函数中计算局部地图点和当前帧的匹配)两地图点融合为同一地图点
 *      5. 根据第3步中计算的地图点重新进行匹配,并融合匹配点和当前关键帧中的地图点
 *      6. 在地图点融合之后,更新当前关键帧的共视图中各个关键帧的相连关键帧,更新连接之后,将这些相邻关键帧全部加入LoopConnections容器
 *      7. 根据四种边(1 新检测到的回环边  2 父关键帧与子关键帧的边 3 历史回环关键帧  4 共视图边)对全局地图中的所有关键帧的位姿进行矫正
 *      8. 根据地图点和关键帧位姿计算重投影误差对全局地图进行优化
 *************************************/
void LoopClosing::CorrectLoop()
{
    // Send a stop signal to Local Mapping
    // Avoid new keyframes are inserted while correcting the loop
    mpLocalMapper->RequestStop();

    // Wait until Local Mapping has effectively stopped
    ros::Rate r(1e4);
    // 等待局部地图线程已经完全停止
    while(ros::ok() && !mpLocalMapper->isStopped())
    {
        r.sleep();
    }

    // Ensure current keyframe is updated
    // 确保当前关键帧不需要进行更新 
    mpCurrentKF->UpdateConnections();

    // Retrive keyframes connected to the current keyframe and compute corrected Sim3 pose by propagation
    // 当前帧及当前帧共视图集合
    mvpCurrentConnectedKFs = mpCurrentKF->GetVectorCovisibleKeyFrames();
    mvpCurrentConnectedKFs.push_back(mpCurrentKF);
    // 矫正后的相机位姿    /    未矫正的相机位姿   （该帧   位姿）
    KeyFrameAndPose CorrectedSim3, NonCorrectedSim3;
    CorrectedSim3[mpCurrentKF]=mg2oScw;// 当前帧对应的矫正位姿
    cv::Mat Twc = mpCurrentKF->GetPoseInverse(); // 这里应该是未矫正当前帧的位姿

    // 计算矫正后和矫正前的相邻关键帧的相机位姿(相似变换矩阵)      待优化之前的当前帧的共视帧
    for(vector<KeyFrame*>::iterator vit=mvpCurrentConnectedKFs.begin(), vend=mvpCurrentConnectedKFs.end(); vit!=vend; vit++)
    {
        KeyFrame* pKFi = *vit;

        cv::Mat Tiw = pKFi->GetPose();

        if(pKFi!=mpCurrentKF)// 将当前帧的相邻关键帧的相机位姿都根据当前帧的相似变换矩阵进行矫正  sim3矫正
        {            
            cv::Mat Tic = Tiw*Twc;//c是当前帧  i是共视帧
            cv::Mat Ric = Tic.rowRange(0,3).colRange(0,3);
            cv::Mat tic = Tic.rowRange(0,3).col(3);
            g2o::Sim3 g2oSic(Converter::toMatrix3d(Ric),Converter::toVector3d(tic),1.0);
            //待回环共视帧 对 世界坐标系的相对位姿 （正确位置 sim3）
            g2o::Sim3 g2oCorrectedSiw = g2oSic*mg2oScw;
            //Pose corrected with the Sim3 of the loop closure
            //依靠待回环帧的共视帧与 待回环帧的相对位姿 乘 sim3修正  位姿修正成功
            CorrectedSim3[pKFi]=g2oCorrectedSiw;
        }

        //没有进行sim3相乘
        cv::Mat Riw = Tiw.rowRange(0,3).colRange(0,3);
        cv::Mat tiw = Tiw.rowRange(0,3).col(3);
        g2o::Sim3 g2oSiw(Converter::toMatrix3d(Riw),Converter::toVector3d(tiw),1.0);
        //Pose without correction
        NonCorrectedSim3[pKFi]=g2oSiw;
    }

    // Correct all MapPoints obsrved by current keyframe and neighbors, so that they align with the other side of the loop
    // 将相邻关键帧的所有地图点都根据更新后的相机位姿(相似变换矩阵)重新计算地图点世界坐标  
    for(KeyFrameAndPose::iterator mit=CorrectedSim3.begin(), mend=CorrectedSim3.end(); mit!=mend; mit++)
    {
        KeyFrame* pKFi = mit->first;
        g2o::Sim3 g2oCorrectedSiw = mit->second;
        g2o::Sim3 g2oCorrectedSwi = g2oCorrectedSiw.inverse();
        //无 sim3  待回环共视帧 在世界坐标系位置（漂移的世界坐标系）
        g2o::Sim3 g2oSiw =NonCorrectedSim3[pKFi];

        vector<MapPoint*> vpMPsi = pKFi->GetMapPointMatches();
        //进行了 地图点 对于世界坐标系的修正
        for(size_t iMP=0, endMPi = vpMPsi.size(); iMP<endMPi; iMP++)
        {
            MapPoint* pMPi = vpMPsi[iMP];
            if(!pMPi)
                continue;
            if(pMPi->isBad())
                continue;
            if(pMPi->mnCorrectedByKF==mpCurrentKF->mnId)
                continue;

            // Project with non-corrected pose and project back with corrected pose
            //未sim3  待回环共视帧kf1 中的 地图点i 在世界坐标系位置（漂移的世界坐标系）
            cv::Mat P3Dw = pMPi->GetWorldPos();
            Eigen::Matrix<double,3,1> eigP3Dw = Converter::toVector3d(P3Dw);
            // 依靠待回环帧共视帧地图点 与 待回环帧共视帧的关系 恢复待回环帧共视帧地图点
            Eigen::Matrix<double,3,1> eigCorrectedP3Dw = g2oCorrectedSwi.map(g2oSiw.map(eigP3Dw));

            cv::Mat cvCorrectedP3Dw = Converter::toCvMat(eigCorrectedP3Dw);
            pMPi->SetWorldPos(cvCorrectedP3Dw);
            pMPi->mnCorrectedByKF = mpCurrentKF->mnId;
            pMPi->mnCorrectedReference = pKFi->mnId;
            pMPi->UpdateNormalAndDepth();
        }

        // Update keyframe pose with corrected Sim3. First transform Sim3 to SE3 (scale translation)
        Eigen::Matrix3d eigR = g2oCorrectedSiw.rotation().toRotationMatrix();
        Eigen::Vector3d eigt = g2oCorrectedSiw.translation();
        double s = g2oCorrectedSiw.scale();

        eigt *=(1./s); //[R t/s;0 1]

        cv::Mat correctedTiw = Converter::toCvSE3(eigR,eigt);

        //待回环点共视帧 对于位姿的修正
        pKFi->SetPose(correctedTiw);

        // Make sure connections are updated
        pKFi->UpdateConnections();
    }    

    // Start Loop Fusion
    // Update matched map points and replace if duplicated
    // 进行地图点融合   将匹配的两地图点融合为同一地图点    对当前帧 的 地图点修正后融合 添加到地图
    for(size_t i=0; i<mvpCurrentMatchedPoints.size(); i++)
    {
        if(mvpCurrentMatchedPoints[i])
        {
            MapPoint* pLoopMP = mvpCurrentMatchedPoints[i];
            MapPoint* pCurMP = mpCurrentKF->GetMapPoint(i);
            if(pCurMP)
                pCurMP->Replace(pLoopMP);
            else
            {
                mpCurrentKF->AddMapPoint(pLoopMP,i);
                pLoopMP->AddObservation(mpCurrentKF,i);
                pLoopMP->ComputeDistinctiveDescriptors();
            }
        }
    }

    // Project MapPoints observed in the neighborhood of the loop keyframe
    // into the current keyframe and neighbors using corrected poses.
    // Fuse duplications.
    SearchAndFuse(CorrectedSim3);


    // After the MapPoint fusion, new links in the covisibility graph will appear attaching both sides of the loop
    map<KeyFrame*, set<KeyFrame*> > LoopConnections;
    // 在地图点融合之后,更新当前关键帧的共视图中各个关键帧的相连关键帧,更新连接之后,将这些相邻关键帧全部加入LoopConnections容器
    for(vector<KeyFrame*>::iterator vit=mvpCurrentConnectedKFs.begin(), vend=mvpCurrentConnectedKFs.end(); vit!=vend; vit++)
    {
        KeyFrame* pKFi = *vit;
        // 返回共视连接关键帧  未进行重新的共视地图点关系计算
        vector<KeyFrame*> vpPreviousNeighbors = pKFi->GetVectorCovisibleKeyFrames();

        // Update connections. Detect new links.
        // 更新链接,将当前帧的关联关键帧的关联关键帧加入LoopConnections容器,不包括直接相邻的和当前帧的关联关键帧
        // 实际就是一系列回环的集合,保证回环的连续性
        pKFi->UpdateConnections();
        // 获取pKFi的关联关键帧
        LoopConnections[pKFi]=pKFi->GetConnectedKeyFrames();
        //去除LoopConnections[pKFi]中与pKFi直接相连的关键帧
        for(vector<KeyFrame*>::iterator vit_prev=vpPreviousNeighbors.begin(), vend_prev=vpPreviousNeighbors.end(); vit_prev!=vend_prev; vit_prev++)
        {
            LoopConnections[pKFi].erase(*vit_prev);
        }
        //去除LoopConnections[pKFi]中与当前帧直接相连的关键帧
        for(vector<KeyFrame*>::iterator vit2=mvpCurrentConnectedKFs.begin(), vend2=mvpCurrentConnectedKFs.end(); vit2!=vend2; vit2++)
        {
            LoopConnections[pKFi].erase(*vit2);
        }
    }

    // 为什么这里要强制执行重定位
    mpTracker->ForceRelocalisation();
    // 进行本质图优化
    Optimizer::OptimizeEssentialGraph(mpMap, mpMatchedKF, mpCurrentKF,  mg2oScw, NonCorrectedSim3, CorrectedSim3, LoopConnections);

    //Add edge
    mpMatchedKF->AddLoopEdge(mpCurrentKF);
    mpCurrentKF->AddLoopEdge(mpMatchedKF);

    ROS_INFO("Loop Closed!");

    // Loop closed. Release Local Mapping.
    mpLocalMapper->Release();

    mpMap->SetFlagAfterBA();

    mLastLoopKFid = mpCurrentKF->mnId;
}

void LoopClosing::SearchAndFuse(KeyFrameAndPose &CorrectedPosesMap)
{
    ORBmatcher matcher(0.8);

    for(KeyFrameAndPose::iterator mit=CorrectedPosesMap.begin(), mend=CorrectedPosesMap.end(); mit!=mend;mit++)
    {
        KeyFrame* pKF = mit->first;

        g2o::Sim3 g2oScw = mit->second;
        cv::Mat cvScw = Converter::toCvMat(g2oScw);

        matcher.Fuse(pKF,cvScw,mvpLoopMapPoints,4);
    }
}


void LoopClosing::RequestReset()
{
    {
        boost::mutex::scoped_lock lock(mMutexReset);
        mbResetRequested = true;
    }

    ros::Rate r(500);
    while(ros::ok())
    {
        {
        boost::mutex::scoped_lock lock2(mMutexReset);
        if(!mbResetRequested)
            break;
        }
        r.sleep();
    }
}

void LoopClosing::ResetIfRequested()
{
    boost::mutex::scoped_lock lock(mMutexReset);
    if(mbResetRequested)
    {
        mlpLoopKeyFrameQueue.clear();
        mLastLoopKFid=0;
        mbResetRequested=false;
    }
}

} //namespace ORB_SLAM
