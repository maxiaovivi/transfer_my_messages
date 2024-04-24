#include "include/CLoop.h"

namespace LIOSAM
{

CLoop::CLoop()
{
    std::cout << __FUNCTION__ << " start" << std::endl;
}

CLoop::~CLoop()
{

}

// void CLoop::performLoopClosure()
// {
//     if (cloudKeyPoses3D->points.empty() == true)
//         return;

//     mtx.lock();
//     *copy_cloudKeyPoses3D = *cloudKeyPoses3D;
//     *copy_cloudKeyPoses6D = *cloudKeyPoses6D;
//     mtx.unlock();

//     // find keys
//     int loopKeyCur;
//     int loopKeyPre;
//     if (detectLoopClosureExternal(&loopKeyCur, &loopKeyPre) == false)
//         if (detectLoopClosureDistance(&loopKeyCur, &loopKeyPre) == false)
//             return;

//     // extract cloud
//     pcl::PointCloud<PointType>::Ptr cureKeyframeCloud(new pcl::PointCloud<PointType>());
//     pcl::PointCloud<PointType>::Ptr prevKeyframeCloud(new pcl::PointCloud<PointType>());
//     {
//         loopFindNearKeyframes(cureKeyframeCloud, loopKeyCur, 0);
//         loopFindNearKeyframes(prevKeyframeCloud, loopKeyPre, historyKeyframeSearchNum);
//         if (cureKeyframeCloud->size() < 300 || prevKeyframeCloud->size() < 1000)
//             return;
//         if (pubHistoryKeyFrames.getNumSubscribers() != 0)
//             publishCloud(&pubHistoryKeyFrames, prevKeyframeCloud, timeLaserInfoStamp, odometryFrame);
//     }

//     // ICP Settings
//     static pcl::IterativeClosestPoint<PointType, PointType> icp;
//     icp.setMaxCorrespondenceDistance(historyKeyframeSearchRadius*2);
//     icp.setMaximumIterations(100);
//     icp.setTransformationEpsilon(1e-6);
//     icp.setEuclideanFitnessEpsilon(1e-6);
//     icp.setRANSACIterations(0);

//     // Align clouds
//     icp.setInputSource(cureKeyframeCloud);
//     icp.setInputTarget(prevKeyframeCloud);
//     pcl::PointCloud<PointType>::Ptr unused_result(new pcl::PointCloud<PointType>());
//     icp.align(*unused_result);

//     if (icp.hasConverged() == false || icp.getFitnessScore() > historyKeyframeFitnessScore)
//         return;

//     // publish corrected cloud
//     if (pubIcpKeyFrames.getNumSubscribers() != 0)
//     {
//         pcl::PointCloud<PointType>::Ptr closed_cloud(new pcl::PointCloud<PointType>());
//         pcl::transformPointCloud(*cureKeyframeCloud, *closed_cloud, icp.getFinalTransformation());
//         publishCloud(&pubIcpKeyFrames, closed_cloud, timeLaserInfoStamp, odometryFrame);
//     }

//     // Get pose transformation
//     float x, y, z, roll, pitch, yaw;
//     Eigen::Affine3f correctionLidarFrame;
//     correctionLidarFrame = icp.getFinalTransformation();
//     // transform from world origin to wrong pose
//     Eigen::Affine3f tWrong = pclPointToAffine3f(copy_cloudKeyPoses6D->points[loopKeyCur]);
//     // transform from world origin to corrected pose
//     Eigen::Affine3f tCorrect = correctionLidarFrame * tWrong;// pre-multiplying -> successive rotation about a fixed frame
//     pcl::getTranslationAndEulerAngles (tCorrect, x, y, z, roll, pitch, yaw);
//     gtsam::Pose3 poseFrom = Pose3(Rot3::RzRyRx(roll, pitch, yaw), Point3(x, y, z));
//     gtsam::Pose3 poseTo = pclPointTogtsamPose3(copy_cloudKeyPoses6D->points[loopKeyPre]);
//     gtsam::Vector Vector6(6);
//     float noiseScore = icp.getFitnessScore();
//     Vector6 << noiseScore, noiseScore, noiseScore, noiseScore, noiseScore, noiseScore;
//     noiseModel::Diagonal::shared_ptr constraintNoise = noiseModel::Diagonal::Variances(Vector6);

//     // Add pose constraint
//     mtx.lock();
//     loopIndexQueue.push_back(make_pair(loopKeyCur, loopKeyPre));
//     loopPoseQueue.push_back(poseFrom.between(poseTo));
//     loopNoiseQueue.push_back(constraintNoise);
//     mtx.unlock();

//     // add loop constriant
//     loopIndexContainer[loopKeyCur] = loopKeyPre;
// }

bool CLoop::detectLoopClosureDistance(int *latestID, int *closestID)
{
    int loopKeyCur = copy_cloudKeyPoses3D->size() - 1;
    int loopKeyPre = -1;

    // check loop constraint added before
    auto it = loopIndexContainer.find(loopKeyCur);
    if (it != loopIndexContainer.end())
        return false;

    // find the closest history key frame
    std::vector<int> pointSearchIndLoop;
    std::vector<float> pointSearchSqDisLoop;
    kdtreeHistoryKeyPoses->setInputCloud(copy_cloudKeyPoses3D);
    kdtreeHistoryKeyPoses->radiusSearch(copy_cloudKeyPoses3D->back(), historyKeyframeSearchRadius, pointSearchIndLoop, pointSearchSqDisLoop, 0);
    
    for (int i = 0; i < (int)pointSearchIndLoop.size(); ++i)
    {
        int id = pointSearchIndLoop[i];
        if (abs(copy_cloudKeyPoses6D->points[id].time - timeLaserInfoCur) > historyKeyframeSearchTimeDiff)
        {
            loopKeyPre = id;
            break;
        }
    }

    if (loopKeyPre == -1 || loopKeyCur == loopKeyPre)
        return false;

    *latestID = loopKeyCur;
    *closestID = loopKeyPre;

    return true;
}

bool CLoop::detectLoopClosureExternal(int *latestID, int *closestID)
{
    // this function is not used yet, please ignore it
    int loopKeyCur = -1;
    int loopKeyPre = -1;

    std::lock_guard<std::mutex> lock(mtxLoopInfo);
    if (loopInfoVec.empty())
        return false;

    double loopTimeCur = loopInfoVec.front().data[0];
    double loopTimePre = loopInfoVec.front().data[1];
    loopInfoVec.pop_front();

    if (abs(loopTimeCur - loopTimePre) < historyKeyframeSearchTimeDiff)
        return false;

    int cloudSize = copy_cloudKeyPoses6D->size();
    if (cloudSize < 2)
        return false;

    // latest key
    loopKeyCur = cloudSize - 1;
    for (int i = cloudSize - 1; i >= 0; --i)
    {
        if (copy_cloudKeyPoses6D->points[i].time >= loopTimeCur)
            loopKeyCur = round(copy_cloudKeyPoses6D->points[i].intensity);
        else
            break;
    }

    // previous key
    loopKeyPre = 0;
    for (int i = 0; i < cloudSize; ++i)
    {
        if (copy_cloudKeyPoses6D->points[i].time <= loopTimePre)
            loopKeyPre = round(copy_cloudKeyPoses6D->points[i].intensity);
        else
            break;
    }

    if (loopKeyCur == loopKeyPre)
        return false;

    auto it = loopIndexContainer.find(loopKeyCur);
    if (it != loopIndexContainer.end())
        return false;

    *latestID = loopKeyCur;
    *closestID = loopKeyPre;

    return true;
}

void CLoop::loopFindNearKeyframes(pcl::PointCloud<PointType>::Ptr& nearKeyframes, const int& key, const int& searchNum)
{
    // extract near keyframes
    nearKeyframes->clear();
    int cloudSize = copy_cloudKeyPoses6D->size();
    for (int i = -searchNum; i <= searchNum; ++i)
    {
        int keyNear = key + i;
        if (keyNear < 0 || keyNear >= cloudSize )
            continue;
        *nearKeyframes += *transformPointCloud(cornerCloudKeyFrames[keyNear], &copy_cloudKeyPoses6D->points[keyNear]);
        *nearKeyframes += *transformPointCloud(surfCloudKeyFrames[keyNear],   &copy_cloudKeyPoses6D->points[keyNear]);
    }

    if (nearKeyframes->empty())
        return;

    // downsample near keyframes
    pcl::PointCloud<PointType>::Ptr cloud_temp(new pcl::PointCloud<PointType>());
    downSizeFilterICP.setInputCloud(nearKeyframes);
    downSizeFilterICP.filter(*cloud_temp);
    *nearKeyframes = *cloud_temp;
}

}//namespace LIOSAM
