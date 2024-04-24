#include "include/utility.h"

// namespace LIOSAM
// {

Eigen::Affine3f pclPointToAffine3fLiosam(PointTypePose thisPoint)
{ 
    return pcl::getTransformation(thisPoint.x, thisPoint.y, thisPoint.z, thisPoint.roll, thisPoint.pitch, thisPoint.yaw);
}

// gtsam::Pose3 pclPointTogtsamPose3Liosam(PointTypePose thisPoint)
// {
//     return gtsam::Pose3(gtsam::Rot3::RzRyRx(double(thisPoint.roll), double(thisPoint.pitch), double(thisPoint.yaw)),
//                                 gtsam::Point3(double(thisPoint.x),    double(thisPoint.y),     double(thisPoint.z)));
// }

sensor_msgs::PointCloud2 publishCloud(ros::Publisher *thisPub, pcl::PointCloud<PointType>::Ptr thisCloud, ros::Time thisStamp, std::string thisFrame)
{
    sensor_msgs::PointCloud2 tempCloud;
    pcl::toROSMsg(*thisCloud, tempCloud);
    tempCloud.header.stamp = thisStamp;
    tempCloud.header.frame_id = thisFrame;
    if (thisPub->getNumSubscribers() != 0)
        thisPub->publish(tempCloud);
    return tempCloud;
}

pcl::PointCloud<PointType>::Ptr transformPointCloud(pcl::PointCloud<PointType>::Ptr cloudIn, PointTypePose* transformIn)
{
    // std::cout << __FUNCTION__ << " start" << std::endl;

    int cloudSize = cloudIn->size();
    pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());
    cloudOut->resize(cloudSize);

    Eigen::Affine3f transCur = pcl::getTransformation(transformIn->x, transformIn->y, transformIn->z, transformIn->roll, transformIn->pitch, transformIn->yaw);
    
    for (int i = 0; i < cloudSize; ++i)
    {
        const auto &pointFrom = cloudIn->points[i];
        cloudOut->points[i].x = transCur(0,0) * pointFrom.x + transCur(0,1) * pointFrom.y + transCur(0,2) * pointFrom.z + transCur(0,3);
        cloudOut->points[i].y = transCur(1,0) * pointFrom.x + transCur(1,1) * pointFrom.y + transCur(1,2) * pointFrom.z + transCur(1,3);
        cloudOut->points[i].z = transCur(2,0) * pointFrom.x + transCur(2,1) * pointFrom.y + transCur(2,2) * pointFrom.z + transCur(2,3);
        cloudOut->points[i].intensity = pointFrom.intensity;
    }

    return cloudOut;
}

// }//namespace LIOSAM
