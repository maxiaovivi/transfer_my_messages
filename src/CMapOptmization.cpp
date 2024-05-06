#include "include/CMapOptmization.h"
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/statistical_outlier_removal.h>

#define OPEN_RAW_LOG 0
#define OPEN_DISPLAY_MAP 0

#define USE_GTSAM 1
#define USE_SCANMATCH 1

using namespace LIOSAM;

CMapOptimization::CMapOptimization(ros::NodeHandle nnh) : nh(nnh)
{
    ISAM2Params parameters;
    parameters.relinearizeThreshold = 0.1;
    parameters.relinearizeSkip = 1;
    isam = new ISAM2(parameters);

    allocateMemory();

    m_local_mapping = new CLocalMapping(nnh);
    m_local_mapping_thread = new thread(&LIOSAM::CLocalMapping::run, m_local_mapping);
    m_cloud_thin_thread = new thread(&LIOSAM::CLocalMapping::cloudThin, m_local_mapping);
#if OPEN_DISPLAY_MAP
    m_map_display = new CMapDisplay();
    m_display_thread = new thread(&LIOSAM::CMapDisplay::run, m_map_display);
    m_map_display->setLocalMapper(m_local_mapping);
#endif

#if OPEN_RAW_LOG
    m_raw_log = new CMessageData();
    m_raw_log_thread = new thread(&LIOSAM::CMessageData::run, m_raw_log);
    m_raw_log->setLocalMapper(m_local_mapping);
#endif

    // ros
    m_pub_ds_surf_points_pre = nh.advertise<sensor_msgs::PointCloud2>("dsSurfPredict", 1);
    m_pub_sel_surf_points_pre = nh.advertise<sensor_msgs::PointCloud2>("selSurfPredict", 1);
    m_pub_ds_surf_points_opt = nh.advertise<sensor_msgs::PointCloud2>("dsSurfOpt", 1);

    m_pub_scan_to_map_icp = nh.advertise<sensor_msgs::PointCloud2>("scanToMapIcp", 1);

    m_pub_tof_odometry_global = nh.advertise<nav_msgs::Odometry>("tofOdomGlobal", 1);
    m_pub_global_path = nh.advertise<nav_msgs::Path>("global_path", 1);
}

CMapOptimization::~CMapOptimization()
{
    m_local_mapping->setExit();
    m_local_mapping_thread->join();
    m_cloud_thin_thread->join();
    delete m_local_mapping;
      
#if OPEN_DISPLAY_MAP
    m_display_thread->join();
    delete m_map_display;
#endif

#if OPEN_RAW_LOG
    m_raw_log->setExit();
    m_raw_log_thread->join();
    delete m_raw_log;
#endif
}

void CMapOptimization::allocateMemory()
{
    m_cloud_key_poses_3d.reset(new pcl::PointCloud<PointType>());       // DEBUG_LJH
    m_cloud_key_poses_6d.reset(new pcl::PointCloud<PointTypePose>());   // DEBUG_LJH
    m_laser_cloud_surf_last.reset(new pcl::PointCloud<PointType>());    // surf feature set from odoOptimization
    m_laser_cloud_surf_last_ds.reset(new pcl::PointCloud<PointType>()); // downsampled surf featuer set from odoOptimization
    m_SubMap.reset(new pcl::PointCloud<PointType>);                     // DEBUG_LJH
    m_temp_feature_cloud.reset(new pcl::PointCloud<PointType>);
    m_laser_cloud_ori.reset(new pcl::PointCloud<PointType>());
    m_robot_traject.reset(new pcl::PointCloud<PointType>());
    m_coeff_sel.reset(new pcl::PointCloud<PointType>());

    m_laser_cloud_ori_surf_vec.resize(N_SCAN_FRONT * H_SCAN_FRONT + N_SCAN_LEFT * H_SCAN_LEFT);
    m_coeff_sel_surf_vec.resize(N_SCAN_FRONT * H_SCAN_FRONT + N_SCAN_LEFT * H_SCAN_LEFT);
    m_laser_cloud_ori_surf_flag.resize(N_SCAN_FRONT * H_SCAN_FRONT + N_SCAN_LEFT * H_SCAN_LEFT);

    m_cloud_info.startRingIndex.resize(N_SCAN_FRONT * H_SCAN_FRONT + N_SCAN_LEFT * H_SCAN_LEFT);
    m_cloud_info.endRingIndex.resize(N_SCAN_FRONT * H_SCAN_FRONT + N_SCAN_LEFT * H_SCAN_LEFT);
    m_cloud_info.pointColInd.resize(N_SCAN_FRONT * H_SCAN_FRONT + N_SCAN_LEFT * H_SCAN_LEFT);
    m_cloud_info.pointRange.resize(N_SCAN_FRONT * H_SCAN_FRONT + N_SCAN_LEFT * H_SCAN_LEFT);

    std::fill(m_laser_cloud_ori_surf_flag.begin(), m_laser_cloud_ori_surf_flag.end(), false);

    for (int i = 0; i < 6; ++i)
    {
        m_transform_tobe_mapped[i] = 0;
    }

    m_mat_p = cv::Mat(6, 6, CV_32F, cv::Scalar::all(0));

    m_map_invalid_times = 0;
    
    initializedFlag = NonInitialized;

}

void CMapOptimization::tofCloudInfoHandler(const pcl::PointCloud<PointType>::Ptr tofCloudFront, const pcl::PointCloud<PointType>::Ptr tofCloudLeft, const double tofTimestampFrint, const double tofTimestampLeft, const odom_info odomInfo, const std::deque<imu_info> imuInfoDeque, const ros::Publisher &pub1, const ros::Publisher &pub2, const ros::Publisher &pub3, const ros::Publisher &pub4, const ros::Publisher &pub5, const ros::Publisher &pub6)
{
#if OPEN_RAW_LOG
    m_raw_log->pushRawLogData(tofCloudFront, tofCloudLeft, odomInfo);
#endif
    PointType currentPose_cloud;
    sensor_msgs::PointCloud2 pointcloud_currentPose;
    m_time_laser_info_cur = odomInfo.odomTimestamp;
    m_imu_roll_deg = imuInfoDeque[0].imuRoll * ARC_TO_DEG;
    m_imu_pitch_deg = imuInfoDeque[0].imuPitch * ARC_TO_DEG;
    m_imu_yaw_vel_deg = imuInfoDeque[0].imuYawVel * ARC_TO_DEG;

    m_laser_cloud_surf_last->clear();
    m_feature_extraction.tofCloudInfoHandler(tofCloudFront, tofCloudLeft, tofTimestampFrint, tofTimestampLeft, m_time_laser_info_cur, imuInfoDeque);
    *m_laser_cloud_surf_last = m_feature_extraction.getFeatureCloud();
    CLog() << " featurePointNum = " << m_laser_cloud_surf_last->size() << std::endl;

    if (!m_local_map_init)
        m_local_map_init = m_local_mapping->isInit();

    if (m_local_map_init)
    {
        m_key_frame_num = m_local_mapping->getKeyFrameNum();
        m_local_mapping->getKeyPosesPre(&m_key_frame_pose_pre);
    }

    gravityAlign(odomInfo, imuInfoDeque); // sai

    updateInitialGuess();

    downsampleCurrentScan(pub4);

    scan2MapOptimization(pub3, pub5, pub1);

    saveKeyFramesAndFactor();

#if USE_GTSAM
    if (m_output_pose)
    {
        addOdomFactor();

        // update iSAM
        isam->update(gtSAMgraph, initialEstimate);

        gtSAMgraph.resize(0);
        initialEstimate.clear();

        // m_scan_match_sucess = false;
        // save key poses
        PointType thisPose3D;
        PointTypePose thisPose6D;

        Pose3 latestEstimate;
        isamCurrentEstimate = isam->calculateEstimate();
        latestEstimate = isamCurrentEstimate.at<Pose3>(isamCurrentEstimate.size() - 1);
        // cout << "****************************************************" << endl;
        // isamCurrentEstimate.print("Current estimate: ");

        thisPose3D.x = latestEstimate.translation().x();
        thisPose3D.y = latestEstimate.translation().y();
        thisPose3D.z = latestEstimate.translation().z();
        thisPose3D.intensity = m_cloud_key_poses_3d->size(); // this can be used as index
        m_cloud_key_poses_3d->push_back(thisPose3D);

        thisPose6D.x = thisPose3D.x;
        thisPose6D.y = thisPose3D.y;
        thisPose6D.z = thisPose3D.z;
        thisPose6D.intensity = thisPose3D.intensity; // this can be used as index
        thisPose6D.roll = latestEstimate.rotation().roll();
        thisPose6D.pitch = latestEstimate.rotation().pitch();
        thisPose6D.yaw = latestEstimate.rotation().yaw();
        thisPose6D.time = m_time_laser_info_cur;
        m_cloud_key_poses_6d->push_back(thisPose6D);

        poseCovariance = isam->marginalCovariance(isamCurrentEstimate.size() - 1);
        m_transform_tobe_mapped[0] = latestEstimate.rotation().roll();
        m_transform_tobe_mapped[1] = latestEstimate.rotation().pitch();
        m_transform_tobe_mapped[2] = latestEstimate.rotation().yaw();
        m_transform_tobe_mapped[3] = latestEstimate.translation().x();
        m_transform_tobe_mapped[4] = latestEstimate.translation().y();
        m_transform_tobe_mapped[5] = latestEstimate.translation().z();
    }
#endif

    std::cout << "[3DSlam]" << "SAI: FINAL x y z roll pitch yaw " << setprecision(6)
              << m_transform_tobe_mapped[3] << " "
              << m_transform_tobe_mapped[4] << " "
              << m_transform_tobe_mapped[5] << " "
              << m_transform_tobe_mapped[0] << " "
              << m_transform_tobe_mapped[1] << " "
              << m_transform_tobe_mapped[2] << std::endl;

    // ros
    // geometry_msgs::PoseStamped pose_stamped;
    // pose_stamped.header.stamp = ros::Time::now();
    // pose_stamped.header.frame_id = "map";
    // pose_stamped.pose.position.x = m_transform_tobe_mapped[3];
    // pose_stamped.pose.position.y = m_transform_tobe_mapped[4];
    // pose_stamped.pose.position.z = m_transform_tobe_mapped[5];
    // tf::Quaternion q = tf::createQuaternionFromRPY(m_transform_tobe_mapped[0], m_transform_tobe_mapped[1], m_transform_tobe_mapped[2]);
    // pose_stamped.pose.orientation.x = q.x();
    // pose_stamped.pose.orientation.y = q.y();
    // pose_stamped.pose.orientation.z = q.z();
    // pose_stamped.pose.orientation.w = q.w();

    // m_global_path.header.stamp = ros::Time::now();
    // m_global_path.header.frame_id = "map";
    // m_global_path.poses.push_back(pose_stamped);
    // m_pub_global_path.publish(m_global_path);

    currentPose_cloud.x = m_transform_tobe_mapped[3];
    currentPose_cloud.y = m_transform_tobe_mapped[4];
    currentPose_cloud.z = m_transform_tobe_mapped[5];
    m_robot_traject->push_back(currentPose_cloud);
    pcl::toROSMsg(*m_robot_traject, pointcloud_currentPose);
    pointcloud_currentPose.header.frame_id = "map";
    pointcloud_currentPose.header.stamp = ros::Time::now();
    pub6.publish(pointcloud_currentPose);
}

void CMapOptimization::gravityAlign(const odom_info odomInfo, const std::deque<struct imu_info> imuInfoDeque)
{
    m_cloud_info.initialGuessX = odomInfo.odomX;
    m_cloud_info.initialGuessY = odomInfo.odomY;
    m_cloud_info.initialGuessZ = odomInfo.odomZ;
    m_cloud_info.initialGuessRoll = odomInfo.odomRoll;
    m_cloud_info.initialGuessPitch = odomInfo.odomPitch;
    m_cloud_info.initialGuessYaw = odomInfo.odomYaw;
    CLog() << " Odom_yaw = " << odomInfo.odomYaw << std::endl;

    if (imuInfoDeque.empty()) {
        return;
    }
    for (int i = 0; i < (int)imuInfoDeque.size(); i++) {
        m_imu_deque_for_gravity.push_back(imuInfoDeque[i]);
    }
    m_climbing = false;
    m_climbing_harder = false;
    m_climbing_harsh = false;
    int climb_count = 0;
    int climb_count2 = 0;
    int climb_count3 = 0;
    // static bool gravity_init_flag = true;
    while (!m_imu_deque_for_gravity.empty()) {
        double time = m_imu_deque_for_gravity[0].imuTimestamp;
        if (time > m_time_laser_info_cur) {
            break;
        }
        m_cloud_info.initialGuessRoll = m_imu_deque_for_gravity[0].imuRoll;
        m_cloud_info.initialGuessPitch = m_imu_deque_for_gravity[0].imuPitch;

        // Eigen::Vector3f imu_linear_acceleration(m_imu_deque_for_gravity[0].imuXAcc, m_imu_deque_for_gravity[0].imuYAcc, m_imu_deque_for_gravity[0].imuZAcc);
        // Eigen::Vector3f imu_angular_velocity(m_imu_deque_for_gravity[0].imuRollVel, m_imu_deque_for_gravity[0].imuPitchVel, m_imu_deque_for_gravity[0].imuYawVel);
        // imuPredict(time, gravity_init_flag);
        // addImuObservation(time, imu_linear_acceleration, imu_angular_velocity, gravity_init_flag);
        // if (gravity_init_flag) {
        //     gravity_init_flag = false;
        // }

        // if (!m_climbing) {
        if (1) {
            double temp_roll = m_imu_deque_for_gravity[0].imuRoll * ARC_TO_DEG;
            double temp_pitch = m_imu_deque_for_gravity[0].imuPitch * ARC_TO_DEG;
            if (abs(temp_roll) > 3.0 || abs(temp_pitch) > 3.0) {
                climb_count++;
            }
            if (abs(temp_roll) > 5.0 || abs(temp_pitch) > 5.0) {
                climb_count2++;
            }
            if (abs(temp_roll) > 8.0 || abs(temp_pitch) > 8.0) {
                climb_count3++;
            }
            if (climb_count >= 2) {
                m_climbing = true;
            }
            if (climb_count2 >= 2) {
                m_climbing_harder = true;
            }
            if (climb_count3 >= 2) {
                m_climbing_harsh = true;
            }
        }
        m_imu_deque_for_gravity.pop_front();
    }
    CLog() << "climb_count above 3/5/8 degree " << climb_count << " " << climb_count2 << " " << climb_count3 << std::endl;

    // Eigen::Affine3f gravity_alignment = Eigen::Affine3f::Identity();
    // gravity_alignment.rotate(m_imu_orientation);
    // float x_align, y_align, z_align, roll_align, pitch_align, yaw_align;
    // pcl::getTranslationAndEulerAngles(gravity_alignment, x_align, y_align, z_align, roll_align, pitch_align, yaw_align);

    // // sai: remove yaw on Z-axis
    // m_gravity_alignment = pcl::getTransformation(x_align, y_align, z_align, roll_align, pitch_align, 0);

    // // sai: print log
    // std::cout << "[3DSlam]" << "SAI: GRAVITY DIRECTION X Y Z " << setprecision(9)
    //         << m_gravity_direction.x() << " " << m_gravity_direction.y() << " " << m_gravity_direction.z() << std::endl;  
    // Eigen::Affine3f gravity_direction = pcl::getTransformation(m_gravity_direction.x(), m_gravity_direction.y(), m_gravity_direction.z(), 0, 0, 0);
    // Eigen::Affine3f predict_z_direction = m_gravity_alignment * gravity_direction;
    // float x_Z, y_Z, z_Z, roll_Z, pitch_Z, yaw_Z;
    // pcl::getTranslationAndEulerAngles(predict_z_direction, x_Z, y_Z, z_Z, roll_Z, pitch_Z, yaw_Z);
    // std::cout << "[3DSlam]" << "SAI: PREDICT Z-DIRECTION X Y Z ROLL PITCH YAW " << setprecision(9)
    //         << x_Z << " " << y_Z << " " << z_Z << " "
    //         << roll_Z << " " << pitch_Z << " " << yaw_Z << std::endl;
}

void CMapOptimization::imuPredict(const double time, const bool init)
{
    if (init) {
        m_imu_last_time = time;
        return;
    }
    const double delta_t = time - m_imu_last_time;
    if (delta_t == 0.0) {
        return;
    }
    const Eigen::Quaternionf rotation = CMapOptimization::AngleAxisVectorToRotationQuaternion(Eigen::Vector3f(m_imu_last_angular_velocity * delta_t));
    m_imu_orientation = (m_imu_orientation * rotation).normalized();
    m_gravity_direction = rotation.inverse() * m_gravity_direction;
}

void CMapOptimization::addImuObservation(const double time, const Eigen::Vector3f& imu_linear_acceleration, const Eigen::Vector3f& imu_angular_velocity, const bool init)
{
    // Update the 'gravity_direction_' with an exponential moving average using the 'imu_gravity_time_constant'.
    const double delta_t = (!init) ? (time - m_imu_last_time) : std::numeric_limits<double>::infinity();
    const double alpha = 1.0 - std::exp(-delta_t / m_imu_gravity_time_constant);
    m_gravity_direction = (1. - alpha) * m_gravity_direction + alpha * imu_linear_acceleration;

    // Change the 'orientation_' so that it agrees with the current 'gravity_direction_'.
    const Eigen::Quaternionf rotation = Eigen::Quaternionf::FromTwoVectors(m_gravity_direction, m_imu_orientation.inverse() * Eigen::Vector3f::UnitZ());
    m_imu_orientation = (m_imu_orientation * rotation).normalized();
    // CHECK_GT((orientation_.inverse() * Eigen::Vector3d::UnitZ())
    //             .dot(gravity_direction_),
    //        0.);

    // Store the current 'imu_angular_velocity'
    m_imu_last_angular_velocity = imu_angular_velocity;
    m_imu_last_time = time;
}

void CMapOptimization::updateInitialGuess()
{
    // use imu pre-integration estimation for pose guess
    static Eigen::Affine3f lastImuPreTransformation;
    // initialization
    static bool mapped_init_flag = true;
    if (mapped_init_flag && m_local_mapping->getKeyFrameNum() < 1)
    {
        mapped_init_flag = false;
        // m_transform_tobe_mapped[0] = m_cloud_info.initialGuessRoll;
        // m_transform_tobe_mapped[1] = m_cloud_info.initialGuessPitch;
        // m_transform_tobe_mapped[2] = m_cloud_info.initialGuessYaw;
        m_transform_tobe_mapped[2] = 1.95;
        lastImuPreTransformation = pcl::getTransformation(0, 0, 0, m_cloud_info.initialGuessRoll, m_cloud_info.initialGuessPitch, m_cloud_info.initialGuessYaw);
        return;
    }

    Eigen::Affine3f transBack = pcl::getTransformation(m_cloud_info.initialGuessX, m_cloud_info.initialGuessY, m_cloud_info.initialGuessZ,
                                                       m_cloud_info.initialGuessRoll, m_cloud_info.initialGuessPitch, m_cloud_info.initialGuessYaw);
    Eigen::Affine3f transIncre = lastImuPreTransformation.inverse() * transBack;
    Eigen::Affine3f transTobe = trans2Affine3f(m_transform_tobe_mapped);
    Eigen::Affine3f transFinal = transTobe * transIncre;
    pcl::getTranslationAndEulerAngles(transFinal, m_transform_tobe_mapped[3], m_transform_tobe_mapped[4], m_transform_tobe_mapped[5],
                                        m_transform_tobe_mapped[0], m_transform_tobe_mapped[1], m_transform_tobe_mapped[2]);
    pcl::getTranslationAndEulerAngles(transFinal, m_transform_tobe_mapped_back[3], m_transform_tobe_mapped_back[4], m_transform_tobe_mapped_back[5],
                                        m_transform_tobe_mapped_back[0], m_transform_tobe_mapped_back[1], m_transform_tobe_mapped_back[2]);
    pcl::getTranslationAndEulerAngles(transTobe, m_transform_tobe_mapped_last[3], m_transform_tobe_mapped_last[4], m_transform_tobe_mapped_last[5],
                                        m_transform_tobe_mapped_last[0], m_transform_tobe_mapped_last[1], m_transform_tobe_mapped_last[2]);
    lastImuPreTransformation = transBack;

    // sai
    m_pure_rotation_flag = false; // sai
    float xIncre, yIncre, zIncre, rollIncre, pitchIncre, yawIncre;
    pcl::getTranslationAndEulerAngles(transIncre, xIncre, yIncre, zIncre, rollIncre, pitchIncre, yawIncre);
    // CLog() << "SAI: INCRE X Y Z ROLL PITCH YAW " << setprecision(20)
    //         << xIncre << " " << yIncre << " " << zIncre<< " "
    //         << rollIncre << " " << pitchIncre << " " << yawIncre << std::endl;
    if(sqrt(pow(xIncre, 2) + pow(yIncre, 2) + pow(zIncre, 2)) < 0.005) {
        m_pure_rotation_flag = true;
        CLog() << "SAI: PURE ROTATION" << std::endl; 
    }
    return;
}

void CMapOptimization::downsampleCurrentScan(const ros::Publisher &pub)
{
    m_laser_cloud_surf_last_ds->clear();

    pcl::VoxelGrid<PointType> m_down_size_filter_surf;
    m_down_size_filter_surf.setInputCloud(m_laser_cloud_surf_last);
    m_down_size_filter_surf.setLeafSize(0.01, 0.01, 0.01);
    // m_down_size_filter_surf.setLeafSize(0.05, 0.05, 0.05);
    m_down_size_filter_surf.filter(*m_laser_cloud_surf_last_ds);

    // pcl::copyPointCloud(*m_laser_cloud_surf_last, *m_laser_cloud_surf_last_ds);
    m_laser_cloud_surf_last_ds_num = m_laser_cloud_surf_last_ds->size();
    CLog() << " dsPointSize = " << m_laser_cloud_surf_last_ds_num << std::endl;

    // ros
    PointTypePose thisPose6D;
    thisPose6D.x = m_transform_tobe_mapped[3];
    thisPose6D.y = m_transform_tobe_mapped[4];
    thisPose6D.z = m_transform_tobe_mapped[5];
    thisPose6D.roll = m_transform_tobe_mapped[0];
    thisPose6D.pitch = m_transform_tobe_mapped[1];
    thisPose6D.yaw = m_transform_tobe_mapped[2];

    pcl::PointCloud<PointType>::Ptr laser_cloud_surf_global = transformPointCloud(m_laser_cloud_surf_last_ds, &thisPose6D);
    sensor_msgs::PointCloud2 pointcloud_ds_surf;
    pcl::toROSMsg(*laser_cloud_surf_global, pointcloud_ds_surf);
    pointcloud_ds_surf.header.frame_id = "map";
    pointcloud_ds_surf.header.stamp = ros::Time::now();
    pub.publish(pointcloud_ds_surf);
}

/*
void CMapOptimization::scan2MapIcp()
{
    if (m_laser_cloud_surf_last_ds->size() < 500)
        return;

    PointTypePose thisPose6D;
    thisPose6D.x = m_transform_tobe_mapped[3];
    thisPose6D.y = m_transform_tobe_mapped[4];
    thisPose6D.z = m_transform_tobe_mapped[5];
    thisPose6D.roll = m_transform_tobe_mapped[0];
    thisPose6D.pitch = m_transform_tobe_mapped[1];
    thisPose6D.yaw = m_transform_tobe_mapped[2];

    pcl::PointCloud<PointType>::Ptr laser_cloud_surf_last_ds_global(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr global_map(new pcl::PointCloud<PointType>());
    laser_cloud_surf_last_ds_global = transformPointCloud(m_laser_cloud_surf_last_ds, &thisPose6D);
    *global_map = m_local_mapping->getGlobalMap();
    // ICP Settings
    static pcl::IterativeClosestPoint<PointType, PointType> icp;
    icp.setMaxCorrespondenceDistance(10);
    icp.setMaximumIterations(100);
    icp.setTransformationEpsilon(1e-6);
    icp.setEuclideanFitnessEpsilon(1e-6);
    icp.setRANSACIterations(0);

    // Align clouds
    icp.setInputSource(laser_cloud_surf_last_ds_global);
    icp.setInputTarget(global_map);
    pcl::PointCloud<PointType>::Ptr unused_result(new pcl::PointCloud<PointType>());
    icp.align(*unused_result);

    // if (icp.hasConverged() == false || icp.getFitnessScore() > 0.02)
    // {
    //     return;
    // }

    if (icp.hasConverged() == true && icp.getFitnessScore() < 0.02)
    {
        // Get pose transformation
        float x, y, z, roll, pitch, yaw;
        Eigen::Affine3f correctionLidarFrame;
        correctionLidarFrame = icp.getFinalTransformation();
        // transform from world origin to wrong pose
        Eigen::Affine3f tWrong = pclPointToAffine3fLiosam(thisPose6D);
        // transform from world origin to corrected pose
        Eigen::Affine3f tCorrect = correctionLidarFrame * tWrong; // pre-multiplying -> successive rotation about a fixed frame
        pcl::getTranslationAndEulerAngles(tCorrect, x, y, z, roll, pitch, yaw);

        m_transform_tobe_mapped[0] = roll;
        m_transform_tobe_mapped[1] = pitch;
        m_transform_tobe_mapped[2] = yaw;
        m_transform_tobe_mapped[3] = x;
        m_transform_tobe_mapped[4] = y;
        m_transform_tobe_mapped[5] = z;
    }

    // // Get pose transformation
    // float x, y, z, roll, pitch, yaw;
    // Eigen::Affine3f correctionLidarFrame;
    // correctionLidarFrame = icp.getFinalTransformation();
    // // transform from world origin to wrong pose
    // Eigen::Affine3f tWrong = pclPointToAffine3fLiosam(thisPose6D);
    // // transform from world origin to corrected pose
    // // Eigen::Affine3f tCorrect = correctionLidarFrame * tWrong;// pre-multiplying -> successive rotation about a fixed frame
    // Eigen::Affine3f tCorrect = correctionLidarFrame;
    // pcl::getTranslationAndEulerAngles (tCorrect, x, y, z, roll, pitch, yaw);

    // m_transform_tobe_mapped[0] = roll;
    // m_transform_tobe_mapped[1] = pitch;
    // m_transform_tobe_mapped[2] = yaw;
    // m_transform_tobe_mapped[3] = x;
    // m_transform_tobe_mapped[4] = y;
    // m_transform_tobe_mapped[5] = z;

    thisPose6D.x = m_transform_tobe_mapped[3];
    thisPose6D.y = m_transform_tobe_mapped[4];
    thisPose6D.z = m_transform_tobe_mapped[5];
    thisPose6D.roll = m_transform_tobe_mapped[0];
    thisPose6D.pitch = m_transform_tobe_mapped[1];
    thisPose6D.yaw = m_transform_tobe_mapped[2];

    pcl::PointCloud<PointType>::Ptr laser_cloud_surf_global = transformPointCloud(m_laser_cloud_surf_last_ds, &thisPose6D);
    std::string frame_id = "map";
    publishCloud(&m_pub_scan_to_map_icp, laser_cloud_surf_global, ros::Time::now(), frame_id);

    // std::string frame_id = "map";
    // pcl::PointCloud<PointType>::Ptr closed_cloud(new pcl::PointCloud<PointType>());
    // pcl::transformPointCloud(*laser_cloud_surf_last_ds_global, *closed_cloud, icp.getFinalTransformation());
    // publishCloud(&m_pub_scan_to_map_icp, closed_cloud, ros::Time::now(), frame_id);
}

*/
void CMapOptimization::scan2MapOptimization(const ros::Publisher &pub1, const ros::Publisher &pub2, const ros::Publisher &pub3) // pub1 = submap,pub2 = scan,pub3 = result
{
    if (!m_local_map_init)
    {
        return;
    }

    m_is_degenerate = false;            // sai: 特征值退化
    m_eigenvalue_big_enough = true;     // sai: 最大特征值足够大（与有效点数有关）
    m_is_divergent = true;              // sai: 优化器发散
    m_gnresult_small_enough = false;    // sai: 优化器残差足够小
    m_use_odom = false;                 // sai: 跳过优化，使用里程计位姿
    m_old_point_ratio = 1.0;            // sai: 旧点比例
    m_finish_bruceAlign = false;        // sai: 当前帧是否做暴力搜索

    if (m_wait_for_bruceAlign && enoughPointCheck(m_laser_cloud_surf_last_ds) && validFrameCheck(m_laser_cloud_surf_last_ds, 0.5, true) )
    {
        // sai: TODO 暴力搜索 先搜索旋转 再搜索位移。开4个线程并行计算
        bruceAlignment(pub1, pub2, pub3);
        m_wait_for_bruceAlign = false;

        // 退出时赋值为经过优化的良好结果
        m_is_divergent = false;
        m_eigenvalue_big_enough = true;
        m_gnresult_small_enough = true;
        m_old_point_ratio = 1.0;
        m_finish_bruceAlign = true; // bad result default
    }
    else 
    {
        int max_iter = m_max_iter;
        if (m_laser_cloud_surf_last_ds_num > m_surf_feature_min_valid_num && validFrameCheck(m_laser_cloud_surf_last_ds, 0.3, false) )
        {
            double total_time = 0.0;
            for (int iterCount = 0; iterCount < max_iter; iterCount++)
            {
                // mrpt::system::TTimeStamp gn_t1 = mrpt::system::now();
                m_laser_cloud_ori->clear();
                m_coeff_sel->clear();
                bool gn_flag = false;

                // sai: initial alignment
                if (iterCount == 0) {
                    surfOptimizationAndCalculateDeltaXYZ();
                    combineOptimizationCoeffs();
                    gn_flag = initialAlignment(iterCount, pub1, pub2, pub3);
                } else {
                    surfOptimization();
                    combineOptimizationCoeffs();
                    gn_flag = gnOptimization(iterCount, max_iter);
                }

                // mrpt::system::TTimeStamp gn_t2 = mrpt::system::now();
                // double time_cost = mrpt::system::timeDifference(gn_t1, gn_t2) * 1000.0;
                // total_time += time_cost;
                // CLog::logAsync(LogSara, LogNormal, "[CMapOptimization] gn cost = %5.2f ms, total cost = %5.2f.\n", time_cost, total_time);

                CLog() << " SAI: ITER" << (iterCount+1) << " gnOpt result R and T = " << dlR << " " << dlT << std::endl;
                if (gn_flag == true)
                {
                    m_is_divergent = false;
                    break;  
                }
                // if(total_time > 200) break;   
            }
        } 
        else
        {
            // 退出时赋值为没有经过优化的良好结果
            m_is_divergent = false;
            m_eigenvalue_big_enough = true;
            m_gnresult_small_enough = true;
            m_old_point_ratio = 0.0;
            m_use_odom = true;
            // CLog::logAsync(LogSara, LogNormal, "[CMapOptimization] surf ds num = %d. \n", m_laser_cloud_surf_last_ds_num);
        }
    }

    // sai: final alignment
    finalAlignment();
}

void CMapOptimization::addOdomFactor()
{
    if (m_cloud_key_poses_3d->points.empty())
    {
        noiseModel::Diagonal::shared_ptr priorNoise = noiseModel::Diagonal::Variances((Vector(6) << 1e-2, 1e-2, M_PI * M_PI, 1e8, 1e8, 1e8).finished()); // rad*rad, meter*meter
        gtSAMgraph.add(PriorFactor<Pose3>(0, trans2gtsamPose(m_transform_tobe_mapped), priorNoise));
        initialEstimate.insert(0, trans2gtsamPose(m_transform_tobe_mapped));
    }
    else
    {
        noiseModel::Diagonal::shared_ptr odometryNoise = noiseModel::Diagonal::Variances((Vector(6) << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4).finished());
        gtsam::Pose3 poseFrom = pclPointTogtsamPose3(m_cloud_key_poses_6d->points.back());
        gtsam::Pose3 poseTo = trans2gtsamPose(m_transform_tobe_mapped);
        gtSAMgraph.add(BetweenFactor<Pose3>(m_cloud_key_poses_3d->size() - 1, m_cloud_key_poses_3d->size(), poseFrom.between(poseTo), odometryNoise));
        initialEstimate.insert(m_cloud_key_poses_3d->size(), poseTo);
    }
}
gtsam::Pose3 CMapOptimization::pclPointTogtsamPose3(PointTypePose thisPoint)
{
    return gtsam::Pose3(gtsam::Rot3::RzRyRx(double(thisPoint.roll), double(thisPoint.pitch), double(thisPoint.yaw)),
                        gtsam::Point3(double(thisPoint.x), double(thisPoint.y), double(thisPoint.z)));
}

gtsam::Pose3 CMapOptimization::trans2gtsamPose(float transformIn[])
{
    return gtsam::Pose3(gtsam::Rot3::RzRyRx(transformIn[0], transformIn[1], transformIn[2]),
                        gtsam::Point3(transformIn[3], transformIn[4], transformIn[5]));
}
void CMapOptimization::saveKeyFramesAndFactor()
{
    if (saveFrame() == false)
    {
        return;
    }

    // save key poses
    PointType thisPose3D;
    PointTypePose thisPose6D;

#if USE_GTSAM
    // add factor
    addOdomFactor();

    // update iSAM
    isam->update(gtSAMgraph, initialEstimate);

    gtSAMgraph.resize(0);
    initialEstimate.clear();

    Pose3 latestEstimate;
    isamCurrentEstimate = isam->calculateEstimate();
    latestEstimate = isamCurrentEstimate.at<Pose3>(isamCurrentEstimate.size() - 1);

    thisPose3D.x = latestEstimate.translation().x();
    thisPose3D.y = latestEstimate.translation().y();
    thisPose3D.z = latestEstimate.translation().z();
    thisPose3D.intensity = m_cloud_key_poses_3d->size(); // this can be used as index
    m_cloud_key_poses_3d->push_back(thisPose3D);

    thisPose6D.x = thisPose3D.x;
    thisPose6D.y = thisPose3D.y;
    thisPose6D.z = thisPose3D.z;
    thisPose6D.intensity = thisPose3D.intensity; // this can be used as index
    thisPose6D.roll = latestEstimate.rotation().roll();
    thisPose6D.pitch = latestEstimate.rotation().pitch();
    thisPose6D.yaw = latestEstimate.rotation().yaw();
    thisPose6D.time = m_time_laser_info_cur;
    m_cloud_key_poses_6d->push_back(thisPose6D);

    poseCovariance = isam->marginalCovariance(isamCurrentEstimate.size() - 1);
    m_transform_tobe_mapped[0] = latestEstimate.rotation().roll();
    m_transform_tobe_mapped[1] = latestEstimate.rotation().pitch();
    m_transform_tobe_mapped[2] = latestEstimate.rotation().yaw();
    m_transform_tobe_mapped[3] = latestEstimate.translation().x();
    m_transform_tobe_mapped[4] = latestEstimate.translation().y();
    m_transform_tobe_mapped[5] = latestEstimate.translation().z();
#else

    thisPose3D.x = m_transform_tobe_mapped[3];
    thisPose3D.y = m_transform_tobe_mapped[4];
    thisPose3D.z = m_transform_tobe_mapped[5];
    thisPose3D.intensity = m_key_frame_num;
    m_cloud_key_poses_3d->push_back(thisPose3D);

    thisPose6D.x = thisPose3D.x;
    thisPose6D.y = thisPose3D.y;
    thisPose6D.z = thisPose3D.z;
    thisPose6D.intensity = thisPose3D.intensity; // this can be used as index
    thisPose6D.roll = m_transform_tobe_mapped[0];
    thisPose6D.pitch = m_transform_tobe_mapped[1];
    thisPose6D.yaw = m_transform_tobe_mapped[2];
    thisPose6D.time = m_time_laser_info_cur;
    m_cloud_key_poses_6d->push_back(thisPose6D);
#endif

    // ros::Publisher pub_map = pub;
    m_local_mapping->insertKeyFrame(thisPose6D, m_laser_cloud_surf_last_ds);

#if OPEN_DISPLAY_MAP
    m_map_display->pushKeyPoseDeque(thisPose6D);
    m_map_display->pushKeyCloudDeque(*m_laser_cloud_surf_last_ds);
#endif
}

void CMapOptimization::getInitMap(PointTypePose thispose, odom_info odomPre, bool init_flag, pcl::PointCloud<PointType>::Ptr cloud)
{
    if (init_flag)
    {
        PointTypePose thisPose6D;
        thisPose6D.x = 0;
        thisPose6D.y = 0;
        thisPose6D.z = 0;
        thisPose6D.intensity = 0; // this can be used as index
        thisPose6D.roll = 0;
        thisPose6D.pitch = 0;
        thisPose6D.yaw = 0;
        thisPose6D.time = odomPre.odomTimestamp;

        m_transform_tobe_mapped[0] = thispose.roll;
        m_transform_tobe_mapped[1] = thispose.pitch;
        m_transform_tobe_mapped[2] = thispose.yaw;
        m_transform_tobe_mapped[3] = thispose.x;
        m_transform_tobe_mapped[4] = thispose.y;
        m_transform_tobe_mapped[5] = thispose.z;
        // m_local_mapping->insertKeyFrame(thisPose6D, cloud);
    }
}

Eigen::Affine3f CMapOptimization::pclPointToAffine3f(PointTypePose thisPoint)
{ 
    return pcl::getTransformation(thisPoint.x, thisPoint.y, thisPoint.z, thisPoint.roll, thisPoint.pitch, thisPoint.yaw);
}

Eigen::Affine3f CMapOptimization::trans2Affine3f(float transformIn[])
{
    return pcl::getTransformation(transformIn[3], transformIn[4], transformIn[5], transformIn[0], transformIn[1], transformIn[2]);
}

void CMapOptimization::updatePointAssociateToMap()
{
    m_trans_Point_associate_to_map = trans2Affine3f(m_transform_tobe_mapped);
}

void CMapOptimization::surfOptimization()
{
    updatePointAssociateToMap();

    int num_used_by_gn = m_laser_cloud_surf_last_ds_num > m_localmap_cloud_size ? m_localmap_cloud_size : m_laser_cloud_surf_last_ds_num;

    m_local_mapping->setKdtreeBusy();
#pragma omp parallel for num_threads(m_number_of_cores)
    for (int i = 0; i < num_used_by_gn; i++)
    {
        size_t num_results = 5;
        PointType pointOri, pointSel, coeff;
        std::vector<size_t> pointSearchInd(num_results);
        std::vector<double> pointSearchSqDis(num_results);
        vector<PointType> this_points;

        pointOri = m_laser_cloud_surf_last_ds->points[i];
        pointAssociateToMap(&pointOri, &pointSel); 

        const double query_pt[3] = {pointSel.x, pointSel.y, pointSel.z};

        num_results = m_local_mapping->knnSearch(&query_pt[0], num_results, &pointSearchInd[0], &pointSearchSqDis[0]);

        for (int j = 0; j < num_results; j++)
            this_points.push_back(m_local_mapping->getPointFromLocalMap(pointSearchInd[j]));

        Eigen::Matrix<float, 5, 3> matA0;
        Eigen::Matrix<float, 5, 1> matB0;
        Eigen::Vector3f matX0;

        matA0.setZero();
        matB0.fill(-1);
        matX0.setZero();

        double distanceThres = 0.05;
        if (pointSearchSqDis[4] < distanceThres)
        {
            for (int j = 0; j < 5; j++)
            {
                matA0(j, 0) = this_points[j].x;
                matA0(j, 1) = this_points[j].y;
                matA0(j, 2) = this_points[j].z;
            }

            matX0 = matA0.colPivHouseholderQr().solve(matB0);

            float pa = matX0(0, 0);
            float pb = matX0(1, 0);
            float pc = matX0(2, 0);
            float pd = 1;

            float ps = sqrt(pa * pa + pb * pb + pc * pc);
            pa /= ps;
            pb /= ps;
            pc /= ps;
            pd /= ps;

            if(abs(pc) < 0.707) // 45du
            {
                float pdThres = 0.05;
                bool planeValid = true;
                for (int j = 0; j < 5; j++)
                {
                    if (fabs(pa * this_points[j].x +
                            pb * this_points[j].y +
                            pc * this_points[j].z + pd) > pdThres)
                    {
                        planeValid = false;
                        break;
                    }
                }

                if (planeValid)
                {
                    float pd2 = pa * pointSel.x + pb * pointSel.y + pc * pointSel.z + pd;

                    float s = 1 - 0.9 * fabs(pd2) / sqrt(sqrt(pointSel.x * pointSel.x + pointSel.y * pointSel.y + pointSel.z * pointSel.z));

                    coeff.x = s * pa;
                    coeff.y = s * pb;
                    coeff.z = s * pc;
                    coeff.intensity = s * pd2;

                    if (s > 0.85)
                    { // 0.1
                        m_laser_cloud_ori_surf_vec[i] = pointOri;
                        m_coeff_sel_surf_vec[i] = coeff;
                        m_laser_cloud_ori_surf_flag[i] = true;
                    }
                }
            }
        }
    }

    m_local_mapping->setKdtreeIdle();
}

void CMapOptimization::surfOptimizationAndCalculateDeltaXYZ()
{
    updatePointAssociateToMap();

    int num_used_by_gn = m_laser_cloud_surf_last_ds_num > m_localmap_cloud_size ? m_localmap_cloud_size : m_laser_cloud_surf_last_ds_num;

    m_local_mapping->setKdtreeBusy();
    m_sum_dx = 0.0;
    m_sum_dy = 0.0;
    m_sum_dz = 0.0;
    m_cnt_dx = 0;
    m_cnt_dy = 0;
    m_cnt_dz = 0;
    int below_cnt_dx, below_cnt_dy, below_cnt_dz;
#pragma omp parallel for num_threads(m_number_of_cores)
    for (int i = 0; i < num_used_by_gn; i++)
    {
        size_t num_results = 5;
        PointType pointOri, pointSel, coeff;
        std::vector<size_t> pointSearchInd(num_results);
        std::vector<double> pointSearchSqDis(num_results);
        vector<PointType> this_points;

        pointOri = m_laser_cloud_surf_last_ds->points[i];
        pointAssociateToMap(&pointOri, &pointSel);

        const double query_pt[3] = {pointSel.x, pointSel.y, pointSel.z};

        num_results = m_local_mapping->knnSearch(&query_pt[0], num_results, &pointSearchInd[0], &pointSearchSqDis[0]);

        for (int j = 0; j < num_results; j++)
            this_points.push_back(m_local_mapping->getPointFromLocalMap(pointSearchInd[j]));

        Eigen::Matrix<float, 5, 3> matA0;
        Eigen::Matrix<float, 5, 1> matB0;
        Eigen::Vector3f matX0;

        matA0.setZero();
        matB0.fill(-1);
        matX0.setZero();

        double distanceThres = 0.2;
        if (pointSearchSqDis[4] < distanceThres)
        {
            for (int j = 0; j < 5; j++)
            {
                matA0(j, 0) = this_points[j].x;
                matA0(j, 1) = this_points[j].y;
                matA0(j, 2) = this_points[j].z;
            }

            matX0 = matA0.colPivHouseholderQr().solve(matB0);

            float pa = matX0(0, 0);
            float pb = matX0(1, 0);
            float pc = matX0(2, 0);
            float pd = 1;

            float ps = sqrt(pa * pa + pb * pb + pc * pc);
            pa /= ps;
            pb /= ps;
            pc /= ps;
            pd /= ps;

            // std::cout << "-------pc = " << pc << "-------" << std::endl;
            if(abs(pc) < 0.6)
            {
                float pdThres = 0.1;
                bool planeValid = true;
                for (int j = 0; j < 5; j++)
                {
                    if (fabs(pa * this_points[j].x +
                            pb * this_points[j].y +
                            pc * this_points[j].z + pd) > pdThres)
                    {
                        planeValid = false;
                        break;
                    }
                }

                if (planeValid)
                {
                    float pd2 = pa * pointSel.x + pb * pointSel.y + pc * pointSel.z + pd;

                    float s = 1 - 0.9 * fabs(pd2) / sqrt(sqrt(pointSel.x * pointSel.x + pointSel.y * pointSel.y + pointSel.z * pointSel.z));

                    coeff.x = s * pa;
                    coeff.y = s * pb;
                    coeff.z = s * pc;
                    coeff.intensity = s * pd2;

                    if (s > 0.85)
                    { // 0.1
                        m_laser_cloud_ori_surf_vec[i] = pointOri;
                        m_coeff_sel_surf_vec[i] = coeff;
                        m_laser_cloud_ori_surf_flag[i] = true;

                        float dx = 0.2*(this_points[0].x+this_points[1].x+this_points[2].x+this_points[3].x+this_points[4].x)-pointSel.x;
                        float dy = 0.2*(this_points[0].y+this_points[1].y+this_points[2].y+this_points[3].y+this_points[4].y)-pointSel.y;
                        float dz = 0.2*(this_points[0].z+this_points[1].z+this_points[2].z+this_points[3].z+this_points[4].z)-pointSel.z;
                        // std::cout << "-------dy = " << dy << "-------" << std::endl;
                        if (abs(dx) < 0.1 && abs(dy) < 0.1 && abs(dz) < 0.05) {
                            if (abs(dx) > 0.015) {
                                m_sum_dx += dx;
                                m_cnt_dx++;
                            } else {
                                below_cnt_dx++;
                            }
                            if (abs(dy) > 0.015) {
                                m_sum_dy += dy;
                                m_cnt_dy++;
                            } else {
                                below_cnt_dy++;
                            }
                            if (abs(dz) > 0.005) {
                                m_sum_dz += dz;
                                m_cnt_dz++;
                            } else {
                                below_cnt_dz++;
                            }
                        }
                        // std::cout << "dx dy dz " << setprecision(6) << dx << " " << dy << " " << dz << std::endl;
                    }
                }
            }
        }
    }
    std::cout << "m_cnt_dx/dy/dz below middow above "
              << below_cnt_dx << " " << below_cnt_dy << " " << below_cnt_dz << " | "
              << m_cnt_dx << " " << m_cnt_dy << " " << m_cnt_dz << " | "
              << num_used_by_gn - below_cnt_dx - m_cnt_dx << " "
              << num_used_by_gn - below_cnt_dy - m_cnt_dy << " "
              << num_used_by_gn - below_cnt_dz - m_cnt_dz << std::endl;
    std::cout << "m_sum_dx/dy/dz " << setprecision(6) << m_sum_dx << " " << m_sum_dy << " " << m_sum_dz << std::endl;

    if (below_cnt_dx > 0.9 * (below_cnt_dx + m_cnt_dx) || m_cnt_dx < 30)  m_sum_dx = 0.;
    if (below_cnt_dy > 0.9 * (below_cnt_dy + m_cnt_dy) || m_cnt_dy < 30)  m_sum_dy = 0.;
    if (below_cnt_dz > 0.9 * (below_cnt_dz + m_cnt_dz) || m_cnt_dz < 30)  m_sum_dz = 0.;

    m_local_mapping->setKdtreeIdle();
}

void CMapOptimization::pointAssociateToMap(PointType const *const pi, PointType *const po)
{
    po->x = m_trans_Point_associate_to_map(0, 0) * pi->x + m_trans_Point_associate_to_map(0, 1) * pi->y + m_trans_Point_associate_to_map(0, 2) * pi->z + m_trans_Point_associate_to_map(0, 3);
    po->y =  m_trans_Point_associate_to_map(1, 0) * pi->x + m_trans_Point_associate_to_map(1, 1) * pi->y + m_trans_Point_associate_to_map(1, 2) * pi->z + m_trans_Point_associate_to_map(1, 3);
    po->z = m_trans_Point_associate_to_map(2, 0) * pi->x + m_trans_Point_associate_to_map(2, 1) * pi->y + m_trans_Point_associate_to_map(2, 2) * pi->z + m_trans_Point_associate_to_map(2, 3);
    po->intensity = pi->intensity;
}

void CMapOptimization::combineOptimizationCoeffs()
{
    int num_used_by_gn = m_laser_cloud_surf_last_ds_num > m_localmap_cloud_size ? m_localmap_cloud_size : m_laser_cloud_surf_last_ds_num;

    // combine surf coeffs
    for (int i = 0; i < num_used_by_gn; ++i)
    {
        if (m_laser_cloud_ori_surf_flag[i] == true)
        {
            m_laser_cloud_ori->push_back(m_laser_cloud_ori_surf_vec[i]);
            m_coeff_sel->push_back(m_coeff_sel_surf_vec[i]);
        }
    }
    // reset flag for next iteration
    std::fill(m_laser_cloud_ori_surf_flag.begin(), m_laser_cloud_ori_surf_flag.end(), false);
}

bool CMapOptimization::gnOptimization(int iterCount, int maxIter)
{
    // This optimization is from the original loam_velodyne by Ji Zhang, need to cope with coordinate transformation
    // lidar <- camera      ---     camera <- lidar
    // x = z                ---     x = y
    // y = x                ---     y = z
    // z = y                ---     z = x
    // roll = yaw           ---     roll = pitch
    // pitch = roll         ---     pitch = yaw
    // yaw = pitch          ---     yaw = roll

    // lidar -> camera
    float srx = sin(m_transform_tobe_mapped[1]);
    float crx = cos(m_transform_tobe_mapped[1]);
    float sry = sin(m_transform_tobe_mapped[2]);
    float cry = cos(m_transform_tobe_mapped[2]);
    float srz = sin(m_transform_tobe_mapped[0]);
    float crz = cos(m_transform_tobe_mapped[0]);

    int laserCloudSelNum = m_laser_cloud_ori->size();
    m_old_point_ratio = laserCloudSelNum / (double)m_laser_cloud_surf_last_ds_num;
    CLog() << " SAI: ITER" << (iterCount+1) << " laserCloudSelNum/m_old_point_ratio " << laserCloudSelNum << " " << setprecision(6) << m_old_point_ratio << std::endl;
    // CLog::logAsync(LogSara, LogNormal, "[CMapOptimization] surf cloud sel num = %d. \n", laserCloudSelNum);
    if (laserCloudSelNum < m_surf_feature_min_valid_num)
    {
        dlR = 0.;
        dlT = 0.;
        return false;
    }

    cv::Mat matA(laserCloudSelNum, 6, CV_32F, cv::Scalar::all(0));
    cv::Mat matAt(6, laserCloudSelNum, CV_32F, cv::Scalar::all(0));
    cv::Mat matAtA(6, 6, CV_32F, cv::Scalar::all(0));
    cv::Mat matB(laserCloudSelNum, 1, CV_32F, cv::Scalar::all(0));
    cv::Mat matAtB(6, 1, CV_32F, cv::Scalar::all(0));
    cv::Mat matX(6, 1, CV_32F, cv::Scalar::all(0));
    // cv::Mat m_mat_p(6, 6, CV_32F, cv::Scalar::all(0));

    PointType pointOri, coeff;

    for (int i = 0; i < laserCloudSelNum; i++)
    {
        // lidar -> camera
        pointOri.x = m_laser_cloud_ori->points[i].y;
        pointOri.y = m_laser_cloud_ori->points[i].z;
        pointOri.z = m_laser_cloud_ori->points[i].x;
        // lidar -> camera
        coeff.x = m_coeff_sel->points[i].y;
        coeff.y = m_coeff_sel->points[i].z;
        coeff.z = m_coeff_sel->points[i].x;
        coeff.intensity = m_coeff_sel->points[i].intensity;
        // in camera
        float arx = (crx * sry * srz * pointOri.x + crx * crz * sry * pointOri.y - srx * sry * pointOri.z) * coeff.x + (-srx * srz * pointOri.x - crz * srx * pointOri.y - crx * pointOri.z) * coeff.y + (crx * cry * srz * pointOri.x + crx * cry * crz * pointOri.y - cry * srx * pointOri.z) * coeff.z;

        float ary = ((cry * srx * srz - crz * sry) * pointOri.x + (sry * srz + cry * crz * srx) * pointOri.y + crx * cry * pointOri.z) * coeff.x + ((-cry * crz - srx * sry * srz) * pointOri.x + (cry * srz - crz * srx * sry) * pointOri.y - crx * sry * pointOri.z) * coeff.z;

        float arz = ((crz * srx * sry - cry * srz) * pointOri.x + (-cry * crz - srx * sry * srz) * pointOri.y) * coeff.x + (crx * crz * pointOri.x - crx * srz * pointOri.y) * coeff.y + ((sry * srz + cry * crz * srx) * pointOri.x + (crz * sry - cry * srx * srz) * pointOri.y) * coeff.z;
        // lidar -> camera
        matA.at<float>(i, 0) = arz;
        matA.at<float>(i, 1) = arx;
        matA.at<float>(i, 2) = ary;
        matA.at<float>(i, 3) = coeff.z;
        matA.at<float>(i, 4) = coeff.x;
        matA.at<float>(i, 5) = coeff.y;
        matB.at<float>(i, 0) = -coeff.intensity;
    }

    cv::transpose(matA, matAt);
    matAtA = matAt * matA;
    matAtB = matAt * matB;
    cv::solve(matAtA, matAtB, matX, cv::DECOMP_CHOLESKY); // sai: 正定矩阵

    if (!m_is_degenerate) {
        // sai: 如果没有退化，检查是否退化
        cv::Mat matE(1, 6, CV_32F, cv::Scalar::all(0));
        cv::Mat matV(6, 6, CV_32F, cv::Scalar::all(0));
        cv::Mat matV2(6, 6, CV_32F, cv::Scalar::all(0));

        cv::eigen(matAtA, matE, matV);
        matV.copyTo(matV2);

        float eignThre[6] = {100, 100, 100, 100, 80, 10};
        for (int i = 5; i >= 0; i--)
        {
            if (matE.at<float>(0, i) < eignThre[i])
            {
                for (int j = 0; j < 6; j++)
                {
                    matV2.at<float>(i, j) = 0;
                }

                m_is_degenerate = true;
            }
            else
            {
                break;
            }
        }

        m_mat_p = matV.inv() * matV2;
        m_eigenvalue_big_enough = true;
        if (matE.at<float>(0, 0) < 100)
            m_eigenvalue_big_enough = false;
        CLog() << "---------------matE "
                << matE.at<float>(0, 0) << " "
                << matE.at<float>(1, 0) << " "
                << matE.at<float>(2, 0) << " "
                << matE.at<float>(3, 0) << " "
                << matE.at<float>(4, 0) << " "
                << matE.at<float>(5, 0) << std::endl; 
    }

    if (m_is_degenerate)
    {
        cv::Mat matX2(6, 1, CV_32F, cv::Scalar::all(0));
        matX.copyTo(matX2);
        matX = m_mat_p * matX2;
    }

    // sai: check gn result
    cv::Mat matResult(6, 1, CV_32F, cv::Scalar::all(0));
    matResult =  matAtA * matX - matAtB;
    CLog() << " SAI: ITER" << (iterCount+1) << " matResult "
           << matResult.at<float>(0, 0) << " "
           << matResult.at<float>(1, 0) << " "
           << matResult.at<float>(2, 0) << " "
           << matResult.at<float>(3, 0) << " "
           << matResult.at<float>(4, 0) << " "
           << matResult.at<float>(5, 0) << std::endl;                      
    m_gnresult_small_enough = (abs(matResult.at<float>(0, 0)) < 0.8
                                && abs(matResult.at<float>(1, 0)) < 0.8
                                && abs(matResult.at<float>(2, 0)) < 0.8
                                && abs(matResult.at<float>(3, 0)) < 0.8
                                && abs(matResult.at<float>(4, 0)) < 0.8
                                && abs(matResult.at<float>(5, 0)) < 0.8);
    {
        m_transform_tobe_mapped[0] += matX.at<float>(0, 0);
        m_transform_tobe_mapped[1] += matX.at<float>(1, 0);
        m_transform_tobe_mapped[2] += matX.at<float>(2, 0);
        m_transform_tobe_mapped[3] += matX.at<float>(3, 0);
        m_transform_tobe_mapped[4] += matX.at<float>(4, 0);
        m_transform_tobe_mapped[5] += matX.at<float>(5, 0);
    }

    float deltaR = sqrt(
        pow(pcl::rad2deg(matX.at<float>(0, 0)), 2) +
        pow(pcl::rad2deg(matX.at<float>(1, 0)), 2) +
        pow(pcl::rad2deg(matX.at<float>(2, 0)), 2));
    float deltaT = sqrt(
        pow(matX.at<float>(3, 0) * 100, 2) +
        pow(matX.at<float>(4, 0) * 100, 2) +
        pow(matX.at<float>(5, 0) * 100, 2));
    dlR = deltaR;
    dlT = deltaT;

    if (iterCount == maxIter-1)
    {
        if (deltaR < 0.028 && deltaT < 0.025 && abs(m_imu_yaw_vel_deg) < 6) // sai: deltaRT condition
            m_is_divergent = false;
        if (m_gnresult_small_enough && m_old_point_ratio > 0.875) // sai: used point condition
            m_is_divergent = false;
    }
    if (deltaR < 0.01 && deltaT < 0.01) {
        return true; // converged
    }
    return false; // keep optimizing
}

bool CMapOptimization::initialAlignment(int iterCount, const ros::Publisher &pub1, const ros::Publisher &pub2, const ros::Publisher &pub3)
{
    // This optimization is from the original loam_velodyne by Ji Zhang, need to cope with coordinate transformation
    // lidar <- camera      ---     camera <- lidar
    // x = z                ---     x = y
    // y = x                ---     y = z
    // z = y                ---     z = x
    // roll = yaw           ---     roll = pitch
    // pitch = roll         ---     pitch = yaw
    // yaw = pitch          ---     yaw = roll

    // lidar -> camera
    float srx = sin(m_transform_tobe_mapped[1]);
    float crx = cos(m_transform_tobe_mapped[1]);
    float sry = sin(m_transform_tobe_mapped[2]);
    float cry = cos(m_transform_tobe_mapped[2]);
    float srz = sin(m_transform_tobe_mapped[0]);
    float crz = cos(m_transform_tobe_mapped[0]);

    int laserCloudSelNum = m_laser_cloud_ori->size();
    m_old_point_ratio = laserCloudSelNum / (double)m_laser_cloud_surf_last_ds_num;
    CLog() << " SAI: ITER" << (iterCount+1) << " laserCloudSelNum/m_old_point_ratio " << laserCloudSelNum << " " << setprecision(6) << m_old_point_ratio << std::endl;
    // CLog::logAsync(LogSara, LogNormal, "[CMapOptimization] surf cloud sel num = %d. \n", laserCloudSelNum);
    if (laserCloudSelNum < m_surf_feature_min_valid_num)
    {
        dlR = 0.;
        dlT = 0.;
        return false;
    }

    cv::Mat matA(laserCloudSelNum, 6, CV_32F, cv::Scalar::all(0));
    cv::Mat matAt(6, laserCloudSelNum, CV_32F, cv::Scalar::all(0));
    cv::Mat matAtA(6, 6, CV_32F, cv::Scalar::all(0));
    cv::Mat matB(laserCloudSelNum, 1, CV_32F, cv::Scalar::all(0));
    cv::Mat matAtB(6, 1, CV_32F, cv::Scalar::all(0));
    cv::Mat matX(6, 1, CV_32F, cv::Scalar::all(0));
    // cv::Mat m_mat_p(6, 6, CV_32F, cv::Scalar::all(0));

    PointType pointOri, coeff;

    for (int i = 0; i < laserCloudSelNum; i++)
    {
        // lidar -> camera
        pointOri.x = m_laser_cloud_ori->points[i].y;
        pointOri.y = m_laser_cloud_ori->points[i].z;
        pointOri.z = m_laser_cloud_ori->points[i].x;
        // lidar -> camera
        coeff.x = m_coeff_sel->points[i].y;
        coeff.y = m_coeff_sel->points[i].z;
        coeff.z = m_coeff_sel->points[i].x;
        coeff.intensity = m_coeff_sel->points[i].intensity;
        // in camera
        float arx = (crx * sry * srz * pointOri.x + crx * crz * sry * pointOri.y - srx * sry * pointOri.z) * coeff.x + (-srx * srz * pointOri.x - crz * srx * pointOri.y - crx * pointOri.z) * coeff.y + (crx * cry * srz * pointOri.x + crx * cry * crz * pointOri.y - cry * srx * pointOri.z) * coeff.z;

        float ary = ((cry * srx * srz - crz * sry) * pointOri.x + (sry * srz + cry * crz * srx) * pointOri.y + crx * cry * pointOri.z) * coeff.x + ((-cry * crz - srx * sry * srz) * pointOri.x + (cry * srz - crz * srx * sry) * pointOri.y - crx * sry * pointOri.z) * coeff.z;

        float arz = ((crz * srx * sry - cry * srz) * pointOri.x + (-cry * crz - srx * sry * srz) * pointOri.y) * coeff.x + (crx * crz * pointOri.x - crx * srz * pointOri.y) * coeff.y + ((sry * srz + cry * crz * srx) * pointOri.x + (crz * sry - cry * srx * srz) * pointOri.y) * coeff.z;
        // lidar -> camera
        matA.at<float>(i, 0) = arz;
        matA.at<float>(i, 1) = arx;
        matA.at<float>(i, 2) = ary;
        matA.at<float>(i, 3) = coeff.z;
        matA.at<float>(i, 4) = coeff.x;
        matA.at<float>(i, 5) = coeff.y;
        matB.at<float>(i, 0) = -coeff.intensity;
    }

    cv::transpose(matA, matAt);
    matAtA = matAt * matA;
    matAtB = matAt * matB;
    cv::solve(matAtA, matAtB, matX, cv::DECOMP_CHOLESKY); // sai: 正定矩阵

    if (!m_is_degenerate) {
        // sai: 如果没有退化，检查是否退化
        cv::Mat matE(1, 6, CV_32F, cv::Scalar::all(0));
        cv::Mat matV(6, 6, CV_32F, cv::Scalar::all(0));
        cv::Mat matV2(6, 6, CV_32F, cv::Scalar::all(0));

        cv::eigen(matAtA, matE, matV);
        matV.copyTo(matV2);

        float eignThre[6] = {100, 100, 100, 100, 80, 10};
        for (int i = 5; i >= 0; i--)
        {
            if (matE.at<float>(0, i) < eignThre[i])
            {
                for (int j = 0; j < 6; j++)
                {
                    matV2.at<float>(i, j) = 0;
                }

                m_is_degenerate = true;
            }
            else
            {
                break;
            }
        }

        m_mat_p = matV.inv() * matV2;
        m_eigenvalue_big_enough = true;
        if (matE.at<float>(0, 0) < 100)
            m_eigenvalue_big_enough = false;
        CLog() << "---------------matE "
                << matE.at<float>(0, 0) << " "
                << matE.at<float>(1, 0) << " "
                << matE.at<float>(2, 0) << " "
                << matE.at<float>(3, 0) << " "
                << matE.at<float>(4, 0) << " "
                << matE.at<float>(5, 0) << std::endl; 
    }

    if (m_is_degenerate)
    {
        cv::Mat matX2(6, 1, CV_32F, cv::Scalar::all(0));
        matX.copyTo(matX2);
        matX = m_mat_p * matX2;
    }

    // sai: check gn result
    cv::Mat matResult(6, 1, CV_32F, cv::Scalar::all(0));
    matResult =  matAtA * matX - matAtB;
    CLog() << " SAI: ITER" << (iterCount+1) << " matResult "
           << matResult.at<float>(0, 0) << " "
           << matResult.at<float>(1, 0) << " "
           << matResult.at<float>(2, 0) << " "
           << matResult.at<float>(3, 0) << " "
           << matResult.at<float>(4, 0) << " "
           << matResult.at<float>(5, 0) << std::endl;                      
    m_gnresult_small_enough = (abs(matResult.at<float>(0, 0)) < 0.8
                                && abs(matResult.at<float>(1, 0)) < 0.8
                                && abs(matResult.at<float>(2, 0)) < 0.8
                                && abs(matResult.at<float>(3, 0)) < 0.8
                                && abs(matResult.at<float>(4, 0)) < 0.8
                                && abs(matResult.at<float>(5, 0)) < 0.8);

    if (iterCount == 0) {
        if ((m_last_frame_wrong || m_last_frame_bruceAlign ) && m_local_mapping->getKeyFrameNum() > 50 && validFrameCheck(m_laser_cloud_surf_last_ds, 0.3, true) ) {
            float temp_scanMatch_result[6];
            if (scanMatch(m_transform_tobe_mapped, m_laser_cloud_surf_last_ds, temp_scanMatch_result, pub1, pub2, pub3)) {
                m_transform_tobe_mapped[0] = temp_scanMatch_result[0];
                m_transform_tobe_mapped[1] = temp_scanMatch_result[1];
                m_transform_tobe_mapped[2] = temp_scanMatch_result[2];
                m_transform_tobe_mapped[3] = temp_scanMatch_result[3];
                m_transform_tobe_mapped[4] = temp_scanMatch_result[4];
                m_transform_tobe_mapped[5] = temp_scanMatch_result[5];
                CLog() << "SAI: scanMatch result "
                        << m_transform_tobe_mapped[3] << " "
                        << m_transform_tobe_mapped[4] << " "
                        << m_transform_tobe_mapped[5] << " "
                        << m_transform_tobe_mapped[0] << " "
                        << m_transform_tobe_mapped[1] << " "
                        << m_transform_tobe_mapped[2] << " " << std::endl;
            } else {
                if (!m_pure_rotation_flag) {
                    m_transform_tobe_mapped[0] = m_transform_tobe_mapped_back[0];
                    m_transform_tobe_mapped[1] = m_transform_tobe_mapped_back[1];
                    m_transform_tobe_mapped[2] = m_transform_tobe_mapped_back[2];
                    m_transform_tobe_mapped[3] = m_transform_tobe_mapped_back[3] + (m_cnt_dx == 0 ? 0 : m_sum_dx / m_cnt_dx);
                    m_transform_tobe_mapped[4] = m_transform_tobe_mapped_back[4] + (m_cnt_dy == 0 ? 0 : m_sum_dy / m_cnt_dy);
                    m_transform_tobe_mapped[5] = m_transform_tobe_mapped_back[5] + (m_cnt_dz == 0 ? 0 : m_sum_dz / m_cnt_dz);
                } else {
                    m_transform_tobe_mapped[0] = m_transform_tobe_mapped_back[0];
                    m_transform_tobe_mapped[1] = m_transform_tobe_mapped_back[1];
                    m_transform_tobe_mapped[2] = m_transform_tobe_mapped_back[2];
                    m_transform_tobe_mapped[3] = m_transform_tobe_mapped_back[3];
                    m_transform_tobe_mapped[4] = m_transform_tobe_mapped_back[4];
                    m_transform_tobe_mapped[5] = m_transform_tobe_mapped_back[5]; 
                }
            }
        }
        else if (!m_gnresult_small_enough) {
            if (abs(m_imu_yaw_vel_deg) > 6 && m_local_mapping->getKeyFrameNum() > 50 && validFrameCheck(m_laser_cloud_surf_last_ds, 0.3, true) ) {
                float temp_scanMatch_result[6];
                if (scanMatch(m_transform_tobe_mapped, m_laser_cloud_surf_last_ds, temp_scanMatch_result, pub1, pub2, pub3)) {
                    m_transform_tobe_mapped[0] = temp_scanMatch_result[0];
                    m_transform_tobe_mapped[1] = temp_scanMatch_result[1];
                    m_transform_tobe_mapped[2] = temp_scanMatch_result[2];
                    m_transform_tobe_mapped[3] = temp_scanMatch_result[3];
                    m_transform_tobe_mapped[4] = temp_scanMatch_result[4];
                    m_transform_tobe_mapped[5] = temp_scanMatch_result[5];
                    CLog() << "SAI: ScanMatch result "
                            << m_transform_tobe_mapped[3] << " "
                            << m_transform_tobe_mapped[4] << " "
                            << m_transform_tobe_mapped[5] << " "
                            << m_transform_tobe_mapped[0] << " "
                            << m_transform_tobe_mapped[1] << " "
                            << m_transform_tobe_mapped[2] << " " << std::endl;
                } else {
                    if (!m_pure_rotation_flag) {
                        m_transform_tobe_mapped[0] = m_transform_tobe_mapped_back[0];
                        m_transform_tobe_mapped[1] = m_transform_tobe_mapped_back[1];
                        m_transform_tobe_mapped[2] = m_transform_tobe_mapped_back[2];
                        m_transform_tobe_mapped[3] = m_transform_tobe_mapped_back[3] + (m_cnt_dx == 0 ? 0 : m_sum_dx / m_cnt_dx);
                        m_transform_tobe_mapped[4] = m_transform_tobe_mapped_back[4] + (m_cnt_dy == 0 ? 0 : m_sum_dy / m_cnt_dy);
                        m_transform_tobe_mapped[5] = m_transform_tobe_mapped_back[5] + (m_cnt_dz == 0 ? 0 : m_sum_dz / m_cnt_dz);
                    } else {
                        m_transform_tobe_mapped[0] = m_transform_tobe_mapped_back[0];
                        m_transform_tobe_mapped[1] = m_transform_tobe_mapped_back[1];
                        m_transform_tobe_mapped[2] = m_transform_tobe_mapped_back[2];
                        m_transform_tobe_mapped[3] = m_transform_tobe_mapped_back[3];
                        m_transform_tobe_mapped[4] = m_transform_tobe_mapped_back[4];
                        m_transform_tobe_mapped[5] = m_transform_tobe_mapped_back[5]; 
                    }
                }
            } else {
                if (!m_pure_rotation_flag) {
                    m_transform_tobe_mapped[0] = m_transform_tobe_mapped_back[0];
                    m_transform_tobe_mapped[1] = m_transform_tobe_mapped_back[1];
                    m_transform_tobe_mapped[2] = m_transform_tobe_mapped_back[2];
                    m_transform_tobe_mapped[3] = m_transform_tobe_mapped_back[3] + (m_cnt_dx == 0 ? 0 : m_sum_dx / laserCloudSelNum);
                    m_transform_tobe_mapped[4] = m_transform_tobe_mapped_back[4] + (m_cnt_dy == 0 ? 0 : m_sum_dy / laserCloudSelNum);
                    m_transform_tobe_mapped[5] = m_transform_tobe_mapped_back[5] + (m_cnt_dz == 0 ? 0 : m_sum_dz / laserCloudSelNum);
                } else {
                    m_transform_tobe_mapped[0] = m_transform_tobe_mapped_back[0];
                    m_transform_tobe_mapped[1] = m_transform_tobe_mapped_back[1];
                    m_transform_tobe_mapped[2] = m_transform_tobe_mapped_back[2];
                    m_transform_tobe_mapped[3] = m_transform_tobe_mapped_back[3];
                    m_transform_tobe_mapped[4] = m_transform_tobe_mapped_back[4];
                    m_transform_tobe_mapped[5] = m_transform_tobe_mapped_back[5]; 
                }
            }
        }
        else
        {
            if (!m_pure_rotation_flag) {
                m_transform_tobe_mapped[0] = m_transform_tobe_mapped_back[0];
                m_transform_tobe_mapped[1] = m_transform_tobe_mapped_back[1];
                m_transform_tobe_mapped[2] = m_transform_tobe_mapped_back[2];
                m_transform_tobe_mapped[3] = m_transform_tobe_mapped_back[3] + (m_cnt_dx == 0 ? 0 : m_sum_dx / laserCloudSelNum);
                m_transform_tobe_mapped[4] = m_transform_tobe_mapped_back[4] + (m_cnt_dy == 0 ? 0 : m_sum_dy / laserCloudSelNum);
                m_transform_tobe_mapped[5] = m_transform_tobe_mapped_back[5] + (m_cnt_dz == 0 ? 0 : m_sum_dz / laserCloudSelNum);
            } else {
                m_transform_tobe_mapped[0] = m_transform_tobe_mapped_back[0];
                m_transform_tobe_mapped[1] = m_transform_tobe_mapped_back[1];
                m_transform_tobe_mapped[2] = m_transform_tobe_mapped_back[2];
                m_transform_tobe_mapped[3] = m_transform_tobe_mapped_back[3];
                m_transform_tobe_mapped[4] = m_transform_tobe_mapped_back[4];
                m_transform_tobe_mapped[5] = m_transform_tobe_mapped_back[5]; 
            }
        }
        dlR = 0.;
        dlT = 0.;
        return false;
    }
    std::cout << "initialAlignment: WHAT HAPPENED?!" << std::endl;
    return false;
}

void CMapOptimization::finalAlignment()
{
    // sai: 从极端情况到常见情况
    // 优化器选点不足
    if (m_local_mapping->getKeyFrameNum() > 50 && m_old_point_ratio < 0.6 && !m_use_odom) {
        m_transform_tobe_mapped[0] = m_transform_tobe_mapped_back[0];
        m_transform_tobe_mapped[1] = m_transform_tobe_mapped_back[1];
        m_transform_tobe_mapped[2] = m_transform_tobe_mapped_back[2];
        m_transform_tobe_mapped[3] = m_transform_tobe_mapped_back[3];
        m_transform_tobe_mapped[4] = m_transform_tobe_mapped_back[4];
        m_transform_tobe_mapped[5] = m_transform_tobe_mapped_back[5];        
    }
    // 优化收敛但是收敛到错解（优化器失灵）
    if (!m_is_divergent && !m_gnresult_small_enough) {
        if (m_climbing)
        {
            // 爬坡时该情况原因通常为：机器被坡顶住，轮速计输出（向前推）与真实位移（被顶住）不符，使优化器初值较差
            // 如果能检测机器是否被顶住，则此处有更好的方法
            if (m_climbing_harsh)
            {
                m_transform_tobe_mapped[0] = 0.0 * m_transform_tobe_mapped[0] + 1.0 * m_transform_tobe_mapped_back[0];
                m_transform_tobe_mapped[1] = 0.0 * m_transform_tobe_mapped[1] + 1.0 * m_transform_tobe_mapped_back[1];
                m_transform_tobe_mapped[2] = 0.0 * m_transform_tobe_mapped[2] + 1.0 * m_transform_tobe_mapped_back[2];
                m_transform_tobe_mapped[3] = 1.0 * m_transform_tobe_mapped_last[3] + 0.0 * m_transform_tobe_mapped_back[3];
                m_transform_tobe_mapped[4] = 1.0 * m_transform_tobe_mapped_last[4] + 0.0 * m_transform_tobe_mapped_back[4];
                m_transform_tobe_mapped[5] = 1.0 * m_transform_tobe_mapped_last[5] + 0.0 * m_transform_tobe_mapped_back[5];
            }
            else if (m_climbing_harder)
            {
                m_transform_tobe_mapped[0] = 0.0 * m_transform_tobe_mapped[0] + 1.0 * m_transform_tobe_mapped_back[0];
                m_transform_tobe_mapped[1] = 0.0 * m_transform_tobe_mapped[1] + 1.0 * m_transform_tobe_mapped_back[1];
                m_transform_tobe_mapped[2] = 0.0 * m_transform_tobe_mapped[2] + 1.0 * m_transform_tobe_mapped_back[2];
                m_transform_tobe_mapped[3] = 0.65 * m_transform_tobe_mapped_last[3] + 0.35 * m_transform_tobe_mapped_back[3];
                m_transform_tobe_mapped[4] = 0.65 * m_transform_tobe_mapped_last[4] + 0.35 * m_transform_tobe_mapped_back[4];
                m_transform_tobe_mapped[5] = 0.65 * m_transform_tobe_mapped_last[5] + 0.35 * m_transform_tobe_mapped_back[5];
            }
            else
            {
                m_transform_tobe_mapped[0] = 0.0 * m_transform_tobe_mapped[0] + 1.0 * m_transform_tobe_mapped_back[0];
                m_transform_tobe_mapped[1] = 0.0 * m_transform_tobe_mapped[1] + 1.0 * m_transform_tobe_mapped_back[1];
                m_transform_tobe_mapped[2] = 0.0 * m_transform_tobe_mapped[2] + 1.0 * m_transform_tobe_mapped_back[2];
                m_transform_tobe_mapped[3] = 0.35 * m_transform_tobe_mapped_last[3] + 0.65 * m_transform_tobe_mapped_back[3];
                m_transform_tobe_mapped[4] = 0.35 * m_transform_tobe_mapped_last[4] + 0.65 * m_transform_tobe_mapped_back[4];
                m_transform_tobe_mapped[5] = 0.35 * m_transform_tobe_mapped_last[5] + 0.65 * m_transform_tobe_mapped_back[5];
            }
        }
        else
        {
            // 平地上里程计的值虽然不准，但是符合运动学约束，优于优化器输出值
            m_transform_tobe_mapped[0] = m_transform_tobe_mapped_back[0];
            m_transform_tobe_mapped[1] = m_transform_tobe_mapped_back[1];
            m_transform_tobe_mapped[2] = m_transform_tobe_mapped_back[2];
            m_transform_tobe_mapped[3] = m_transform_tobe_mapped_back[3];
            m_transform_tobe_mapped[4] = m_transform_tobe_mapped_back[4];
            m_transform_tobe_mapped[5] = m_transform_tobe_mapped_back[5];
        }
    }
    // 优化发散
    if (m_is_divergent) {
        m_transform_tobe_mapped[0] = m_transform_tobe_mapped_back[0];
        m_transform_tobe_mapped[1] = m_transform_tobe_mapped_back[1];
        m_transform_tobe_mapped[2] = m_transform_tobe_mapped_back[2];
        m_transform_tobe_mapped[3] = m_transform_tobe_mapped_back[3];
        m_transform_tobe_mapped[4] = m_transform_tobe_mapped_back[4];
        m_transform_tobe_mapped[5] = m_transform_tobe_mapped_back[5];
    }
    // 优化收敛，机器纯旋转
    if (!m_is_divergent && m_pure_rotation_flag) {
        m_transform_tobe_mapped[3] = m_transform_tobe_mapped_back[3];
        m_transform_tobe_mapped[4] = m_transform_tobe_mapped_back[4];
        m_transform_tobe_mapped[5] = m_transform_tobe_mapped_back[5];
    }
    // 平地运动
    {
        m_transform_tobe_mapped[0] = constraintTransformation(m_transform_tobe_mapped[0], m_rotation_tollerance);
        m_transform_tobe_mapped[1] = constraintTransformation(m_transform_tobe_mapped[1], m_rotation_tollerance);
        m_transform_tobe_mapped[5] = constraintTransformation(m_transform_tobe_mapped[5], m_z_tollerance);
    }
}

float CMapOptimization::constraintTransformation(float value, float limit)
{
    if (value < -limit)
        value = -limit;
    if (value > limit)
        value = limit;

    return value;
}

bool CMapOptimization::saveFrame()
{
    setRobotPose();
    bool this_frame_wrong = (!m_gnresult_small_enough || m_is_divergent || (m_local_mapping->getKeyFrameNum() > 50 && m_old_point_ratio < 0.6 && !m_use_odom) );
    if (m_local_mapping->getKeyFrameNum() > 100 && m_last_frame_wrong && this_frame_wrong && (m_climbing || abs(m_imu_yaw_vel_deg) > 50) ) {
        CLog() << "saveFrame() -1-1-1-1-1-1-1-1-1-1-1-1-1" << std::endl;
        m_wait_for_bruceAlign = true;
    } else {
        m_wait_for_bruceAlign = false;
    }

    m_last_frame_wrong = this_frame_wrong;
    m_last_frame_use_odom = m_use_odom;
    m_output_pose = false;

    if (!m_local_map_init) {
        CLog() << "saveFrame() 1111111111111" << std::endl;
        m_output_pose = true;
        return true;
    }

    if (m_finish_bruceAlign)
    {
        m_output_pose = false;
        CLog() << "saveFrame() 222222222222222" << std::endl;
        // m_last_frame_bruceAlign--上一帧。 m_finish_bruceAlign--m_finish_bruceAlign当前帧。 m_wait_for_bruceAlign--下一帧。
        CLog() << "saveFrame() m_last_frame_bruceAlign " << m_last_frame_bruceAlign
               << " m_finish_bruceAlign " << m_finish_bruceAlign
               << " m_wait_for_bruceAlign " << m_wait_for_bruceAlign
               << std::endl;
        m_last_frame_bruceAlign = m_finish_bruceAlign;
        return false;
    } else {
        m_last_frame_bruceAlign = m_finish_bruceAlign;
    }

    if (m_is_divergent || !m_eigenvalue_big_enough || abs(m_imu_yaw_vel_deg) > 50
        || m_climbing_harder || !m_gnresult_small_enough || (m_local_mapping->getKeyFrameNum() > 50 && m_old_point_ratio < 0.6 && !m_use_odom) )
    {
        // sai: just the frame passed gnOpt could be added.
        m_output_pose = false;
        CLog() << "saveFrame() 222222222222222" << std::endl;
        CLog() << "saveFrame() m_is_divergent " << m_is_divergent 
                  << " !m_eigenvalue_big_enough " << !m_eigenvalue_big_enough
                  << " abs(m_imu_yaw_vel_deg) > 50 " << (abs(m_imu_yaw_vel_deg) > 50)
                  << " m_use_odom " << m_use_odom
                  << " m_climbing_harder " << m_climbing_harder
                  << " !m_gnresult_small_enough " << !m_gnresult_small_enough
                  << " m_imu_pitch_deg > 3 " << (m_imu_pitch_deg > 3)
                  << " m_imu_roll_deg > 3 " << (m_imu_roll_deg > 3)
                  << " m_old_point_ratio " << setprecision(6) << m_old_point_ratio
                  << std::endl;
        return false;
    }

    // if (m_old_point_ratio > 0.995 && !m_use_odom) {
    //     m_output_pose = true;
    //     CLog() << "saveFrame() 222222222222222" << std::endl;
    //     CLog() << "saveFrame() m_is_divergent " << m_is_divergent 
    //               << " !m_eigenvalue_big_enough " << !m_eigenvalue_big_enough
    //               << " abs(m_imu_yaw_vel_deg) > 50 " << (abs(m_imu_yaw_vel_deg) > 50)
    //               << " m_use_odom " << m_use_odom
    //               << " m_climbing " << m_climbing
    //               << " !m_gnresult_small_enough " << !m_gnresult_small_enough
    //               << " m_imu_pitch_deg > 3 " << (m_imu_pitch_deg > 3)
    //               << " m_imu_roll_deg > 3 " << (m_imu_roll_deg > 3)
    //               << " m_old_point_ratio " << setprecision(6) << m_old_point_ratio
    //               << std::endl;
    //     return false;
    // }

    Eigen::Affine3f transStart = pclPointToAffine3f(m_key_frame_pose_pre);
    Eigen::Affine3f transFinal = pcl::getTransformation(m_transform_tobe_mapped[3], m_transform_tobe_mapped[4], m_transform_tobe_mapped[5],
                                                        m_transform_tobe_mapped[0], m_transform_tobe_mapped[1], m_transform_tobe_mapped[2]);
    Eigen::Affine3f transBetween = transStart.inverse() * transFinal;
    float x, y, z, roll, pitch, yaw;
    pcl::getTranslationAndEulerAngles(transBetween, x, y, z, roll, pitch, yaw);

    // Eigen::Affine3f transStart2 = pcl::getTransformation(m_transform_tobe_mapped_back[3], m_transform_tobe_mapped_back[4], m_transform_tobe_mapped_back[5],
    //                                                      m_transform_tobe_mapped_back[0], m_transform_tobe_mapped_back[1], m_transform_tobe_mapped_back[2]);
    // Eigen::Affine3f transBetween2 = transStart2.inverse() * transFinal;
    // float x2, y2, z2, roll2, pitch2, yaw2;
    // pcl::getTranslationAndEulerAngles(transBetween2, x2, y2, z2, roll2, pitch2, yaw2);

    CLog() << "saveFrame() delta x y z roll pitch yaw " << setprecision(6)
            << x << " " << y << " " << z << " " << roll << " " << pitch << " " << yaw << std::endl;
    if (abs(yaw) < m_keyframe_add_angle_threshold && sqrt(x * x + y * y + z * z) < m_keyframe_add_dist_threshold)
    {
        m_output_pose = true;
        CLog() << "saveFrame() 33333333333333333" << std::endl;
        return false;
    }

    m_output_pose = true;
    CLog() << "saveFrame() 444444444444444444" << std::endl;
    return true;
}

void CMapOptimization::setRobotPose()
{
    PointTypePose robot_pose;
    robot_pose.x = m_transform_tobe_mapped[3];
    robot_pose.y = m_transform_tobe_mapped[4];
    robot_pose.z = m_transform_tobe_mapped[5];
    robot_pose.intensity = m_key_frame_num;
    robot_pose.roll = m_transform_tobe_mapped[0];
    robot_pose.pitch = m_transform_tobe_mapped[1];
    robot_pose.yaw = m_transform_tobe_mapped[2];
    robot_pose.time = m_time_laser_info_cur;
    m_local_mapping->setRobotPose(robot_pose);

#if OPEN_DISPLAY_MAP
    m_map_display->pushSlamPoseDeque(robot_pose);
#endif
}


bool CMapOptimization::validFrameCheck(pcl::PointCloud<PointType>::Ptr couldFrame, const float distance, const bool avoid_ceiling)
{
    int count = 0;
    double dis_valid_point;
    for(int i = 0; i < couldFrame->size(); i++)
    {
        dis_valid_point = sqrt(pow(couldFrame->points[i].x, 2) +
                               pow(couldFrame->points[i].y, 2) +
                               pow(couldFrame->points[i].z, 2));
        if (dis_valid_point < distance) {
            count++;
        } else if (avoid_ceiling) {
            if (couldFrame->points[i].z < 0.10) {
                count++;
            }
        }
    }

    float scale = float(count) / couldFrame->size();
    std::cout << "---------------SCALE " << scale << std::endl;
    if(scale > 0.8)
    {
        return false;
    }

    return true;
}

bool CMapOptimization::not_valid_2DFrameDegCheck(pcl::PointCloud<PointType>::Ptr could2DFrame, double deg_thresh)
{
    double l_deg = 0, r_deg = 0;
    for (int i = 0; i < could2DFrame->size(); i++)
    {
        r_deg = std::min(r_deg, atan2(could2DFrame->points[i].x, could2DFrame->points[i].y) * ARC_TO_DEG);
        l_deg = std::max(l_deg, atan2(could2DFrame->points[i].x, could2DFrame->points[i].y) * ARC_TO_DEG);
    }

    double deg = l_deg - r_deg;
    if (deg < deg_thresh)
    {
        return true;
    }

    return false;
}

void CMapOptimization::setOccupancyGrid(nav_msgs::OccupancyGrid *og)
{
    og->info.resolution = g_resolution;
    og->info.width = g_cell_width;
    og->info.height = g_cell_height;

    og->info.origin.position.x = (-1) * (g_cell_width / 2.0) * g_resolution + g_offset_x;
    og->info.origin.position.y = (-1) * (g_cell_height / 2.0) * g_resolution + g_offset_y;
    og->info.origin.position.z = g_offset_z;
    og->info.origin.orientation.x = 0.0;
    og->info.origin.orientation.y = 0.0;
    og->info.origin.orientation.z = 0.0;
    og->info.origin.orientation.w = 1.0;
}

bool CMapOptimization::scanMatch(const float transformIn[6], const pcl::PointCloud<PointType>::Ptr CloudInput, float (&transformOut)[6], const ros::Publisher &pub, const ros::Publisher &pub1, const ros::Publisher &pub2)
{
    // cur_scan
    PointTypePose thisPose6D;
    thisPose6D.x = transformIn[3];
    thisPose6D.y = transformIn[4];
    thisPose6D.z = transformIn[5];
    thisPose6D.roll = transformIn[0];
    thisPose6D.pitch = transformIn[1];
    thisPose6D.yaw = transformIn[2];
    pcl::PointCloud<PointType>::Ptr Trans2mScan = transformPointCloud(CloudInput, &thisPose6D);
    pcl::PointCloud<PointType>::Ptr this_scan_cloud(new pcl::PointCloud<PointType>());
    sensor_msgs::PointCloud2 cloud2D_currentScan;
    *this_scan_cloud = transform2Scan(Trans2mScan, -0.02, 0.6, true, 0.02); // 0.05
    if (this_scan_cloud->size() < m_surf_feature_min_valid_num)
    {
        m_scan_match_sucess = false;
        return false;
    }
    // if (300 <= this_scan_cloud->size() && this_scan_cloud->size() < 400)
    // {
    //     if (not_valid_2DFrameDegCheck(this_scan_cloud, 70))
    //     {
    //         return false;
    //     }
    // }
    // pcl::toROSMsg(*this_scan_cloud, cloud2D_currentScan);
    // cloud2D_currentScan.header.frame_id = "map";
    // cloud2D_currentScan.header.stamp = ros::Time::now();
    // pub1.publish(cloud2D_currentScan);

    // submap
    {
        m_SubMap->clear();
        m_local_mapping->getLocalMap(m_SubMap);
    }
    pcl::PointCloud<PointType>::Ptr this_submap_cloud(new pcl::PointCloud<PointType>());
    sensor_msgs::PointCloud2 cloud2D_submap;
    *this_submap_cloud = transform2Scan(m_SubMap, -0.02, 0.6, true, 0.05);
    pcl::toROSMsg(*this_submap_cloud, cloud2D_submap);
    cloud2D_submap.header.frame_id = "map";
    cloud2D_submap.header.stamp = ros::Time::now();
    pub.publish(cloud2D_submap);

	// // sai: NDT
    // static pcl::NormalDistributionsTransform<PointType, PointType> ndt;
    // ndt.setResolution(0.05); // 1.0
    // ndt.setMaximumIterations(30);
    // ndt.setTransformationEpsilon(0.005);
    // ndt.setStepSize(0.01); // 0.1

    // sai: ICP2D
    static pcl::IterativeClosestPoint<PointType, PointType> icp;
    pcl::registration::TransformationEstimation2D<PointType, PointType>::Ptr est;
    est.reset(new pcl::registration::TransformationEstimation2D<PointType, PointType>);
    icp.setTransformationEstimation(est);
    icp.setMaxCorrespondenceDistance(0.2);
    icp.setMaximumIterations(20);
    icp.setTransformationEpsilon(0.0001);
    icp.setEuclideanFitnessEpsilon(0.0001);
    icp.setRANSACIterations(0);

    // // sai: NDT2D
    // static pcl::NormalDistributionsTransform2D<PointType, PointType> ndt2d;
    // ndt2d.setMaximumIterations(30);
    // // ndt2d.setGridCentre(Eigen::Vector2f(0,0));
    // // ndt2d.setGridExtent(Eigen::Vector2f(20,20));
    // ndt2d.setGridStep(Eigen::Vector2f(0.05,0.05));
    // ndt2d.setOptimizationStepSize(0.005);
    // ndt2d.setTransformationEpsilon(0.001);

    // Align clouds
    icp.setInputSource(this_scan_cloud);
    icp.setInputTarget(this_submap_cloud);
    pcl::PointCloud<PointType>::Ptr unused_result(new pcl::PointCloud<PointType>());
    // Eigen::Translation3f init_translation(m_transform_tobe_mapped[3], m_transform_tobe_mapped[4], m_transform_tobe_mapped[5]);
    // Eigen::AngleAxisf init_rotation_x(m_transform_tobe_mapped[0], Eigen::Vector3f::UnitX());
    // Eigen::AngleAxisf init_rotation_y(m_transform_tobe_mapped[1], Eigen::Vector3f::UnitY());
    // Eigen::AngleAxisf init_rotation_z(m_transform_tobe_mapped[2], Eigen::Vector3f::UnitZ());
    // Eigen::Matrix4f init_guess = (init_translation * init_rotation_z * init_rotation_y * init_rotation_x).matrix();
    // ndt.align(*unused_result, init_guess);
    icp.align(*unused_result);
    std::cout << __FUNCTION__ << " __DEBUG_LJH__ " << __LINE__ << " SCAN_SCORE = " << icp.getFitnessScore() << std::endl;

    if (icp.hasConverged() == true && icp.getFitnessScore() < 0.005)
    {
        // Get pose transformation
        float x, y, z, roll, pitch, yaw;
        Eigen::Affine3f correctionLidarFrame;
        correctionLidarFrame = icp.getFinalTransformation();

        // sai: TODO CHECK TRANSFORMATION -> icp.getFinalTransformation()
        // sai: TODO 在点云密集时，有大位移和大角度偏差的可能
        float delta_x, delta_y, delta_z, delta_roll, delta_pitch, delta_yaw;
        pcl::getTranslationAndEulerAngles(correctionLidarFrame, delta_x, delta_y, delta_z, delta_roll, delta_pitch, delta_yaw);
        std::cout << " scanMatch delta x y yaw " << setprecision(6) << delta_x << " " << delta_y << " " << delta_yaw << std::endl;
        // CLog::logAsync(LogSara, LogNormal, "[C3DSlam][CMapOptimization] scanMatch score %.6f delta x y z roll pitch yaw %.6f %.6f %.6f %.6f %.6f %.6f \n", icp.getFitnessScore(), delta_x, delta_y, delta_z, delta_roll, delta_pitch, delta_yaw);
        if (sqrt(pow(delta_x,2)+pow(delta_y,2)) > 0.025 || abs(delta_yaw) > 0.05 ) {
            return false;
        }

        pcl::transformPointCloud(*this_scan_cloud, *unused_result, icp.getFinalTransformation());
        sensor_msgs::PointCloud2 cloud_msg;
        pcl::toROSMsg(*unused_result, cloud_msg);
        cloud_msg.header.frame_id = "map";
        pub2.publish(cloud_msg);

        // transform from world origin to wrong pose
        Eigen::Affine3f tWrong = pclPointToAffine3f(thisPose6D);
        // transform from world origin to corrected pose
        Eigen::Affine3f tCorrect = correctionLidarFrame * tWrong; // pre-multiplying -> successive rotation about a fixed frame
        pcl::getTranslationAndEulerAngles(tCorrect, x, y, z, roll, pitch, yaw);

        transformOut[0] = constraintTransformation(roll, m_rotation_tollerance);
        transformOut[1] = constraintTransformation(pitch, m_rotation_tollerance);
        transformOut[2] = yaw;
        transformOut[3] = x;
        transformOut[4] = y;
        transformOut[5] = constraintTransformation(z, m_z_tollerance);
        m_scan_match_sucess = true;
        return true;
    }
    m_scan_match_sucess = false;
    return false;
}

// sai: TODO bruceAlignment()
bool CMapOptimization::bruceAlignment(const ros::Publisher &pub, const ros::Publisher &pub1, const ros::Publisher &pub2)
{
    if(angularBruceAlignment(pub, pub1, pub2) ) {
        return true;
    } else {
        // linearBruceAlignment(pub, pub1, pub2);
        return false;
    }
}

bool CMapOptimization::enoughPointCheck(pcl::PointCloud<PointType>::Ptr couldFrame)
{
    // check point count
    PointTypePose thisPose6D;
    thisPose6D.x = m_transform_tobe_mapped[3];
    thisPose6D.y = m_transform_tobe_mapped[4];
    thisPose6D.z = m_transform_tobe_mapped[5];
    thisPose6D.roll = m_transform_tobe_mapped[0];
    thisPose6D.pitch = m_transform_tobe_mapped[1];
    thisPose6D.yaw = m_transform_tobe_mapped[2];
    pcl::PointCloud<PointType>::Ptr Trans2mScan = transformPointCloud(couldFrame, &thisPose6D);
    pcl::PointCloud<PointType>::Ptr this_scan_cloud(new pcl::PointCloud<PointType>());
    *this_scan_cloud = transform2Scan(Trans2mScan, -0.02, 0.6, true, 0.02); // 0.05  0.5
    std::cout << "this_scan_cloud->size() " << this_scan_cloud->size() << std::endl;

    return (int)this_scan_cloud->size() > 100;
}

bool CMapOptimization::scanMatchForBruceAlignment(const float transformIn[6], const pcl::PointCloud<PointType>::Ptr CloudInput, const pcl::PointCloud<PointType>::Ptr CloudTarget, float (&transformOut)[6], double &final_score)
{
    bool scanmatch_sucess = false;
    double score = 0.0;
    // cur_scan
    PointTypePose thisPose6D;
    thisPose6D.x = transformIn[3];
    thisPose6D.y = transformIn[4];
    thisPose6D.z = transformIn[5];
    thisPose6D.roll = transformIn[0];
    thisPose6D.pitch = transformIn[1];
    thisPose6D.yaw = transformIn[2];

    pcl::PointCloud<PointType>::Ptr Trans2mScan = transformPointCloud(CloudInput, &thisPose6D);
    pcl::PointCloud<PointType>::Ptr this_scan_cloud(new pcl::PointCloud<PointType>());
    sensor_msgs::PointCloud2 cloud2D_currentScan;
    *this_scan_cloud = transform2Scan(Trans2mScan, -0.02, 0.6, true, 0.02); // 0.05  0.5

    pcl::PointCloud<PointType>::Ptr this_submap_cloud(new pcl::PointCloud<PointType>());
    pcl::copyPointCloud(*CloudTarget, *this_submap_cloud);   

    // static pcl::NormalDistributionsTransform<PointType, PointType> ndt;
    // ndt.setResolution(0.05); // 1.0
    // ndt.setMaximumIterations(30);
    // ndt.setTransformationEpsilon(0.005);
    // ndt.setStepSize(0.01); // 0.1

    pcl::IterativeClosestPoint<PointType, PointType> icp;
    pcl::registration::TransformationEstimation2D<PointType, PointType>::Ptr est;
    est.reset(new pcl::registration::TransformationEstimation2D<PointType, PointType>);
    icp.setTransformationEstimation(est);
    icp.setMaxCorrespondenceDistance(0.2);
    icp.setMaximumIterations(15);
    icp.setTransformationEpsilon(0.0001);
    icp.setEuclideanFitnessEpsilon(0.0001);
    icp.setRANSACIterations(0);

    // static pcl::NormalDistributionsTransform2D<PointType, PointType> ndt2d;
    // ndt2d.setMaximumIterations(30);
    // // ndt2d.setGridCentre(Eigen::Vector2f(0,0));
    // ndt2d.setGridExtent(Eigen::Vector2f(20,20));
    // ndt2d.setGridStep(Eigen::Vector2f(0.05,0.05));
    // ndt2d.setOptimizationStepSize(0.005);
    // ndt2d.setTransformationEpsilon(0.001);

    // Align clouds
    icp.setInputSource(this_scan_cloud);
    icp.setInputTarget(this_submap_cloud);
    pcl::PointCloud<PointType>::Ptr unused_result(new pcl::PointCloud<PointType>());
    // Eigen::Translation3f init_translation(m_transform_tobe_mapped[3], m_transform_tobe_mapped[4], m_transform_tobe_mapped[5]);
    // Eigen::AngleAxisf init_rotation_x(m_transform_tobe_mapped[0], Eigen::Vector3f::UnitX());
    // Eigen::AngleAxisf init_rotation_y(m_transform_tobe_mapped[1], Eigen::Vector3f::UnitY());
    // Eigen::AngleAxisf init_rotation_z(m_transform_tobe_mapped[2], Eigen::Vector3f::UnitZ());
    // Eigen::Matrix4f init_guess = (init_translation * init_rotation_z * init_rotation_y * init_rotation_x).matrix();
    // ndt.align(*unused_result, init_guess);
    icp.align(*unused_result);
    score = icp.getFitnessScore();
    // std::cout << __FUNCTION__ << " __DEBUG_LJH__ " << __LINE__ << " SCAN_SCORE = " << score << std::endl;

    // Get pose transformation
    Eigen::Affine3f correctionLidarFrame;
    correctionLidarFrame = icp.getFinalTransformation();
    // Judge
    if (icp.hasConverged() == true && score < 0.005){
        // sai: TODO CHECK TRANSFORMATION -> icp.getFinalTransformation()
        // sai: TODO 在点云密集时，有大位移和大角度偏差的可能
        float delta_x, delta_y, delta_z, delta_roll, delta_pitch, delta_yaw;
        pcl::getTranslationAndEulerAngles(correctionLidarFrame, delta_x, delta_y, delta_z, delta_roll, delta_pitch, delta_yaw);
        // std::cout << " scanMatch score & delta x y yaw " << setprecision(6) << score << " " << delta_x << " " << delta_y << " " << delta_yaw << std::endl;
        // CLog::logAsync(LogSara, LogNormal, "[C3DSlam][CMapOptimization] scanMatch score %.6f delta x y z roll pitch yaw %.6f %.6f %.6f %.6f %.6f %.6f \n", icp.getFitnessScore(), delta_x, delta_y, delta_z, delta_roll, delta_pitch, delta_yaw);
        if (sqrt(pow(delta_x,2)+pow(delta_y,2)) > 0.025 || abs(delta_yaw) > 0.05 ) {
            scanmatch_sucess = false;
        } else {
            scanmatch_sucess = true;
        }
    } else {
        scanmatch_sucess = false;
    }

    // transform from world origin to wrong pose
    Eigen::Affine3f tWrong = pclPointToAffine3f(thisPose6D);
    // transform from world origin to corrected pose
    Eigen::Affine3f tCorrect = correctionLidarFrame * tWrong; // pre-multiplying -> successive rotation about a fixed frame
    float x, y, z, roll, pitch, yaw;
    pcl::getTranslationAndEulerAngles(tCorrect, x, y, z, roll, pitch, yaw);

    transformOut[0] = constraintTransformation(roll, m_rotation_tollerance);
    transformOut[1] = constraintTransformation(pitch, m_rotation_tollerance);
    transformOut[2] = yaw;
    transformOut[3] = x;
    transformOut[4] = y;
    transformOut[5] = constraintTransformation(z, m_z_tollerance);

    unused_result->clear();
    this_scan_cloud->clear();
    this_submap_cloud->clear();
    final_score = score;
    return scanmatch_sucess;
}

bool CMapOptimization::angularBruceAlignment(const ros::Publisher &pub, const ros::Publisher &pub1, const ros::Publisher &pub2)
{
    // sub_map
    pcl::PointCloud<PointType>::Ptr this_sub_map(new pcl::PointCloud<PointType>());
    m_local_mapping->getLocalMap(this_sub_map);
    pcl::PointCloud<PointType>::Ptr this_submap_cloud(new pcl::PointCloud<PointType>());
    *this_submap_cloud = transform2Scan(this_sub_map, -0.02, 0.6, true, 0.05);
    this_sub_map->clear();

    // set search window
    Eigen::Affine3f transTobe = trans2Affine3f(m_transform_tobe_mapped);
    float angular_search_window = 6.0 * DEG_TO_ARC; // 4.5
    int step_count = 5; // 4

    // angular candidate
    std::vector<float> angular_bound;
    std::vector<Eigen::Affine3f> angular_candidate;
    for (int i = 0; i < step_count; i++) {
        float temp_angular_bound = -1.0 * angular_search_window + i * (2.0 * angular_search_window / (step_count - 1) );
        angular_bound.push_back(temp_angular_bound);
        angular_candidate.push_back(
            transTobe * pcl::getTransformation(0., 0., 0., 0., 0., temp_angular_bound)
        );
    }

    // angular search
    std::vector<std::pair<bool, std::pair<double, Eigen::Affine3f> > > angular_search_results(angular_candidate.size() );
#pragma omp parallel for num_threads(m_number_of_cores)
    for (int i = 0; i < (int)angular_candidate.size(); i++) {
        float temp_angular_candidate[6] = {0., 0., 0., 0., 0., 0.};
        pcl::getTranslationAndEulerAngles(angular_candidate[i], temp_angular_candidate[3], temp_angular_candidate[4], temp_angular_candidate[5],
                                            temp_angular_candidate[0], temp_angular_candidate[1], temp_angular_candidate[2]);
        double final_score = 0.0;
        float temp_angular_search_result[6] = {0., 0., 0., 0., 0., 0.};
        // std::cout << "-------------angular search " << i << " start-------------" << std::endl;
        bool is_good = scanMatchForBruceAlignment(temp_angular_candidate, m_laser_cloud_surf_last_ds, this_submap_cloud, temp_angular_search_result, final_score);
        // std::cout << "-------------angular search " << i << " finish:" << is_good << "-------------" << std::endl;
        angular_search_results[i] = std::make_pair(is_good, std::make_pair(final_score, trans2Affine3f(temp_angular_search_result) ) );
    }
    this_submap_cloud->clear();

    // set score theshold
    double angular_score_thres = 0.005;
    float angular_xy_thres = 0.05;

    // sort
    std::sort(angular_search_results.begin(), angular_search_results.end(), [angular_score_thres, transTobe](std::pair<bool, std::pair<double, Eigen::Affine3f> > candidate1, std::pair<bool, std::pair<double, Eigen::Affine3f> > candidate2) {
        if ((candidate1.second).first < angular_score_thres && !((candidate2.second).first < angular_score_thres) ) {
            return true;
        } else if (!((candidate1.second).first < angular_score_thres) && (candidate2.second).first < angular_score_thres) {
            return false;
        } else {
            float dummy1[6];
            float dummy2[6];
            Eigen::Affine3f transBetween1 = transTobe.inverse() * (candidate1.second).second;
            Eigen::Affine3f transBetween2 = transTobe.inverse() * (candidate2.second).second;
            pcl::getTranslationAndEulerAngles(transBetween1, dummy1[3], dummy1[4], dummy1[5], dummy1[0], dummy1[1], dummy1[2]);
            pcl::getTranslationAndEulerAngles(transBetween2, dummy2[3], dummy2[4], dummy2[5], dummy2[0], dummy2[1], dummy2[2]);
            float delta_yaw1 = abs(dummy1[2]);
            float delta_yaw2 = abs(dummy2[2]);
            return delta_yaw1 < delta_yaw2;
        }
    });

    // just debug
    for (int i = 0; i < (int)angular_search_results.size(); i++) {
        float temp_debug[6] = {0., 0., 0., 0., 0., 0.};
        pcl::getTranslationAndEulerAngles(
            (angular_search_results[i].second).second,
            temp_debug[3],
            temp_debug[4],
            temp_debug[5],
            temp_debug[0],
            temp_debug[1],
            temp_debug[2]
        );
        CLog() << "SAI: DEBUG OUTER angular search result "
                << "is_good " << angular_search_results[i].first << " "
                << "score " << (angular_search_results[i].second).first << " "
                << "pose "
                << temp_debug[3] << " "
                << temp_debug[4] << " "
                << temp_debug[5] << " "
                << temp_debug[0] << " "
                << temp_debug[1] << " "
                << temp_debug[2] << " " << std::endl;
    }

    float angular_dummy[6];
    Eigen::Affine3f angular_transBetween = transTobe.inverse() * (angular_search_results.front().second).second;
    pcl::getTranslationAndEulerAngles(angular_transBetween, angular_dummy[3], angular_dummy[4], angular_dummy[5], angular_dummy[0], angular_dummy[1], angular_dummy[2]);
    float final_delta_xy = sqrt(pow(angular_dummy[3], 2) + pow(angular_dummy[4], 2) );

    // transport result
    // 条件比位移搜索更加严格，因为角度搜索成功的点云会被加入地图。而位移搜索不会。
    if ((angular_search_results.front().second).first < angular_score_thres && final_delta_xy < angular_xy_thres) {
        pcl::getTranslationAndEulerAngles(
            (angular_search_results.front().second).second,
            m_transform_tobe_mapped[3],
            m_transform_tobe_mapped[4],
            m_transform_tobe_mapped[5],
            m_transform_tobe_mapped[0],
            m_transform_tobe_mapped[1],
            m_transform_tobe_mapped[2]
        );
        CLog() << "SAI: angular search result "
                << m_transform_tobe_mapped[3] << " "
                << m_transform_tobe_mapped[4] << " "
                << m_transform_tobe_mapped[5] << " "
                << m_transform_tobe_mapped[0] << " "
                << m_transform_tobe_mapped[1] << " "
                << m_transform_tobe_mapped[2] << " " << std::endl;

        // ros
        {
            m_SubMap->clear();
            m_local_mapping->getLocalMap(m_SubMap);
        }
        pcl::PointCloud<PointType>::Ptr this_submap_cloud(new pcl::PointCloud<PointType>());
        sensor_msgs::PointCloud2 cloud2D_submap;
        *this_submap_cloud = transform2Scan(m_SubMap, -0.02, 0.6, true, 0.05);
        pcl::toROSMsg(*this_submap_cloud, cloud2D_submap);
        cloud2D_submap.header.frame_id = "map";
        cloud2D_submap.header.stamp = ros::Time::now();
        pub.publish(cloud2D_submap);

        PointTypePose thisPose6D;
        thisPose6D.x = m_transform_tobe_mapped[3];
        thisPose6D.y = m_transform_tobe_mapped[4];
        thisPose6D.z = m_transform_tobe_mapped[5];
        thisPose6D.roll = m_transform_tobe_mapped[0];
        thisPose6D.pitch = m_transform_tobe_mapped[1];
        thisPose6D.yaw = m_transform_tobe_mapped[2];
        pcl::PointCloud<PointType>::Ptr Trans2mScan = transformPointCloud(m_laser_cloud_surf_last_ds, &thisPose6D);
        pcl::PointCloud<PointType>::Ptr this_scan_cloud(new pcl::PointCloud<PointType>());
        *this_scan_cloud = transform2Scan(Trans2mScan, -0.02, 0.6, true, 0.02); // 0.05  0.5
        sensor_msgs::PointCloud2 cloud_msg;
        pcl::toROSMsg(*this_scan_cloud, cloud_msg);
        cloud_msg.header.frame_id = "map";
        pub2.publish(cloud_msg);

        std::cout << "-------------angular search quit SUCCEEDED -------------" << std::endl;
        return true;
    }
    std::cout << "-------------angular search quit FAILED -------------" << std::endl;
    return false;
}

bool CMapOptimization::linearBruceAlignment(const ros::Publisher &pub, const ros::Publisher &pub1, const ros::Publisher &pub2)
{
    // sub_map
    pcl::PointCloud<PointType>::Ptr this_sub_map(new pcl::PointCloud<PointType>());
    m_local_mapping->getLocalMap(this_sub_map);
    pcl::PointCloud<PointType>::Ptr this_submap_cloud(new pcl::PointCloud<PointType>());
    *this_submap_cloud = transform2Scan(this_sub_map, -0.02, 0.6, true, 0.05);
    this_sub_map->clear();

    // set search window
    Eigen::Affine3f transTobe = trans2Affine3f(m_transform_tobe_mapped);
    int step_count_H = 3; // 向左右,y
    int step_count_V = 3; // 向前后,x
    float linear_search_window_H = 0.05;
    float linear_search_window_V = 0.05;

    // linear candidate
    std::vector<Eigen::Affine3f> linear_candidate;
    for (int i = 0; i < step_count_V; i++) {
        float temp_x_bound = -1.0 * linear_search_window_V + i * (2.0 * linear_search_window_V / (step_count_V - 1) );
        for (int j = 0; j < step_count_H; j++) {
            float temp_y_bound = -1.0 * linear_search_window_H + j * (2.0 * linear_search_window_H / (step_count_H - 1) );
            linear_candidate.push_back(
                transTobe * pcl::getTransformation(temp_x_bound, temp_y_bound, 0., 0., 0., 0.)
            );
        }
    }

    // linear search
    std::vector<std::pair<bool, std::pair<double, Eigen::Affine3f> > > linear_search_results(linear_candidate.size() );
#pragma omp parallel for num_threads(m_number_of_cores)
    for (int i = 0; i < (int)linear_candidate.size(); i++) {
        float temp_linear_candidate[6] = {0., 0., 0., 0., 0., 0.};
        pcl::getTranslationAndEulerAngles(linear_candidate[i], temp_linear_candidate[3], temp_linear_candidate[4], temp_linear_candidate[5],
                                            temp_linear_candidate[0], temp_linear_candidate[1], temp_linear_candidate[2]);
        double final_score = 0.0;
        float temp_linear_search_result[6] = {0., 0., 0., 0., 0., 0.};
        // std::cout << "-------------linear search " << i << " start-------------" << std::endl;
        bool is_good = scanMatchForBruceAlignment(temp_linear_candidate, m_laser_cloud_surf_last_ds, this_submap_cloud, temp_linear_search_result, final_score);
        // std::cout << "-------------linear search " << i << " finish:" << is_good << "-------------" << std::endl;
        linear_search_results[i] = std::make_pair(is_good, std::make_pair(final_score, trans2Affine3f(temp_linear_search_result) ) );
    }
    this_submap_cloud->clear();

    // set score theshold
    double linear_score_thres = 0.01;
    float linear_yaw_thres = 0.05;

    // sort
    std::sort(linear_search_results.begin(), linear_search_results.end(), [linear_score_thres, linear_yaw_thres, transTobe](std::pair<bool, std::pair<double, Eigen::Affine3f> > candidate1, std::pair<bool, std::pair<double, Eigen::Affine3f> > candidate2) {
        if ((candidate1.second).first < linear_score_thres && !((candidate2.second).first < linear_score_thres) ) {
            return true;
        } else if (!((candidate1.second).first < linear_score_thres) && (candidate2.second).first < linear_score_thres) {
            return false;
        } else {
            float dummy1[6];
            float dummy2[6];
            Eigen::Affine3f transBetween1 = transTobe.inverse() * (candidate1.second).second;
            Eigen::Affine3f transBetween2 = transTobe.inverse() * (candidate2.second).second;
            pcl::getTranslationAndEulerAngles(transBetween1, dummy1[3], dummy1[4], dummy1[5], dummy1[0], dummy1[1], dummy1[2]);
            pcl::getTranslationAndEulerAngles(transBetween2, dummy2[3], dummy2[4], dummy2[5], dummy2[0], dummy2[1], dummy2[2]);
            float delta_xy1 = sqrt(pow(dummy1[3], 2) + pow(dummy1[4], 2) );
            float delta_xy2 = sqrt(pow(dummy2[3], 2) + pow(dummy2[4], 2) );
            return delta_xy1 < delta_xy2;
        }
    });

    // just debug
    for (int i = 0; i < (int)linear_search_results.size(); i++) {
        float temp_debug[6] = {0., 0., 0., 0., 0., 0.};
        pcl::getTranslationAndEulerAngles(
            (linear_search_results[i].second).second,
            temp_debug[3],
            temp_debug[4],
            temp_debug[5],
            temp_debug[0],
            temp_debug[1],
            temp_debug[2]
        );
        float dummy[6];
        Eigen::Affine3f final_transBetween = transTobe.inverse() * (linear_search_results[i].second).second;
        pcl::getTranslationAndEulerAngles(final_transBetween, dummy[3], dummy[4], dummy[5], dummy[0], dummy[1], dummy[2]);
        float final_delta_yaw = abs(dummy[2]);
        CLog() << "SAI: DEBUG OUTER linear search result" << (i+1) << " "
                << "is_good " << linear_search_results[i].first << " "
                << "score " << (linear_search_results[i].second).first << " "
                << "delta_yaw " << final_delta_yaw << " "
                << "pose "
                << temp_debug[3] << " "
                << temp_debug[4] << " "
                << temp_debug[5] << " "
                << temp_debug[0] << " "
                << temp_debug[1] << " "
                << temp_debug[2] << " " << std::endl;
    }

    float linear_dummy[6];
    Eigen::Affine3f linear_transBetween = transTobe.inverse() * (linear_search_results.front().second).second;
    pcl::getTranslationAndEulerAngles(linear_transBetween, linear_dummy[3], linear_dummy[4], linear_dummy[5], linear_dummy[0], linear_dummy[1], linear_dummy[2]);
    float final_delta_yaw = abs(linear_dummy[2]);

    // transport result
    if ((linear_search_results.front().second).first < linear_score_thres && final_delta_yaw < linear_yaw_thres) {
        pcl::getTranslationAndEulerAngles(
            (linear_search_results.front().second).second,
            m_transform_tobe_mapped[3],
            m_transform_tobe_mapped[4],
            m_transform_tobe_mapped[5],
            m_transform_tobe_mapped[0],
            m_transform_tobe_mapped[1],
            m_transform_tobe_mapped[2]
        );
        CLog() << "SAI: linear search result "
                << m_transform_tobe_mapped[3] << " "
                << m_transform_tobe_mapped[4] << " "
                << m_transform_tobe_mapped[5] << " "
                << m_transform_tobe_mapped[0] << " "
                << m_transform_tobe_mapped[1] << " "
                << m_transform_tobe_mapped[2] << " " << std::endl;

        // ros
        {
            m_SubMap->clear();
            m_local_mapping->getLocalMap(m_SubMap);
        }
        pcl::PointCloud<PointType>::Ptr this_submap_cloud(new pcl::PointCloud<PointType>());
        sensor_msgs::PointCloud2 cloud2D_submap;
        *this_submap_cloud = transform2Scan(m_SubMap, -0.02, 0.6, true, 0.05);
        pcl::toROSMsg(*this_submap_cloud, cloud2D_submap);
        cloud2D_submap.header.frame_id = "map";
        cloud2D_submap.header.stamp = ros::Time::now();
        pub.publish(cloud2D_submap);

        PointTypePose thisPose6D;
        thisPose6D.x = m_transform_tobe_mapped[3];
        thisPose6D.y = m_transform_tobe_mapped[4];
        thisPose6D.z = m_transform_tobe_mapped[5];
        thisPose6D.roll = m_transform_tobe_mapped[0];
        thisPose6D.pitch = m_transform_tobe_mapped[1];
        thisPose6D.yaw = m_transform_tobe_mapped[2];
        pcl::PointCloud<PointType>::Ptr Trans2mScan = transformPointCloud(m_laser_cloud_surf_last_ds, &thisPose6D);
        pcl::PointCloud<PointType>::Ptr this_scan_cloud(new pcl::PointCloud<PointType>());
        *this_scan_cloud = transform2Scan(Trans2mScan, -0.02, 0.6, true, 0.02); // 0.05  0.5
        sensor_msgs::PointCloud2 cloud_msg;
        pcl::toROSMsg(*this_scan_cloud, cloud_msg);
        cloud_msg.header.frame_id = "map";
        pub2.publish(cloud_msg);

        std::cout << "-------------linear search quit SUCCEEDED -------------" << std::endl;
        return true;
    }
    std::cout << "-------------linear search quit FAILED -------------" << std::endl;
    return false;
}

pcl::PointCloud<PointType> CMapOptimization::transform2Scan(pcl::PointCloud<PointType>::Ptr CloudIn, float z_min, float z_max, bool ds_flag, float ds_cellsize)
{
    pcl::PointCloud<PointType>::Ptr temp_feature_cloud(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr temp_feature_cloud_ds(new pcl::PointCloud<PointType>()); // DEBUG_LJH
    // std::vector<float> scan;
    // scan.resize(360, 0);

    for (size_t i = 0; i < CloudIn->size(); i++)
    {
        //     // if (CloudIn->points[i].z > 0.02 && CloudIn->points[i].z < 0.15)
        if (CloudIn->points[i].z > z_min && CloudIn->points[i].z < z_max)
        {
            double range = sqrt(pow(CloudIn->points[i].x, 2) + pow(CloudIn->points[i].y, 2));
            double angle = atan2(CloudIn->points[i].y, CloudIn->points[i].x); //(-M_PI, M_PI]
            // int index = std::round((angle + M_PI) * 180.0 / M_PI);
            int index = std::round(angle + M_PI / 2);
            // std::cout << "__DEBUG_LJH__"
            //           << " INDEX_POINT = " << index << std::endl;
            // if (index >= 180)
            //     index = index - 180;

            // if (scan[index] == 0 || (range < scan[index] && range > 0))
            {
                // scan[index] = range;
                PointType temp_scanPoint;
                temp_scanPoint.x = CloudIn->points[i].x;
                temp_scanPoint.y = CloudIn->points[i].y;
                temp_scanPoint.z = 4;
                temp_feature_cloud_ds->push_back(temp_scanPoint);
            }
        }
    }
    // std::cout << "__DEBUG_LJH__"
    //           << " remin cloud= " << temp_feature_cloud_ds->size() << std::endl;

    if(ds_flag) {
        pcl::VoxelGrid<PointType> ds_filter_scan;
        ds_filter_scan.setInputCloud(temp_feature_cloud_ds);
        ds_filter_scan.setLeafSize(ds_cellsize, ds_cellsize, ds_cellsize);
        ds_filter_scan.filter(*temp_feature_cloud);        
    } else {
        pcl::copyPointCloud(*temp_feature_cloud_ds, *temp_feature_cloud);
    }
    // std::cout << "__DEBUG_LJH__"
    //           << "After remin cloud= " << temp_feature_cloud->size() << std::endl;
    return *temp_feature_cloud;
}

/*
bool CMapOptimization::PointDistribution(pcl::PointCloud<PointType>::Ptr cloudInput)
{
    float XData = 0.0, YData = 0.0, ZData = 0.0;
    PointType aver;
    double SquareData = 0.0;
    double StandardDeviation = 0.0;
    for (auto iter = cloudInput->begin(); iter != cloudInput->end(); iter++)
    {
        XData += (*iter).x;
        YData += (*iter).y;
        ZData += (*iter).z;
    }
    aver.x = XData / cloudInput->size();
    aver.y = YData / cloudInput->size();
    aver.z = ZData / cloudInput->size();

    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    normals->resize(cloudInput->size());
    // for (auto iter = cloudInput->begin(); iter != cloudInput->end(); iter++)
    // {
    //     SquareData += (aver.x - (*iter).x) * (aver.x - (*iter).x) + (aver.y - (*iter).y) * (aver.y - (*iter).y) + (aver.z - (*iter).z) * (aver.z - (*iter).z);
    // }

    //计算协方差矩阵
    double xx = 0, xy = 0, xz = 0, yy = 0, yz = 0, zz = 0;
    for (auto iter = cloudInput->begin(); iter != cloudInput->end(); iter++)
    {
        xx += ((*iter).x - aver.x) * ((*iter).x - aver.x);
        xy += ((*iter).x - aver.x) * ((*iter).y - aver.y);
        xz += ((*iter).x - aver.x) * ((*iter).z - aver.z);
        yy += ((*iter).y - aver.y) * ((*iter).y - aver.y);
        yz += ((*iter).y - aver.y) * ((*iter).z - aver.z);
        zz += ((*iter).z - aver.z) * ((*iter).z - aver.z);
    }
    //大小为3*3的协方差矩阵
    Eigen::Matrix3f covMat(3, 3);
    covMat(0, 0) = xx / cloudInput->size();
    covMat(0, 1) = covMat(1, 0) = xy / cloudInput->size();
    covMat(0, 2) = covMat(2, 0) = xz / cloudInput->size();
    covMat(1, 1) = yy / cloudInput->size();
    covMat(1, 2) = covMat(2, 1) = yz / cloudInput->size();
    covMat(2, 2) = zz / cloudInput->size();

    //求特征值与特征向量
    Eigen::EigenSolver<Eigen::Matrix3f> es(covMat);
    Eigen::Matrix3f val = es.pseudoEigenvalueMatrix();
    Eigen::Matrix3f vec = es.pseudoEigenvectors();

    std::cout << "cloud_num = " << cloudInput->size() << " val = " << val(0, 0) << " " << val(1, 1) << " " << val(2, 2) << std::endl;

    double pseuSum = val(0, 0) + val(1, 1) + val(2, 2);
    bool b_flag = false;
    bool s_flag = false;
    for (int i = 0; i < 3; ++i)
    {
        if (val(i, i) / pseuSum > 0.5)
        {
            b_flag = true;
            break;
        }
    }
    for (int i = 0; i < 3; ++i)
    {
        if (val(i, i) / pseuSum < 0.02)
        {
            s_flag = true;
            break;
        }
    }

    if (b_flag && s_flag)
        return true;
    // //找到最小特征值t1
    // double t1 = val(0, 0);
    // int ii = 0;
    // if (t1 > val(1, 1))
    // {
    //     ii = 1;
    //     t1 = val(1, 1);
    // }
    // if (t1 > val(2, 2))
    // {
    //     ii = 2;
    //     t1 = val(2, 2);
    // }

    // //最小特征值对应的特征向量v_n
    // Eigen::Vector3f v(vec(0, ii), vec(1, ii), vec(2, ii));
    // //特征向量单位化
    // v /= v.norm();
    // for (int i = 0; i < cloudInput->size(); i++)
    // {
    //     normals->points[i].normal_x = v(0);
    //     normals->points[i].normal_y = v(1);
    //     normals->points[i].normal_z = v(2);
    //     normals->points[i].curvature = t1 / (val(0, 0) + val(1, 1) + val(2, 2));
    // }

    // CLog() << "特征向量 : " << v << std::endl;
    // StandardDeviation = sqrt(SquareData) / cloudInput->size();

    // CLog() << "点云的标准差为： " << StandardDeviation << std::endl;

    // if (StandardDeviation < 0.002) //过于集中
    //     return false;

    return false;
}

void CMapOptimization::setRobotPose()
{
    PointTypePose robot_pose;
    robot_pose.x = m_transform_tobe_mapped[3];
    robot_pose.y = m_transform_tobe_mapped[4];
    robot_pose.z = m_transform_tobe_mapped[5];
    robot_pose.intensity = m_key_frame_num;
    robot_pose.roll = m_transform_tobe_mapped[0];
    robot_pose.pitch = m_transform_tobe_mapped[1];
    robot_pose.yaw = m_transform_tobe_mapped[2];
    robot_pose.time = m_time_laser_info_cur;
    m_local_mapping->setRobotPose(robot_pose);

#if OPEN_DISPLAY_MAP
    m_map_display->pushSlamPoseDeque(robot_pose);
#endif

    // ros pub
    pcl::PointCloud<PointType>::Ptr laser_cloud_surf_global = transformPointCloud(m_laser_cloud_surf_last_ds, &robot_pose);
    std::string frame_id = "map";
    publishCloud(&m_pub_ds_surf_points_opt, laser_cloud_surf_global, ros::Time::now(), frame_id);

    nav_msgs::Odometry laserOdometryROS;
    laserOdometryROS.header.stamp = ros::Time::now();
    laserOdometryROS.header.frame_id = "map";
    laserOdometryROS.pose.pose.position.x = m_transform_tobe_mapped[3];
    laserOdometryROS.pose.pose.position.y = m_transform_tobe_mapped[4];
    laserOdometryROS.pose.pose.position.z = m_transform_tobe_mapped[5];
    laserOdometryROS.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(m_transform_tobe_mapped[0], m_transform_tobe_mapped[1], m_transform_tobe_mapped[2]);
    m_pub_tof_odometry_global.publish(laserOdometryROS);

    // save path for visualization
    updatePath(robot_pose);
}

void CMapOptimization::updatePath(const PointTypePose &pose_in)
{
    geometry_msgs::PoseStamped pose_stamped;
    // pose_stamped.header.stamp = ros::Time().fromSec(pose_in.time);
    pose_stamped.header.stamp = ros::Time::now();
    pose_stamped.header.frame_id = "map";
    pose_stamped.pose.position.x = pose_in.x;
    pose_stamped.pose.position.y = pose_in.y;
    pose_stamped.pose.position.z = pose_in.z;
    tf::Quaternion q = tf::createQuaternionFromRPY(pose_in.roll, pose_in.pitch, pose_in.yaw);
    pose_stamped.pose.orientation.x = q.x();
    pose_stamped.pose.orientation.y = q.y();
    pose_stamped.pose.orientation.z = q.z();
    pose_stamped.pose.orientation.w = q.w();

    // m_global_path.header.stamp = ros::Time().fromSec(pose_in.time);
    m_global_path.header.stamp = ros::Time::now();
    m_global_path.header.frame_id = "map";
    m_global_path.poses.push_back(pose_stamped);
    m_pub_global_path.publish(m_global_path);
}
} //namespace LIOSAM
*/