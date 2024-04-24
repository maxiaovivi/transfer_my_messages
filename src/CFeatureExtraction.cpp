#include "include/CFeatureExtraction.h"

// using namespace LIOSAM;

// NEW:1 OLD:0
#define SELECTION_PARAMETER 1

CFeatureExtraction::CFeatureExtraction()
{
    initializationValue();
}

void CFeatureExtraction::initializationValue()
{
#if SELECTION_PARAMETER
    // sai
    roll_deg_left = 3.1415926;      // 绕X轴（X轴向右，对应于机体-Y轴）
    // pitch_deg_left = -1.142318;  // 绕Y轴（Y轴向下，对应于机体-Z轴）// 114.55度 // 原数值 -1.492680;
    // pitch_deg_left = -1.1693706;    // 绕Y轴（Y轴向下，对应于机体-Z轴）// 113度
    pitch_deg_left = -1.174606587;    // 绕Y轴（Y轴向下，对应于机体-Z轴）// 112.7度
    yaw_deg_left = -3.1415926;     // 绕Z轴（Z轴向前，对应于机体+X轴）
#else 
    //old
    roll_deg_left = 3.13461;      // 绕X轴
    pitch_deg_left = -1.63360525;     // 绕Y轴
    yaw_deg_left = -3.11322;         // 绕Z轴
#endif

    roll_deg_front = 0.0;      // 绕X轴
    pitch_deg_front = 0.0;     // 绕Y轴
    yaw_deg_front = 0.0;         // 绕Z轴

    // 转化为弧度
    double roll_arc_left = roll_deg_left * DEG_TO_ARC;    // 绕X轴
    double pitch_arc_left = pitch_deg_left * DEG_TO_ARC;  // 绕Y轴
    double yaw_arc_left = yaw_deg_left * DEG_TO_ARC;      // 绕Z轴

    double roll_arc_front = roll_deg_front * DEG_TO_ARC;    // 绕X轴
    double pitch_arc_front = pitch_deg_front * DEG_TO_ARC;  // 绕Y轴
    double yaw_arc_front = yaw_deg_front * DEG_TO_ARC;      // 绕Z轴

    // std::cout << "[C3DSlam]" << endl;
    // std::cout << "[C3DSlam]" << "roll_arc = " << roll_arc_left << endl;
    // std::cout << "[C3DSlam]" << "pitch_arc = " << pitch_arc_left << endl;
    // std::cout << "[C3DSlam]" << "yaw_arc = " << yaw_arc_left << endl;

    Eigen::Vector3d euler_angle_left(roll_deg_left, pitch_deg_left, yaw_deg_left);
    Eigen::Vector3d euler_angle_front(roll_deg_front, pitch_deg_front, yaw_deg_front);

    // 使用Eigen库将欧拉角转换为旋转矩阵
    R_left = Eigen::AngleAxisd(euler_angle_left[2], Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(euler_angle_left[1], Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(euler_angle_left[0], Eigen::Vector3d::UnitX());
    
    R_front = Eigen::AngleAxisd(euler_angle_front[2], Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(euler_angle_front[1], Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(euler_angle_front[0], Eigen::Vector3d::UnitX());

#if SELECTION_PARAMETER
    // sai
    // t_left = Eigen::Vector3d(-0.1585431, 0.00553, -0.2326494);
    t_left = Eigen::Vector3d(-0.1625431, 0.00553, -0.2402634232);
#else
    //old
    t_left = Eigen::Vector3d(-0.1606209, -0.0132981, -0.0309949);
#endif

    t_front = Eigen::Vector3d(0.0, 0.0, 0.0);

    // std::cout << "[C3DSlam]" << "\nleft R =\n" << R_left << endl << endl;
    // std::cout << "[C3DSlam]" << "\nleft t =\n" << t_left << endl << endl;
    // std::cout << "[C3DSlam]" << "\nfront R =\n" << R_front << endl << endl;
    // std::cout << "[C3DSlam]" << "\nfront t =\n" << t_front << endl << endl;

    m_cloud_smoothness.resize(N_SCAN_FRONT * H_SCAN_FRONT + N_SCAN_LEFT * H_SCAN_LEFT);

    // m_ori_cloud_front.reset(new pcl::PointCloud<PointType>());
    // m_ori_cloud_left.reset(new pcl::PointCloud<PointType>());
    // m_extracted_cloud.reset(new pcl::PointCloud<PointType>());
    m_surface_cloud.reset(new pcl::PointCloud<PointType>());

    m_cloud_curvature = new float[N_SCAN_FRONT * H_SCAN_FRONT + N_SCAN_LEFT * H_SCAN_LEFT];
    m_cloud_neighbor_picked = new int[N_SCAN_FRONT * H_SCAN_FRONT + N_SCAN_LEFT * H_SCAN_LEFT];
    m_cloud_label = new int[N_SCAN_FRONT * H_SCAN_FRONT + N_SCAN_LEFT * H_SCAN_LEFT];

    m_cloud_info.startRingIndex.resize(N_SCAN_FRONT * H_SCAN_FRONT + N_SCAN_LEFT * H_SCAN_LEFT);
    m_cloud_info.endRingIndex.resize(N_SCAN_FRONT * H_SCAN_FRONT + N_SCAN_LEFT * H_SCAN_LEFT);
    m_cloud_info.pointColInd.resize(N_SCAN_FRONT * H_SCAN_FRONT + N_SCAN_LEFT * H_SCAN_LEFT);
    m_cloud_info.pointRange.resize(N_SCAN_FRONT * H_SCAN_FRONT + N_SCAN_LEFT * H_SCAN_LEFT);
}

void CFeatureExtraction::tofCloudInfoHandler(const pcl::PointCloud<PointType>::Ptr tofCloudFront,
                                             const pcl::PointCloud<PointType>::Ptr tofCloudLeft,
                                             const double tofTimestampFrint,
                                             const double tofTimestampLeft,
                                             const double curTimestamp,
                                             const std::deque<struct imu_info> imuInfoDeque)
{
    m_surface_cloud->clear();

    pcl::PointCloud<PointType>::Ptr this_ori_cloud_front(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr this_ori_cloud_left(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr this_extracted_cloud(new pcl::PointCloud<PointType>());

    pcl::copyPointCloud(*tofCloudFront, *this_ori_cloud_front);
    pcl::copyPointCloud(*tofCloudLeft, *this_ori_cloud_left);

    double thisTimestampFrint = tofTimestampFrint;
    double thisTimestampLeft = tofTimestampLeft;

    cloudExtraction(this_extracted_cloud, this_ori_cloud_front, this_ori_cloud_left, tofTimestampFrint, tofTimestampLeft, curTimestamp, imuInfoDeque);
    LIOSAM::CLog() << "SAI: AFTER cloudExtraction() cloud.size " << this_extracted_cloud->size() << std::endl;

    pcl::copyPointCloud(*this_extracted_cloud, *m_surface_cloud);
    
    // calculateSmoothness(this_extracted_cloud);
    // LIOSAM::CLog() << "SAI: AFTER calculateSmoothness() cloud.size " << this_extracted_cloud->size() << std::endl;
    // extractFeatures(this_extracted_cloud);
    // LIOSAM::CLog() << "SAI: AFTER extractFeatures() cloud.size " << m_surface_cloud->size() << std::endl;
}

void CFeatureExtraction::cloudExtraction(pcl::PointCloud<PointType>::Ptr extractedCloud,
                                         pcl::PointCloud<PointType>::Ptr tofCloudFront,
                                         pcl::PointCloud<PointType>::Ptr tofCloudLeft,
                                         double tofTimestampFrint,
                                         double tofTimestampLeft,
                                         double curTimestamp,
                                         const std::deque<struct imu_info> imuInfoDeque)
{
    int count = 0, index = 0;
    // front
    for (int v = 0; v < N_SCAN_FRONT; v += 1)
    {
        m_cloud_info.startRingIndex[v] = count - 1 + 5; // point's index
        for (int u = 15; u < H_SCAN_FRONT-15; u++)
        {
            index = v * H_SCAN_FRONT + u;
            // const float range = tofCloudFront->points[index].x;
            const float range = sqrt(tofCloudFront->points[index].x * tofCloudFront->points[index].x +
                                     tofCloudFront->points[index].y * tofCloudFront->points[index].y +
                                     tofCloudFront->points[index].z * tofCloudFront->points[index].z);

            if (range > 0.05 && range < 8.0)
            // if (range > 0.3 && range < 7.0)
            {
                Eigen::Vector3d point(tofCloudFront->points[index].x, tofCloudFront->points[index].y, tofCloudFront->points[index].z);
                Eigen::Vector3d toOdomPoint = R_front * point + t_front;
                PointType thisPoint;
                // sai
                // thisPoint.x = toOdomPoint(2) + 0.16023;    // 160.23mm
                thisPoint.x = toOdomPoint(2) + 0.173;
                thisPoint.y = -toOdomPoint(0) - 0.0;
                thisPoint.z = -toOdomPoint(1) - 0.0;

                if (thisPoint.z > -0.03 && thisPoint.z < 2.0) // point on the floor
                {
                    thisPoint = deskewPoint(thisPoint, tofTimestampFrint, curTimestamp, imuInfoDeque);

                    m_cloud_info.pointColInd[count] = u;
                    m_cloud_info.pointRange[count] = range;
                    extractedCloud->push_back(thisPoint);
                    ++count;
                }
            }
        } // for all u
        m_cloud_info.endRingIndex[v] = count - 1 - 5;
    }

    // left
    // 新模组点数过多，这里改为每读一行丢一行
    for (int v = 0; v < N_SCAN_LEFT; v += 1) 
    {
        // if (v % 2 == 0) {
        //     continue;
        // }
        m_cloud_info.startRingIndex[N_SCAN_FRONT + v] = count - 1 + 5;
        for (int u = 15; u < H_SCAN_LEFT-15; u += 1)
        {
            index = v * H_SCAN_LEFT + u;
            // float range = tofCloudLeft->points[index].x;
            float range = sqrt(tofCloudLeft->points[index].x * tofCloudLeft->points[index].x +
                               tofCloudLeft->points[index].y * tofCloudLeft->points[index].y +
                               tofCloudLeft->points[index].z * tofCloudLeft->points[index].z);
            if (range > 0.05 && range < 8.0)
            {
                Eigen::Vector3d point(tofCloudLeft->points[index].x, tofCloudLeft->points[index].y, tofCloudLeft->points[index].z);
                Eigen::Vector3d toOdomPoint = R_left * point + t_left;
                PointType thisPoint;
                // sai
                // thisPoint.x = toOdomPoint(2) + 0.16023;    // 160.23mm
                thisPoint.x = toOdomPoint(2) + 0.173;
                thisPoint.y = -toOdomPoint(0) - 0.0;
                thisPoint.z = -toOdomPoint(1) - 0.0;

                if (thisPoint.z > -0.03 && thisPoint.z < 2.0)
                {
                    thisPoint = deskewPoint(thisPoint, tofTimestampLeft, curTimestamp, imuInfoDeque);

                    m_cloud_info.pointColInd[count] = u;
                    m_cloud_info.pointRange[count] = range;
                    extractedCloud->push_back(thisPoint);
                    ++count;
                }
            }
        } // for all u
        m_cloud_info.endRingIndex[N_SCAN_FRONT + v] = count - 1 - 5;
    }
}

void CFeatureExtraction::calculateSmoothness(pcl::PointCloud<PointType>::Ptr extractedCloud)
{
    int cloudSize = extractedCloud->points.size();
    for (int i = 5; i < cloudSize - 5; i++)
    {
        float diffRange = m_cloud_info.pointRange[i-5] + m_cloud_info.pointRange[i-4]
                        + m_cloud_info.pointRange[i-3] + m_cloud_info.pointRange[i-2]
                        + m_cloud_info.pointRange[i-1] - m_cloud_info.pointRange[i] * 10
                        + m_cloud_info.pointRange[i+1] + m_cloud_info.pointRange[i+2]
                        + m_cloud_info.pointRange[i+3] + m_cloud_info.pointRange[i+4]
                        + m_cloud_info.pointRange[i+5];            

        m_cloud_curvature[i] = diffRange * diffRange; // diffX * diffX + diffY * diffY + diffZ * diffZ;

        m_cloud_neighbor_picked[i] = 0;
        m_cloud_label[i] = 0;
        // m_cloud_smoothness for sorting
        m_cloud_smoothness[i].value = m_cloud_curvature[i];
        m_cloud_smoothness[i].ind = i;
    }
}

void CFeatureExtraction::extractFeatures(pcl::PointCloud<PointType>::Ptr extractedCloud)
{
    pcl::PointCloud<PointType>::Ptr surfaceCloudScan(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr surfaceCloudScanDS(new pcl::PointCloud<PointType>());

    // sai debug
    int edge_num = 0;
    int surf_num = 0;

    for (int i = 0; i < N_SCAN_FRONT + N_SCAN_LEFT; i++)
    {
        surfaceCloudScan->clear();

        for (int j = 0; j < 2; j++)
        {
            int sp = (m_cloud_info.startRingIndex[i] * (2 - j) + m_cloud_info.endRingIndex[i] * j) / 2;
            int ep = (m_cloud_info.startRingIndex[i] * (1 - j) + m_cloud_info.endRingIndex[i] * (j + 1)) / 2 - 1;

            // int sp = m_cloud_info.startRingIndex[i];
            // int ep = m_cloud_info.endRingIndex[i];

            if (sp >= ep)
                continue;

            std::sort(m_cloud_smoothness.begin() + sp, m_cloud_smoothness.begin() + ep, by_value());

            int largestPickedNum = 0;
            for (int k = ep; k >= sp; k--)
            {
                int ind = m_cloud_smoothness[k].ind;
                if (m_cloud_neighbor_picked[ind] == 0 && m_cloud_curvature[ind] > m_edge_threshold)
                {
                    edge_num++;
                    largestPickedNum++;
                    if (largestPickedNum <= 20)
                    {
                        m_cloud_label[ind] = 1;
                    }
                    else
                    {
                        break;
                    }

                    m_cloud_neighbor_picked[ind] = 1;
                    for (int l = 1; l <= 5; l++)
                    {
                        int columnDiff = std::abs(int(m_cloud_info.pointColInd[ind + l] - m_cloud_info.pointColInd[ind + l - 1]));
                        if (columnDiff > 10)
                            break;
                        m_cloud_neighbor_picked[ind + l] = 1;
                    }
                    for (int l = -1; l >= -5; l--)
                    {
                        int columnDiff = std::abs(int(m_cloud_info.pointColInd[ind + l] - m_cloud_info.pointColInd[ind + l + 1]));
                        if (columnDiff > 10)
                            break;
                        m_cloud_neighbor_picked[ind + l] = 1;
                    }
                }
            }

            for (int k = sp; k <= ep; k++)
            {
                int ind = m_cloud_smoothness[k].ind;
                if (m_cloud_neighbor_picked[ind] == 0 && m_cloud_curvature[ind] < m_surf_threshold)
                {
                    surf_num++;
                    m_cloud_label[ind] = -1;
                    m_cloud_neighbor_picked[ind] = 1;

                    for (int l = 1; l <= 5; l++)
                    {

                        int columnDiff = std::abs(int(m_cloud_info.pointColInd[ind + l] - m_cloud_info.pointColInd[ind + l - 1]));
                        if (columnDiff > 10)
                            break;

                        m_cloud_neighbor_picked[ind + l] = 1;
                    }
                    for (int l = -1; l >= -5; l--)
                    {

                        int columnDiff = std::abs(int(m_cloud_info.pointColInd[ind + l] - m_cloud_info.pointColInd[ind + l + 1]));
                        if (columnDiff > 10)
                            break;

                        m_cloud_neighbor_picked[ind + l] = 1;
                    }
                }
            }

            for (int k = sp; k <= ep; k++)
            {
                if (m_cloud_label[k] <= 0)
                {
                    surfaceCloudScan->push_back(extractedCloud->points[k]);
                }
            }
        }

        *m_surface_cloud += *surfaceCloudScan;
    }
    LIOSAM::CLog() << "SAI: edge/surf point num " << edge_num << " " << surf_num << std::endl;
}

pcl::PointCloud<PointType> CFeatureExtraction::getFeatureCloud()
{
    return *m_surface_cloud;
}

PointType CFeatureExtraction::deskewPoint(const PointType points,
                                          double pointsTimestamp,
                                          double targetTimestamp,
                                          const std::deque<struct imu_info> imuInfoDeque)
{
    // sai:
    // targetTimestamp - 当前帧里程计时间，以此为基准
    // pointsTimestamp - 当前帧点云时间。数据传输算法可以保证：前向和侧向点云一个略晚于里程计时间（不超过50ms），另一个略早于里程计时间（不超过10ms）
    double dur_time = pointsTimestamp - targetTimestamp;
    // std::cout << "dur_time = " << dur_time << std::endl;
    double roll_sum = 0;
    double pitch_sum = 0;
    double yaw_sum = 0;
    int size_imu_deque = 0;
    double startTimestamp = pointsTimestamp < targetTimestamp ? pointsTimestamp : targetTimestamp;
    double endTimestamp = pointsTimestamp > targetTimestamp ? pointsTimestamp : targetTimestamp;

    for (int i = 0; i < imuInfoDeque.size(); ++i)
    {
        if (imuInfoDeque[i].imuTimestamp < startTimestamp || imuInfoDeque[i].imuTimestamp > endTimestamp)
            continue;
        roll_sum = imuInfoDeque[i].imuRollVel + roll_sum;
        pitch_sum = imuInfoDeque[i].imuPitchVel + pitch_sum;
        yaw_sum = imuInfoDeque[i].imuYawVel + yaw_sum;
        size_imu_deque++;
    }

    if (size_imu_deque < 1)
        return points;

    double dur_roll = roll_sum / size_imu_deque * dur_time;
    double dur_pitch = pitch_sum / size_imu_deque * dur_time;
    double dur_yaw = yaw_sum / size_imu_deque * dur_time;

    Eigen::Matrix3d R_mat;
    Eigen::Vector3d t_mat;
    Eigen::Vector3d euler_angle(dur_roll, dur_pitch, dur_yaw);
    R_mat = Eigen::AngleAxisd(euler_angle[2], Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(euler_angle[1], Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(euler_angle[0], Eigen::Vector3d::UnitX());

    t_mat = Eigen::Vector3d(0, 0, 0);

    // LIOSAM::CLog() << "SAI: START deskewPoint. size_imu_deque " << size_imu_deque << std::endl;

    Eigen::Vector3d point(points.x, points.y, points.z);
    Eigen::Vector3d deskew_point = R_mat * point + t_mat;
    PointType this_point;
    this_point.x = deskew_point(0);
    this_point.y = deskew_point(1);
    this_point.z = deskew_point(2);
    return this_point;
}