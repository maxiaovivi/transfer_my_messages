#include "include/CLocalMapping.h"
#include "include/myicp.h"
#include "include/myicp_helpers.h"
#include <unistd.h>
// using namespace LIOSAM;
namespace LIOSAM
{
    CLocalMapping::CLocalMapping(ros::NodeHandle nnh) : nh(nnh)
    {
	    // sai: exit safe
	    m_exit = false;

        m_kdtree_idle = true;
        m_loop_build = 1;
        m_init = false;
        m_waiting_key_frames = false;
        m_local_map_num = 0;
        m_Keyframe_num = 0;
        m_kdtree_nano = NULL;
        m_thin_map_flag = false;

        scManager = new CScancontext();
        scManager->loadPoses();
        //sleep(5);
        scManager->loadAllSCD();

        
        // m_octo_tree = new octomap::OcTree(0.1);

        m_tof_cloud_surf_global_map.reset(new pcl::PointCloud<PointType>());        
        m_tof_cloud_surf_local_map.reset(new pcl::PointCloud<PointType>());
        m_tof_cloud_surf_thin_map.reset(new pcl::PointCloud<PointType>());
        m_key_frame_poses_6d.reset(new pcl::PointCloud<PointTypePose>());
        // m_frame_poses_6d.reset(new pcl::PointCloud<PointTypePose>());

        m_kdtree_nano_1 = new KDTreeNano(3, m_local_map_1, KDTreeSingleIndexAdaptorParams(10));
        m_kdtree_nano_2 = new KDTreeNano(3, m_local_map_2, KDTreeSingleIndexAdaptorParams(10));

        // m_app_down_size_filter_surf.setLeafSize(0.05, 0.05, 0.05);

        // ros
        pub_global_map = nh.advertise<sensor_msgs::PointCloud2>("global_map", 1);
        pub_icp = nh.advertise<sensor_msgs::PointCloud2>("key_frame_icp", 1);
    }

    CLocalMapping::~CLocalMapping()
    {

	    m_exit = true;
        delete m_kdtree_nano_1;
        delete m_kdtree_nano_2;
        // delete m_octo_tree;
        free(scManager);
	    std::cout << "[C3DSLAM] EXIT CLocalMapping" << std::endl;
    }

    bool CLocalMapping::isInit()
    {
        std::lock_guard<std::mutex> lock5(m_mtx_init);
        bool return_init = m_init;
        return return_init;
    }

    void CLocalMapping::setInit(bool flag)
    {
        std::lock_guard<std::mutex> lock5(m_mtx_init);
        m_init = flag;
    }

    void CLocalMapping::resetMap()
    {
        setInit(false);
        clearGlobalMap();
        clearKeyFrames();
    }

    void CLocalMapping::addWaitingKeyFrame()
    {
        std::lock_guard<std::mutex> lock6(m_mtx_waiting_key_frame);
        m_waiting_key_frames = true;
    }

    void CLocalMapping::subWaitingKeyFrame()
    {
        std::lock_guard<std::mutex> lock6(m_mtx_waiting_key_frame);
        m_waiting_key_frames = false;
    }

    void CLocalMapping::getPreOdomfromSys(odom_info pose)
    {
        std::lock_guard<std::mutex> lock9(m_mtx_odom_pose);
        m_odom_pose_deque.push_back(pose);
        if (m_odom_pose_deque.size() > 100)
        {
            m_odom_pose_deque.pop_front();
        }
    }

    void CLocalMapping::insertKeyFrame(PointTypePose pose, pcl::PointCloud<PointType>::Ptr cloud)
    {
        std::lock_guard<std::mutex> lock2(m_mtx_key_pose);
        std::lock_guard<std::mutex> lock3(m_mtx_key_point);

        PointTypePose thisPose6D;
        thisPose6D.x = pose.x;
        thisPose6D.y = pose.y;
        thisPose6D.z = pose.z;
        thisPose6D.intensity = pose.intensity;
        thisPose6D.roll = pose.roll;
        thisPose6D.pitch = pose.pitch;
        thisPose6D.yaw = pose.yaw;
        thisPose6D.time = pose.time;
        m_key_frame_poses_6d->push_back(thisPose6D);

        m_key_frame_pose_pre = thisPose6D;
        m_Keyframe_num = m_key_frame_poses_6d->size();

        pcl::PointCloud<PointType>::Ptr this_surf_key_frame(new pcl::PointCloud<PointType>());
        pcl::copyPointCloud(*cloud, *this_surf_key_frame);

        if (m_key_frame_points.size() > 5)
            m_key_frame_points.pop_front();
        m_key_frame_points.push_back(this_surf_key_frame);

        // pcl::PointCloud<PointType> laser_cloud_surf_temp = *transformPointCloud(this_surf_key_frame, &thisPose6D);
        // *m_tof_cloud_surf_global_map   += laser_cloud_surf_temp;

        addWaitingKeyFrame();
    }



    void CLocalMapping::getKeyPosesPre(PointTypePose *pose)
    {
        *pose = m_key_frame_pose_pre;
    }

    size_t CLocalMapping::getKeyFrameNum()
    {
        std::lock_guard<std::mutex> lock2(m_mtx_key_pose);
        int return_num = m_Keyframe_num;
        return return_num;
    }

    // size_t CLocalMapping::getFrameNum()
    // {
    //     CLog() << __FUNCTION__ << " start" << std::endl;

    //     std::lock_guard<std::mutex> lock7(m_mtx_robot_pose);

    //     return m_frame_poses_6d->size();
    // }

    size_t CLocalMapping::getlLocalMapNum()
    {
        std::lock_guard<std::mutex> lock8(m_mtx_map_num);

        size_t return_num = m_local_map_num;
        return return_num;
    }

    size_t CLocalMapping::getGlobalMapNum()
    {
        std::lock_guard<std::mutex> lock4(m_mtx_map);

        size_t return_num = m_tof_cloud_surf_global_map->points.size();
        return return_num;
    }

    void CLocalMapping::setlLocalMapNum(size_t num)
    {
        std::lock_guard<std::mutex> lock8(m_mtx_map_num);

        m_local_map_num = num;
    }

    size_t CLocalMapping::knnSearch(const double *query,
                                    const size_t results,
                                    size_t *pointSearchInd,
                                    double *pointSearchSqDis)
    {
        size_t num_results = m_kdtree_nano->knnSearch(query, results, pointSearchInd, pointSearchSqDis);

        return num_results;
    }

    PointType CLocalMapping::getPointFromLocalMap(size_t index)
    {
        std::lock_guard<std::mutex> lock1(m_mtx_kdtree);

        PointType this_point;
        this_point.x = m_local_map.pts[index].x;
        this_point.y = m_local_map.pts[index].y;
        this_point.z = m_local_map.pts[index].z;

        return this_point;
    }

    // PointTypePose CLocalMapping::getPoseFromFrames(size_t index)
    // {
    //     std::lock_guard<std::mutex> lock7(m_mtx_robot_pose);
    //     return m_frame_poses_6d->points[index];
    // }

    PointTypePose CLocalMapping::getPoseFromKeyFrames(size_t index)
    {
        std::lock_guard<std::mutex> lock2(m_mtx_key_pose);

        return m_key_frame_poses_6d->points[index];
    }

    void CLocalMapping::clearKeyFrames()
    {
        std::lock_guard<std::mutex> lock2(m_mtx_key_pose);
        m_key_frame_poses_6d->clear();
    }

    pcl::PointCloud<PointType> CLocalMapping::getCloudFromKeyFrames(int index)
    {
        std::lock_guard<std::mutex> lock3(m_mtx_key_point);

        return *m_key_frame_points[index];
    }

    bool CLocalMapping::checkNewKeyFrames()
    {
        std::lock_guard<std::mutex> lock6(m_mtx_waiting_key_frame);

        return m_waiting_key_frames;
    }

    void CLocalMapping::getGlobalMap(pcl::PointCloud<PointType>::Ptr this_global_map)
    {
        std::lock_guard<std::mutex> lock4(m_mtx_map);

        pcl::copyPointCloud(*m_tof_cloud_surf_global_map, *this_global_map);
    }

    void CLocalMapping::getLocalMap(pcl::PointCloud<PointType>::Ptr local_map)
    {
        std::lock_guard<std::mutex> lock4(m_mtx_map);

        pcl::copyPointCloud(*m_tof_cloud_surf_local_map, *local_map);
    }

    void CLocalMapping::updataLocalMap(pcl::PointCloud<PointType>::Ptr local_map)
    {
        std::lock_guard<std::mutex> lock4(m_mtx_map);

        m_tof_cloud_surf_local_map->clear();
        pcl::copyPointCloud(*local_map, *m_tof_cloud_surf_local_map);
    }

    void CLocalMapping::clearGlobalMap()
    {
        std::lock_guard<std::mutex> lock4(m_mtx_map);

        m_tof_cloud_surf_global_map->clear();
    }

    void CLocalMapping::updataGlobalMap(pcl::PointCloud<PointType>::Ptr global_map_ds)
    {
        std::lock_guard<std::mutex> lock4(m_mtx_map);

        m_tof_cloud_surf_global_map->clear();
        pcl::copyPointCloud(*global_map_ds, *m_tof_cloud_surf_global_map);
    }

    void CLocalMapping::globalMapAddKeyCloud(pcl::PointCloud<PointType>::Ptr key_cloud)
    {
        std::lock_guard<std::mutex> lock4(m_mtx_map);

        *m_tof_cloud_surf_global_map += *key_cloud;
    }

    void CLocalMapping::thinMapAddKeyCloud(pcl::PointCloud<PointType>::Ptr key_cloud)
    {
        std::lock_guard<std::mutex> lock10(m_mtx_thin_map);

        if (m_thin_map_flag)
            *m_tof_cloud_surf_thin_map += *key_cloud;
    }

    void CLocalMapping::getThinMap(pcl::PointCloud<PointType>::Ptr this_map)
    {
        std::lock_guard<std::mutex> lock10(m_mtx_thin_map);

        pcl::copyPointCloud(*m_tof_cloud_surf_thin_map, *this_map);
        m_tof_cloud_surf_thin_map->clear();
    }

    void CLocalMapping::clearThinMap()
    {
        std::lock_guard<std::mutex> lock10(m_mtx_thin_map);

        m_tof_cloud_surf_thin_map->clear();
    }

    void CLocalMapping::setThinMapFlag(bool falg)
    {
        std::lock_guard<std::mutex> lock10(m_mtx_thin_map);
        m_thin_map_flag = falg;
    }

    pcl::PointCloud<PointType>::Ptr CLocalMapping::transformPointCloud(pcl::PointCloud<PointType>::Ptr cloudIn, PointTypePose *transformIn)
    {
        int cloudSize = cloudIn->size();
        pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());
        cloudOut->resize(cloudSize);

        Eigen::Affine3f transCur = pcl::getTransformation(transformIn->x, transformIn->y, transformIn->z, transformIn->roll, transformIn->pitch, transformIn->yaw);

        for (int i = 0; i < cloudSize; ++i)
        {
            const auto &pointFrom = cloudIn->points[i];
            cloudOut->points[i].x = transCur(0, 0) * pointFrom.x + transCur(0, 1) * pointFrom.y + transCur(0, 2) * pointFrom.z + transCur(0, 3);
            cloudOut->points[i].y = transCur(1, 0) * pointFrom.x + transCur(1, 1) * pointFrom.y + transCur(1, 2) * pointFrom.z + transCur(1, 3);
            cloudOut->points[i].z = transCur(2, 0) * pointFrom.x + transCur(2, 1) * pointFrom.y + transCur(2, 2) * pointFrom.z + transCur(2, 3);
            cloudOut->points[i].intensity = pointFrom.intensity;
        }

        return cloudOut;
    }

    void CLocalMapping::setRobotPose(PointTypePose pose_6d)
    {
        std::lock_guard<std::mutex> lock7(m_mtx_robot_pose);
        m_cur_pose[0] = pose_6d.x;
        m_cur_pose[1] = pose_6d.y;
        m_cur_pose[2] = pose_6d.z;
    }

    bool CLocalMapping::checkPointValid(PointType point, float distance)
    {
        float this_cur_pose[3];
        {
            std::lock_guard<std::mutex> lock7(m_mtx_robot_pose);
            this_cur_pose[0] = m_cur_pose[0];
            this_cur_pose[1] = m_cur_pose[1];
            this_cur_pose[2] = m_cur_pose[2];
        }

        if (abs(point.x - this_cur_pose[0]) > distance || abs(point.y - this_cur_pose[1]) > distance || abs(point.z - this_cur_pose[2]) > distance)
        {
            return false;
        }

        return true;
    }

    void CLocalMapping::get3DFrom6D(pcl::PointCloud<PointType>::Ptr pose_3d, pcl::PointCloud<PointTypePose>::Ptr pose_6d)
    {
        for (size_t i = 0; i < pose_6d->size(); i++)
        {
            PointType thisPose3D;

            thisPose3D.x = pose_6d->points[i].x;
            thisPose3D.y = pose_6d->points[i].y;
            thisPose3D.z = pose_6d->points[i].z;
            thisPose3D.intensity = pose_6d->points[i].intensity;
            pose_3d->push_back(thisPose3D);
        }
    }

    void CLocalMapping::setKdtreeBusy()
    {
        std::lock_guard<std::mutex> lock1(m_mtx_kdtree);

        m_kdtree_idle = false;
    }

    void CLocalMapping::setKdtreeIdle()
    {
        std::lock_guard<std::mutex> lock1(m_mtx_kdtree);

        m_kdtree_idle = true;
    }

    void CLocalMapping::deleteGlobalMap(double fileName)
    {
        std::string this_file_path = m_save_globalMap_dir + std::to_string(fileName) + ".pcd";
        std::string cmd = "rm -rf " + this_file_path;
        std::system(cmd.c_str());
    }

    void CLocalMapping::updataKDtreeInfo(size_t loop_build)
    {
        std::lock_guard<std::mutex> lock1(m_mtx_kdtree);

        if (loop_build == 1)
        {
            m_loop_build = 2;
            m_kdtree_nano = m_kdtree_nano_1;
            m_local_map = m_local_map_1;

            m_local_map_2.clear_pts();
            m_kdtree_nano_2->freeIndex(*m_kdtree_nano_2);
        }
        else if (loop_build == 2)
        {
            m_loop_build = 1;
            m_kdtree_nano = m_kdtree_nano_2;
            m_local_map = m_local_map_2;

            m_local_map_1.clear_pts();
            m_kdtree_nano_1->freeIndex(*m_kdtree_nano_1);
        }
    }

    void savePLY(char *filename, pcl::PointCloud<PointType>::Ptr pointCloud, unsigned int numPoints)
    {
        FILE *fp = NULL;

        unsigned int i;

        fp = fopen(filename, "w");
        if (!fp)
        {
            printf(" Open %s error\n", (char *)filename);

            return;
        }
        fprintf(fp, "%s\n", "ply");
        fprintf(fp, "%s\n", "format ascii 1.0");
        fprintf(fp, "%s %d\n", "element vertex", numPoints);
        fprintf(fp, "%s\n", "property float32 x");
        fprintf(fp, "%s\n", "property float32 y");
        fprintf(fp, "%s\n", "property float32 z");
        fprintf(fp, "%s\n", "end_header");

        for (i = 0; i < numPoints; i++)
        {
            fprintf(fp, "%f %f %f\n", pointCloud->points[i].x, pointCloud->points[i].y, pointCloud->points[i].z);
        }

        fclose(fp);
    }

    void CLocalMapping::icpCheck()
    {
        pcl::PointCloud<PointType>::Ptr keyframe_surf(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr this_local_map(new pcl::PointCloud<PointType>());

        {
            std::lock_guard<std::mutex> lock2(m_mtx_key_pose);
            std::lock_guard<std::mutex> lock3(m_mtx_key_point);

            if (m_key_frame_points.size() > 1)
            {
                // CLog::logAsync(LogSara, LogNormal, "[CLocalMapping] keyframe accu = %d. \n", m_key_frame_points.size());
                std::cout << "[CLocalMapping] keyframe accu = " << m_key_frame_points.size() << std::endl;
            }

            int temp_count = m_key_frame_points.size();
            // for (int i = 0; i < temp_count; i++)
            // {
            //     *keyframe_surf += *(transformPointCloud(m_key_frame_points.back(), &m_key_frame_poses_6d->points[m_key_frame_poses_6d->size() - i - 1]));
            //     m_key_frame_points.pop_back();
            // }

            keyframe_surf = transformPointCloud(m_key_frame_points.back(), &m_key_frame_poses_6d->back());
            // getLocalMap(this_local_map);
            // getGlobalMap(this_local_map);
        }

        *m_tof_cloud_surf_local_map += *keyframe_surf;



        // // *1 保存点云文件
        static int  save_count = 0 ;
        // std::stringstream filename3;
        // filename3 << "/home/maxiaovivi/doubletx-offline/doubletx-offline/allscd/temp_global_pcd/scPCD" << std::setw(6) << std::setfill('0') << save_count << ".pcd";
        // pcl::io::savePCDFileASCII(filename3.str(), *keyframe_surf);
        // std::cout <<RED <<"Saved " << filename3.str() <<RESET <<std::endl;
        save_count++;

        // // *2 保存SC算子
        // scManager->makeAndSaveScancontextAndKeys(*keyframe_surf);
        // const auto& curr_scd = scManager->getConstRefRecentSCD();

        // const static Eigen::IOFormat the_format(3,Eigen::DontAlignCols," ","\n");
        
        // std::ostringstream oss;
        // oss << "/home/maxiaovivi/doubletx-offline/doubletx-offline/global_scancontext/scContext" << std::setw(6) << std::setfill('0') << save_count << ".scd";
        // std::string filename2 = oss.str();
        // std::ofstream file(filename2);
        // if(file.is_open())
        // {
        //     file << curr_scd.format(the_format);
        //     file.close();
        // }

        // // *3 保存当前的位姿
        // PointTypePose lastKeyFramePoint = m_key_frame_poses_6d->back();
        // std::ofstream file3("/home/maxiaovivi/doubletx-offline/doubletx-offline/scPose.txt", std::ios::out | std::ios::app);
        // if (!file3.is_open()) 
        // {
        //     std::cerr << "Failed to open file for appending." << std::endl;
        // }
        // else
        // {
        //     file3 << lastKeyFramePoint.x << " " << lastKeyFramePoint.y << " " << lastKeyFramePoint.z
        //     << " " << lastKeyFramePoint.roll << " " << lastKeyFramePoint.pitch << " " << lastKeyFramePoint.yaw << std::endl;
        //     file3.close();
        // }
        // save_count++;
    
        //! 重定位模拟
        static int relocount = 0;
        static int successcount =0;
        static int nomatchcount = 0;
        static int matchsccount = 0;
        bool real_flag = false;
        pcl::PointCloud<PointType>::Ptr thisRawCloudKeyFrame(new pcl::PointCloud<PointType>);
        {
        std::lock_guard<std::mutex> lock11(m_mtx_keycloud);
        pcl::copyPointCloud(*keyframe_surf,*thisRawCloudKeyFrame);
        }
        scManager->makeAndSaveScancontextAndKeys(*thisRawCloudKeyFrame);
        auto detectResult  = scManager->detectLoopClosureID();
        int  loop_KeyCur   = scManager->getSCsize()-1;
        int  loop_KeyPre   = detectResult.first;
        float yawDiffRad   = detectResult.second;
        if(loop_KeyPre == -1)
        {
            std::cout<< "maxiao 没有可以匹配上的值"<<std::endl;
            nomatchcount++;
    //        matchsccount++;
        }
        else if(!scManager->enoughPointCheck(thisRawCloudKeyFrame,m_key_frame_poses_6d->back()))
        {
            nomatchcount++;
        }
        else
        {
            std::cout<<"maxiao 找到相似的SC：   匹配上的是"<< loop_KeyPre << " 我现在是用的帧是 " << relocount <<std::endl ;
            PointTypePose lastKeyFramePoint = m_key_frame_poses_6d->back();
            auto          Poses             = scManager->getPoses();
            PointTypePose currentKeyFramePonit = Poses[loop_KeyPre];

            float point_distance = sqrt((lastKeyFramePoint.x-currentKeyFramePonit.x)*(lastKeyFramePoint.x-currentKeyFramePonit.x)
            +(lastKeyFramePoint.y-currentKeyFramePonit.y)*(lastKeyFramePoint.y-currentKeyFramePonit.y)+
            (lastKeyFramePoint.z-currentKeyFramePonit.z)*(lastKeyFramePoint.z-currentKeyFramePonit.z));
            float point_diffyaw  =  lastKeyFramePoint.yaw  - currentKeyFramePonit.yaw;
            
            std::cout<<"maxiao 当前距离差为 "<< point_distance << " and  当前角度差为" << point_diffyaw <<std::endl;
            //(point_distance < 0.5)
           // {
            if(scManager->relo_scan2MapOptimization(thisRawCloudKeyFrame,lastKeyFramePoint,currentKeyFramePonit))
            {
            successcount++;
            }
           // }
            else
            {
                auto detectResult2  = scManager->detectLoopClosureIDqueue();
                for(int i = 0;i<3;i++)
                {
                    currentKeyFramePonit = Poses[detectResult2[i].first];
                    // point_distance = sqrt((lastKeyFramePoint.x-currentKeyFramePonit.x)*(lastKeyFramePoint.x-currentKeyFramePonit.x)
                    //                       +(lastKeyFramePoint.y-currentKeyFramePonit.y)*(lastKeyFramePoint.y-currentKeyFramePonit.y)+
                    //                        (lastKeyFramePoint.z-currentKeyFramePonit.z)*(lastKeyFramePoint.z-currentKeyFramePonit.z));
                   // if(point_distance< 0.5)
                  //  {
                    if(scManager->relo_scan2MapOptimization(thisRawCloudKeyFrame,lastKeyFramePoint,currentKeyFramePonit))
                    {
                    successcount++;
                    real_flag =true;
                    break;
                    }
                   //     successcount++;

                   // }
                }
                if(!real_flag) 
                {
                     // *1 观察错误的点云形状
                    std::stringstream filename;
                    filename << "/home/maxiaovivi/doubletx-offline/doubletx-offline/allscd/temp_pcd/scPCDerror" << std::setw(6) << std::setfill('0') << loop_KeyPre << ".pcd";
                    pcl::io::savePCDFileASCII(filename.str(), *thisRawCloudKeyFrame);
                }
            }
        }
        scManager->popCurrentKey();

        relocount++;
        float rate = float(successcount) /float( relocount);
        float rate2 = float(successcount) /float(relocount-nomatchcount);
  //      float rate3 = float(relocount-matchsccount) /float(relocount);
        std::cout<<"maxiao 重定位成功率为   "<< rate2 <<"  没有值的次数： "<<nomatchcount<<std::endl;
        std::cout<<"maxiao 成功帧率为 "<< rate  <<"  执行次数：     "<<relocount   <<"  成功次数： "<<successcount<<std::endl;
//        std::cout<<"maxiao sc帧率为 "<< rate3<<  std::endl;

        globalMapAddKeyCloud(keyframe_surf);
        thinMapAddKeyCloud(keyframe_surf);
        std::cout  <<std::endl;
        std::cout  <<std::endl;

        // octomapBuild(m_key_frame_poses_6d->back(), keyframe_surf);
        return;

#if 0
    if(keyframe_surf->size() < 800 || this_local_map->size() < 1000)
    {
        globalMapAddKeyCloud(keyframe_surf);
        thinMapAddKeyCloud(keyframe_surf);
        // octomapBuild(m_key_frame_poses_6d->back(), keyframe_surf);
        return;
    }

    chrono::steady_clock::time_point icp_t1 = chrono::steady_clock::now();

    // ICP Settings
    // static MyICP icp;
	// icp.setSourceCloud(keyframe_surf);
	// icp.setTargetCloud(this_local_map);
	// icp.setLeafSize(0.05);
	// icp.downsample();
	// icp.setMinError(0.01);
	// icp.setMaxIters(50);
	// icp.registration();
    // icp.saveICPCloud("icp_cloud.pcd");
     
    // ICP Settings
    static pcl::IterativeClosestPoint<PointType, PointType> icp;
    icp.setMaxCorrespondenceDistance(10);
    icp.setMaximumIterations(100);
    icp.setTransformationEpsilon(1e-6);
    icp.setEuclideanFitnessEpsilon(1e-6);
    icp.setRANSACIterations(0);

    // Align clouds
    icp.setInputSource(keyframe_surf);
    icp.setInputTarget(this_local_map);
    pcl::PointCloud<PointType>::Ptr unused_result(new pcl::PointCloud<PointType>());
    icp.align(*unused_result);

    chrono::steady_clock::time_point icp_t2 = chrono::steady_clock::now();
    chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(icp_t2 - icp_t1);
    // CLog() << "[CLocalMapping] " << "icp cost = " << time_used.count() * 1000 << " ms." << std::endl;

    // CLog() << "[CLocalMapping] " << __FUNCTION__ << " icp: hasConverged = " << icp.hasConverged() << " getFitnessScore = " << icp.getFitnessScore() << std::endl;
    if (icp.hasConverged() == false || icp.getFitnessScore() > 0.01)
    // if (icp.getScore() > 0.01)
    {
        globalMapAddKeyCloud(keyframe_surf);
        thinMapAddKeyCloud(keyframe_surf);
        // octomapBuild(m_key_frame_poses_6d->back(), keyframe_surf);
        return;
    }    
    
    pcl::PointCloud<PointType>::Ptr closed_cloud(new pcl::PointCloud<PointType>());
    pcl::transformPointCloud(*keyframe_surf, *closed_cloud, icp.getFinalTransformation());
    // pcl::transformPointCloud(*keyframe_surf, *closed_cloud, icp.getTransformationMatrix());

    globalMapAddKeyCloud(closed_cloud);
    thinMapAddKeyCloud(closed_cloud);
    // octomapBuild(m_key_frame_poses_6d->back(), closed_cloud);
#endif
    }
/*
    void CLocalMapping::octomapBuild(PointTypePose pose, pcl::PointCloud<PointType>::Ptr points)
    {
        double mrpt_t1 = ros::Time::now();

        octomap::Pointcloud cloud_octo;
        for(auto point:points->points)
            cloud_octo.push_back(point.x, point.y, point.z);

        m_octo_tree->insertPointCloud(cloud_octo, octomap::point3d(pose.x, pose.y, pose.z));
        // m_octo_tree->updateInnerOccupancy();

        double mrpt_t2 = ros::Time::now();
        double time_cost = mrpt::system::timeDifference(mrpt_t1, mrpt_t2) * 1000.0;
        CLog::logAsync(LogSara, LogNormal, "[CLocalMapping] octomapBuild cost = %5.2f ms.\n", time_cost);
    }

    void CLocalMapping::octomapToPCL(pcl::PointCloud<PointType>::Ptr global_map)
    {
        double mrpt_t1 = ros::Time::now();
        m_octo_tree->updateInnerOccupancy();
        for(octomap::OcTree::leaf_iterator it = m_octo_tree->begin_leafs(), end = m_octo_tree->end_leafs(); it != end; ++it)
        {
            PointType thisPose3D;
            thisPose3D.x = it.getX();
            thisPose3D.y = it.getY();
            thisPose3D.z = it.getZ();

            if(m_octo_tree->isNodeOccupied(*it))
                global_map->push_back(thisPose3D);
        }

        double mrpt_t2 = ros::Time::now();
        double time_cost = mrpt::system::timeDifference(mrpt_t1, mrpt_t2) * 1000.0;
        CLog::logAsync(LogSara, LogNormal, "[CLocalMapping] octomapToPCL cost = %5.2f ms.\n", time_cost);
    }
*/
    void CLocalMapping::makeCloudThin(pcl::PointCloud<PointType>::Ptr cloudFiltered) // WARNING no lock will cause some newest cloud loss
    {
        // double makeCloudThin_t1 = ros::Time::now();

        PointCloudNano<double> cloudFilteredCopy;

        cloudFilteredCopy.pts.resize(cloudFiltered->points.size());
        for (int i = 0; i < cloudFiltered->points.size(); i++)
        {
            cloudFilteredCopy.pts[i].x = cloudFiltered->points[i].x;
            cloudFilteredCopy.pts[i].y = cloudFiltered->points[i].y;
            cloudFilteredCopy.pts[i].z = cloudFiltered->points[i].z;
        }

        if (cloudFilteredCopy.pts.size() == 0 || cloudFiltered->points.size() == 0)
        {
            // CLog::logAsync(LogSara, LogNormal, "[CLocalMapping] why it happens ?.\n");
            return;
        }

        pcl::PointCloud<PointType>::Ptr cloudFilteredTemp(new pcl::PointCloud<PointType>());
        KDTreeNano kdtree_nano(3, cloudFilteredCopy, KDTreeSingleIndexAdaptorParams(10));
        kdtree_nano.buildIndex();

        int thin_num = 0;
        for (auto p : cloudFiltered->points)
        {
            if (p.intensity > 5)
            {
                cloudFilteredTemp->push_back(p);
                continue;
            }

            const int planePointNum = 10;
            size_t num_results = 10;
            PointType pointOri, pointSel, coeff;
            std::vector<size_t> pointSearchInd(num_results);
            std::vector<double> pointSearchSqDis(num_results);

            const double query[3] = {p.x, p.y, p.z};
            kdtree_nano.knnSearch(&query[0], num_results, &pointSearchInd[0], &pointSearchSqDis[0]);

            Eigen::Matrix<float, planePointNum, 3> matA0;
            Eigen::Matrix<float, planePointNum, 1> matB0;
            Eigen::Vector3f matX0;
            matA0.setZero();
            matB0.fill(-1);
            matX0.setZero();
            for (int j = 0; j < planePointNum; j++)
            {
                matA0(j, 0) = cloudFilteredCopy.pts[pointSearchInd[j]].x;
                matA0(j, 1) = cloudFilteredCopy.pts[pointSearchInd[j]].y;
                matA0(j, 2) = cloudFilteredCopy.pts[pointSearchInd[j]].z;
            }

            PointType p1;

            // 假设平面方程为ax+by+cz+1=0，这里就是求方程的系数abc，d=1
            matX0 = matA0.colPivHouseholderQr().solve(matB0);
            // 平面方程的系数，也是法向量的分量
            float pa = matX0(0, 0);
            float pb = matX0(1, 0);
            float pc = matX0(2, 0);
            float pd = 1;
            // 单位法向量
            float ps = sqrt(pa * pa + pb * pb + pc * pc);
            pa /= ps;
            pb /= ps;
            pc /= ps;
            pd /= ps;
            float k = -1 * (p.x * pa + p.y * pb + p.z * pc) - pd;
            p1.x = p.x + k * pa;
            p1.y = p.y + k * pb;
            p1.z = p.z + k * pc;

            p1.intensity = p.intensity + 1;
            cloudFilteredTemp->push_back(p1);

            thin_num++;
        }

        cloudFiltered->clear();
        pcl::copyPointCloud(*cloudFilteredTemp, *cloudFiltered);

        // double makeCloudThin_t2 = ros::Time::now();
        // double time_cost = mrpt::system::timeDifference(makeCloudThin_t1, makeCloudThin_t2) * 1000.0;
        // CLog::logAsync(LogSara, LogNormal, "[CLocalMapping] makeCloudThin cost = %5.2f ms, cloud map thiner = %d, cloud map size = %d. \n", time_cost, thin_num, cloudFiltered->size());
    }

    void CLocalMapping::cloudThin()
    {
        int cur_nums = 0;
 		while(!m_exit)
        {
            int key_nums = getKeyFrameNum();
            int cloudthin_gap = key_nums > 4*m_cloudthin_keyframe_gap ? m_cloudthin_keyframe_gap : 2*m_cloudthin_keyframe_gap;
            //std::cout << __FUNCTION__ << __LINE__ << " key_frame_num = " << key_nums << std::endl;
            if (key_nums % cloudthin_gap == 0 && key_nums > cur_nums && getGlobalMapNum() > 5*m_localmap_cloud_size )
            {
                cur_nums = key_nums;
                pcl::PointCloud<PointType>::Ptr this_global_map(new pcl::PointCloud<PointType>());
                pcl::PointCloud<PointType>::Ptr this_thin_map(new pcl::PointCloud<PointType>());

                clearThinMap();
                setThinMapFlag(true);
                getGlobalMap(this_global_map);

                if (key_nums % cloudthin_gap == 0)
                {
                    pcl::PointCloud<PointType>::Ptr this_global_map_ror(new pcl::PointCloud<PointType>());
                    pcl::RadiusOutlierRemoval<PointType> outrem;
                    outrem.setInputCloud(this_global_map);
                    outrem.setRadiusSearch(0.1);       // 设置搜索半径
                    outrem.setMinNeighborsInRadius(3); // 设置最少的邻点数量
                    outrem.filter(*this_global_map_ror);
                    this_global_map->clear();
                    pcl::copyPointCloud(*this_global_map_ror, *this_global_map);
                }

                makeCloudThin(this_global_map);
                setThinMapFlag(false);
                getThinMap(this_thin_map);
                *this_global_map += *this_thin_map;
                updataGlobalMap(this_global_map);

                // if (key_nums % 50 == 0)
                // {
                //     char source_data_path[100];
                //     unsigned int global_map_size = this_global_map->size();
                //     // sprintf(source_data_path, "%s", "/userdata/3dslam/ply/PointCloud-slam-global-map.ply");
                //     sprintf(source_data_path, "%s/%s_%d_%d%s", "/home/sc/catkin_lio/plysave", "PointCloud", key_nums, global_map_size, "-slam-global-map.ply");
                //     savePLY(source_data_path, this_global_map, global_map_size);

                //     // debug
                //     //  if(key_nums % 200 == 0 && key_nums != 0)
                //     //  {
                //     //      //save slam map
                //     //      char source_data_path[64];
                //     //      // sprintf(source_data_path, "%s/%s_%d%s", "/media/usb0/ply", "PointCloud", key_nums, "-slam-global-map.ply");
                //     //      // sprintf(source_data_path, "%s/%s_%d%s", "/userdata/3dslam/ply", "PointCloud", key_nums, "-slam-global-map.ply");
                //     //      sprintf(source_data_path, "%s", "/userdata/3dslam/ply/PointCloud-slam-global-map.ply");
                //     //      savePLY(source_data_path, this_global_map_valid, this_global_map_valid->size());

                //     //     // //save octomap
                //     //     // pcl::PointCloud<PointType>::Ptr this_global_map(new pcl::PointCloud<PointType>());
                //     //     // octomapToPCL(this_global_map);
                //     //     // sprintf(source_data_path, "%s/%s_%d%s", "/media/usb0/ply", "PointCloud", key_nums, "-octomap-global-map.ply");
                //     //     // // sprintf(source_data_path, "%s/%s_%d%s", "/userdata/3dslam/ply", "PointCloud", key_nums, "-global-map.ply");
                //     //     // savePLY(source_data_path, this_global_map, this_global_map->size());
                //     // }
                // }
            }

            usleep(1000 * 100);
        }
    }

    void CLocalMapping::run()
    {
    	while(!m_exit)
        {
            if (checkNewKeyFrames())
            {
                // double mrpt_run_t1 = ros::Time::now();

                subWaitingKeyFrame();
                icpCheck();

                pcl::PointCloud<PointType>::Ptr this_global_map(new pcl::PointCloud<PointType>());
                getGlobalMap(this_global_map);

                if (m_loop_build == 1) // build --> m_kdtree_nano_1
                {
                    m_local_map_1.pts.resize(m_tof_cloud_surf_local_map->points.size());
                    for (int i = 0; i < m_tof_cloud_surf_local_map->points.size(); i++)
                    {
                        m_local_map_1.pts[i].x = m_tof_cloud_surf_local_map->points[i].x;
                        m_local_map_1.pts[i].y = m_tof_cloud_surf_local_map->points[i].y;
                        m_local_map_1.pts[i].z = m_tof_cloud_surf_local_map->points[i].z;
                    }

                    m_kdtree_nano_1->buildIndex();

                    if (m_kdtree_idle)
                    {
                        updataKDtreeInfo(m_loop_build);
                        setlLocalMapNum(m_tof_cloud_surf_local_map->points.size());

                        if (!isInit() && this_global_map->size() > m_localmap_cloud_size)
                            setInit(true);
                    }
                    else
                    {
                        m_local_map_1.clear_pts();
                        m_kdtree_nano_1->freeIndex(*m_kdtree_nano_1);
                    }
                }
                else if (m_loop_build == 2) // build --> m_kdtree_nano_2
                {
                    m_local_map_2.pts.resize(m_tof_cloud_surf_local_map->points.size());
                    for (int i = 0; i < m_tof_cloud_surf_local_map->points.size(); i++)
                    {
                        m_local_map_2.pts[i].x = m_tof_cloud_surf_local_map->points[i].x;
                        m_local_map_2.pts[i].y = m_tof_cloud_surf_local_map->points[i].y;
                        m_local_map_2.pts[i].z = m_tof_cloud_surf_local_map->points[i].z;
                    }

                    m_kdtree_nano_2->buildIndex();

                    if (m_kdtree_idle)
                    {
                        updataKDtreeInfo(m_loop_build);
                        setlLocalMapNum(m_tof_cloud_surf_local_map->points.size());

                        if (!isInit() && this_global_map->size() > m_localmap_cloud_size)
                            setInit(true);
                    }
                    else
                    {
                        m_local_map_2.clear_pts();
                        m_kdtree_nano_2->freeIndex(*m_kdtree_nano_2);
                    }
                }

                // double VoxelGrid_t1 = ros::Time::now();

                pcl::PointCloud<PointType>::Ptr this_global_map_valid(new pcl::PointCloud<PointType>());
                pcl::PointCloud<PointType>::Ptr this_local_map(new pcl::PointCloud<PointType>());
                pcl::PointCloud<PointType>::Ptr this_nonlocal_map(new pcl::PointCloud<PointType>());
                pcl::PointCloud<PointType>::Ptr this_local_map_valid(new pcl::PointCloud<PointType>());
                for (int i = 0; i < this_global_map->size(); i++)
                {
                    if (this_global_map->points[i].z < 0)
                        continue;

                    if (checkPointValid(this_global_map->points[i], 8.0) )
                    {
                        this_local_map->push_back(this_global_map->points[i]);
                    }
                    else
                    {
                        this_nonlocal_map->push_back(this_global_map->points[i]);
                    }
                }
                this_global_map->clear();

                int N_flag = 50000;
                int N = getlLocalMapNum();
                int weight_1 = N > N_flag ? (N - N_flag) / 1000 : 0;
                int weight_2 = 0; // N / 30000;
                float leaf = 0.05 + 0.001 * (weight_1 + weight_2);
                if (leaf > 0.09)
                    leaf = 0.09;
                pcl::VoxelGrid<PointType> m_app_down_size_filter_surf;
                m_app_down_size_filter_surf.setInputCloud(this_local_map);
                m_app_down_size_filter_surf.setLeafSize(leaf, leaf, leaf);
                m_app_down_size_filter_surf.filter(*this_local_map_valid);

                *this_global_map_valid += *this_local_map_valid;
                *this_global_map_valid += *this_nonlocal_map;
                updataGlobalMap(this_global_map_valid);
                updataLocalMap(this_local_map_valid);
                // ros
                //std::cout << __FUNCTION__ << "___DEBUG_LJH___" << __LINE__ << " global_map_size = " << this_global_map_valid->size() << std::endl;
                sensor_msgs::PointCloud2 pointcloud_global;
                pcl::toROSMsg(*this_global_map_valid, pointcloud_global);
                pointcloud_global.header.frame_id = "map";
                pointcloud_global.header.stamp = ros::Time::now();
                pub_global_map.publish(pointcloud_global);
                // double mrpt_run_t2 = ros::Time::now();
                // double run_time_cost = mrpt::system::timeDifference(mrpt_run_t1, mrpt_run_t2) * 1000.0;
                // double vg_time_cost = mrpt::system::timeDifference(VoxelGrid_t1, mrpt_run_t2) * 1000.0;
                // CLog::logAsync(LogSara, LogNormal, "[CLocalMapping] run cost = %5.2f ms, voxel grid cost = %5.2f ms, leaf = %.3f, global map = %d, local map = %d. \n",
                //                run_time_cost, vg_time_cost, leaf, (int)this_global_map_valid->size(), getlLocalMapNum());
            }
            usleep(1000 * 5);
        }
    }

    /*
    void CLocalMapping::saveMap()
    {
        CLog() << "****************************************************" << endl;

        if (m_key_frame_poses_6d->empty())
        {
            CLog() << "key frames is empty !!!" << std::endl;
            return;
        }

        CLog() << "Saving map to pcd files ..." << endl;
        CLog() << "Save destination: " << m_save_pcd_directory << endl;

        // // save key frame transformations
        pcl::PointCloud<PointType>::Ptr m_trajectory_key_frame(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr m_trajectory_ori_frame(new pcl::PointCloud<PointType>());
        get3DFrom6D(m_trajectory_key_frame, m_key_frame_poses_6d);
        get3DFrom6D(m_trajectory_ori_frame, m_frame_poses_6d);

        std::string cmd;
        // cmd = "rm -rf " + m_save_pcd_directory + "/trajectoryKeyFrame.pcd";
        // std::system(cmd.c_str());

        // cmd = "rm -rf " + m_save_pcd_directory + "/trajectoryOriFrame.pcd";
        // std::system(cmd.c_str());

        // cmd = "rm -rf " + m_save_pcd_directory + "/SurfMap.pcd";
        // std::system(cmd.c_str());

        cmd = "rm -rf " + m_save_pcd_directory;
        CLog() << cmd << std::endl;
        std::system(cmd.c_str());

        cmd = "mkdir -p " + m_save_pcd_directory;
        CLog() << cmd << std::endl;
        std::system(cmd.c_str());

        pcl::io::savePCDFileBinary(m_save_pcd_directory + "/trajectoryKeyFrame.pcd", *m_trajectory_key_frame);
        pcl::io::savePCDFileBinary(m_save_pcd_directory + "/trajectoryOriFrame.pcd", *m_trajectory_ori_frame);

        // extract global point cloud map
        pcl::PointCloud<PointType>::Ptr globalSurfCloudDS(new pcl::PointCloud<PointType>());

        // down-sample and save surf cloud
        pcl::io::savePCDFileBinary(m_save_pcd_directory + "/SurfMap.pcd", *m_tof_cloud_surf_global_map);

        CLog() << "global Surf Cloud: " << m_tof_cloud_surf_global_map->size() << endl;
        CLog() << "key frame num: " << m_trajectory_key_frame->size() << endl;
        CLog() << "ori frame num: " << m_trajectory_ori_frame->size() << endl;

        CLog() << "****************************************************" << endl;
        CLog() << "Saving map to pcd files completed\n"
               << endl;
    }
*/
    void CLocalMapping::saveGlobalMap(double fileName)
    {
        // DO1:save thisPose6D
        std::ofstream odom_out1;
        // odom_out1.open(mapPoseDirectory.c_str(), std::ios::app);
        odom_out1.open(mapPoseDirectory.c_str());
        odom_out1 << std::to_string(m_frame_poses_6d->points[m_frame_poses_6d->size() - 1].time) << ","
                  << m_frame_poses_6d->points[m_frame_poses_6d->size() - 1].x << ","
                  << m_frame_poses_6d->points[m_frame_poses_6d->size() - 1].y << ","
                  << m_frame_poses_6d->points[m_frame_poses_6d->size() - 1].z << ","
                  << m_frame_poses_6d->points[m_frame_poses_6d->size() - 1].intensity << ","
                  << m_frame_poses_6d->points[m_frame_poses_6d->size() - 1].roll << ","
                  << m_frame_poses_6d->points[m_frame_poses_6d->size() - 1].pitch << ","
                  << m_frame_poses_6d->points[m_frame_poses_6d->size() - 1].yaw << std::endl;
        odom_out1.close();

        // DO2:save m_odom_pose_deque.back()
        std::ofstream odom_out2;
        odom_out2.open(odomPrePoseDirectory.c_str());
        odom_out2 << std::to_string(m_odom_pose_deque.back().odomTimestamp) << ","
                  << m_odom_pose_deque.back().odomX << ","
                  << m_odom_pose_deque.back().odomY << ","
                  << m_odom_pose_deque.back().odomZ << ","
                  << m_odom_pose_deque.back().odomRoll << ","
                  << m_odom_pose_deque.back().odomPitch << ","
                  << m_odom_pose_deque.back().odomYaw << std::endl;
        odom_out2.close();
        // // save key frame transformations
        // pcl::PointCloud<PointType>::Ptr m_trajectory_key_frame(new pcl::PointCloud<PointType>());
        // pcl::PointCloud<PointType>::Ptr m_trajectory_ori_frame(new pcl::PointCloud<PointType>());
        // get3DFrom6D(m_trajectory_key_frame, m_key_frame_poses_6d);
        // get3DFrom6D(m_trajectory_ori_frame, m_frame_poses_6d);

        // std::string rm_cmd;
        // rm_cmd = "rm -rf " + m_save_pcd_directory + "/trajectoryKeyFrame.pcd";
        // std::system(rm_cmd.c_str());
        // if (flag_save)

        CLog() << "Saving Globalmap to pcd files ..." << endl;
        pcl::io::savePCDFileBinary(m_save_globalMap_dir + std::to_string(fileName) + ".pcd", *m_tof_cloud_surf_global_map);

        CLog() << "global Map Cloud Size: " << m_tof_cloud_surf_global_map->size() << endl;
    }

    void CLocalMapping::saveScancontextPcd()
    {
        pcl::KdTreeFLANN<PointType> kdtree;
        kdtree.setInputCloud(m_tof_cloud_surf_global_map);

        PointType minPt, maxPt;
        pcl::getMinMax3D(*m_tof_cloud_surf_global_map, minPt, maxPt);   

        // 划分网格
        float gridSize = 2.0;

        int gridX = std::ceil((maxPt.x - minPt.x) / gridSize);
        int gridY = std::ceil((maxPt.y - minPt.y) / gridSize);
        

        std::vector<PointType> gridCenters;
        for (int i = 0; i < gridX; ++i) {
            for (int j = 0; j < gridY; ++j) {
                PointType center;
                center.x = minPt.x + gridSize * i + gridSize / 2.0;
                center.y = minPt.y + gridSize * j + gridSize / 2.0;
                center.z = 0; // Z值可以设置为0，因为我们只关注二维平面
                gridCenters.push_back(center);
            }
        }

            // 打印搜索中心点
        for (const auto& center : gridCenters) {
            std::cout <<RED<< "Center: " << center.x << ", " << center.y << ", " << center.z << RESET<<std::endl;
        }

        float searchRadius = 6.0; // 以米为单位的搜索半径

        // 遍历每个搜索中心
        for (const auto& center : gridCenters) 
        {
            std::vector<int> pointIdxRadiusSearch;
            std::vector<float> pointRadiusSquaredDistance;

            // 对每个中心点执行半径搜索
            if (kdtree.radiusSearch(center, searchRadius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0) 
            {
                pcl::PointCloud<PointType>::Ptr cloud_cluster(new pcl::PointCloud<PointType>);

                // 将搜索到的点添加到新的点云中
                for (std::size_t i = 0; i < pointIdxRadiusSearch.size(); ++i) 
                {
                    cloud_cluster->points.push_back(m_tof_cloud_surf_global_map->points[pointIdxRadiusSearch[i]]);
                }

                // 为这个点云集创建一个唯一的文件名
                std::stringstream ss;
                ss << "/home/maxiaovivi/doubletx-offline/doubletx-offline/allscd/scd_PCD/cloud_cluster_" << center.x << "_" << center.y << ".pcd";

                
                // 确保cloudCluster是无组织的点云
                cloud_cluster->width = cloud_cluster->points.size();
                cloud_cluster->height = 1;
                cloud_cluster->is_dense = false; // 设置为false，除非你确信所有点都是有效的

                // 然后保存点云到文件
                pcl::io::savePCDFileASCII(ss.str(), *cloud_cluster);

                std::cout << YELLOW<<"Saved " << cloud_cluster->points.size() << " points to " << ss.str()<<RESET << std::endl;
            }

        }
    }
}
