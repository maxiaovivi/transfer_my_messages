#include "include/CMapDisplay.h"

using namespace LIOSAM;


CMapDisplay::CMapDisplay()
{

}

CMapDisplay::~CMapDisplay()
{

}

void CMapDisplay::setLocalMapper(CLocalMapping* local_mapper)
{
    m_local_mapper = local_mapper;
}

pcl::PointCloud<PointType>::Ptr CMapDisplay::transformPointCloud(pcl::PointCloud<PointType>::Ptr cloudIn, PointTypePose* transformIn)
{
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

void CMapDisplay::pushSlamPoseDeque(PointTypePose pose)
{
    std::lock_guard<std::mutex> lock1(m_mtx_slam_pose);

    m_slam_pose_deque.push_back(pose);
}

void CMapDisplay::pushKeyPoseDeque(PointTypePose pose)
{
    std::lock_guard<std::mutex> lock2(m_mtx_key_pose);

    m_key_pose_deque.push_back(pose);
}

void CMapDisplay::pushKeyCloudDeque(pcl::PointCloud<PointType> cloud)
{
    std::lock_guard<std::mutex> lock3(m_mtx_key_cloud);

    m_key_cloud_deque.push_back(cloud);
}

PointTypePose CMapDisplay::getSlamPoseDeque()
{
    std::lock_guard<std::mutex> lock1(m_mtx_slam_pose);

    PointTypePose this_pose = m_slam_pose_deque.front();
    m_slam_pose_deque.pop_front();

    return this_pose;
}

PointTypePose CMapDisplay::getKeyPoseDeque()
{
    std::lock_guard<std::mutex> lock2(m_mtx_key_pose);

    PointTypePose this_pose = m_key_pose_deque.front();
    m_key_pose_deque.pop_front();

    return this_pose;
}

pcl::PointCloud<PointType> CMapDisplay::getKeyCloudDeque()
{
    std::lock_guard<std::mutex> lock3(m_mtx_key_cloud);

    pcl::PointCloud<PointType> this_pose = m_key_cloud_deque.front();
    m_key_cloud_deque.pop_front();

    return this_pose;
}

bool CMapDisplay::isSPDequeEmpty()
{
    std::lock_guard<std::mutex> lock1(m_mtx_slam_pose);

    return m_slam_pose_deque.empty();
}

bool CMapDisplay::isKPDequeEmpty()
{
    std::lock_guard<std::mutex> lock2(m_mtx_key_pose);

    return m_key_pose_deque.empty();
}

bool CMapDisplay::isKCDequeEmpty()
{
    std::lock_guard<std::mutex> lock3(m_mtx_key_cloud);

    return m_key_cloud_deque.empty();
}

void CMapDisplay::run()
{
    // CLog() << "tcp slam msg send thread ...." << std::endl;
    int sockfd;
	sockfd = socket(AF_INET, SOCK_STREAM, 0);// 创建通信端点：套接字

    // 设置服务器地址结构体
	struct sockaddr_in server_addr;
	bzero(&server_addr,sizeof(server_addr)); // 初始化服务器地址
	server_addr.sin_family = AF_INET;	// IPv4
	server_addr.sin_port = htons(8000);	// 端口
	server_addr.sin_addr.s_addr = INADDR_ANY;              //本地的任意地址
	
	 //绑定然后监听
    if (::bind(sockfd, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
        // CLog() << "bind: error" << std::endl;
        return;
    }

    if(listen(sockfd, 5))
    {
        // CLog() << "listen: error" << std::endl;
        return;
    }

    char send_buf[BUF_SIZE] = {0};

    slamPacket slam_packet_odom, slam_packet_point, slam_packet_end;

    slam_packet_odom.msg_head1 = 0xAA;
    slam_packet_odom.msg_head2 = 0x66;
    slam_packet_odom.msg_size = 32;
    slam_packet_odom.frame_len = 272;

    slam_packet_point.msg_head1 = 0xBB;
    slam_packet_point.msg_head2 = 0x66;
    slam_packet_point.msg_size = 32;
    slam_packet_point.frame_len = 272;

    slamPose slam_pose;
    cloudPoint cloud_point;
    bool send_fail_flag = false;
    
    while(1)
    {
        //接收连接请求，得到连接成功的通信套接字 //创建一个地址结构体，用来存放请求连接的客户端的地址 
        struct sockaddr_in caddr;
        //IPV4地址结构体 
        socklen_t addrlen = sizeof(struct sockaddr); 
        int new_sockat = accept(sockfd, (struct sockaddr *)&caddr, &addrlen); 
        if(new_sockat < 0) 
        { 
            // CLog() << "accept: error" << std::endl;
            exit(1);
        } 

        int key_send_index = 0;
        int ori_send_index = 0;
        int key_num = 0;
        int ori_num = 0;
        double key_time = 0.0;
        double ori_time = 0.0;
        while (1)
        {
            if(!isSPDequeEmpty())
            {
                PointTypePose this_pose = getSlamPoseDeque();
                slam_pose.x = this_pose.x; 
                slam_pose.y = this_pose.y; 
                slam_pose.phi = this_pose.yaw;
                slam_pose.timestamp = this_pose.time;
                ori_time = slam_pose.timestamp;

                memcpy(slam_packet_odom.msg_buf, &slam_pose, sizeof(slam_pose));
                memcpy(send_buf, &slam_packet_odom, sizeof(slam_packet_odom));
                int ret = send(new_sockat, send_buf, sizeof(send_buf), 0); 
                if(ret < 0) 
                { 
                    // CLog() << "send pose error" << std::endl;
                    fflush(stdout);
                    break;
                } 
                // CLog() << "send pose success" << std::endl;
                ori_send_index++;

                if(ori_time < key_time)
                    continue;
            }

            if(!isKCDequeEmpty() && !isKPDequeEmpty() && false)
            {
                PointTypePose this_pose = getKeyPoseDeque();
                key_time = this_pose.time;

                pcl::PointCloud<PointType> this_cloud = getKeyCloudDeque();
                if(this_cloud.points.size() < 10)
                {
                    key_send_index++;
                    continue;
                }                 

                pcl::PointCloud<PointType>::Ptr send_cloud_ds(new pcl::PointCloud<PointType>());
                pcl::VoxelGrid<PointType> send_down_size_filter;
                send_down_size_filter.setLeafSize(0.05, 0.05, 0.05);
                send_down_size_filter.setInputCloud(this_cloud.makeShared());
                send_down_size_filter.filter(*send_cloud_ds);

                *send_cloud_ds = *transformPointCloud(send_cloud_ds, &this_pose);

                if(send_cloud_ds->points.size() < 10)
                {
                    key_send_index++;
                    continue;
                }

                for(int i = 0; i < send_cloud_ds->points.size(); i++)
                {
                    cloud_point.x = send_cloud_ds->points[i].x; 
                    cloud_point.y = send_cloud_ds->points[i].y;
                    cloud_point.z = send_cloud_ds->points[i].z;
                    cloud_point.timestamp = this_pose.time;
                    
                    memcpy(slam_packet_point.msg_buf, &cloud_point, sizeof(cloud_point));
                    memcpy(send_buf, &slam_packet_point, sizeof(slam_packet_point));

                    int ret = send(new_sockat, send_buf, sizeof(send_buf), 0); 
                    if(ret < 0) 
                    { 
                        // CLog() << "send point error" << std::endl;
                        fflush(stdout);
                        send_fail_flag = true;
                        break;
                    }

                    usleep(100);
                }

                if(send_fail_flag)
                {
                    send_fail_flag = false;
                    break;
                }
                    
                key_send_index++;
            }

            if(!isKPDequeEmpty())
            {
                PointTypePose this_pose = getKeyPoseDeque();
                key_time = this_pose.time;

                pcl::PointCloud<PointType>::Ptr global_map(new pcl::PointCloud<PointType>());
                m_local_mapper->getGlobalMap(global_map);

                for(int i = 0; i < global_map->points.size(); i++)
                {
                    cloud_point.x = global_map->points[i].x; 
                    cloud_point.y = global_map->points[i].y;
                    cloud_point.z = global_map->points[i].z;
                    cloud_point.timestamp = this_pose.time;
                    
                    memcpy(slam_packet_point.msg_buf, &cloud_point, sizeof(cloud_point));
                    memcpy(send_buf, &slam_packet_point, sizeof(slam_packet_point));

                    int ret = send(new_sockat, send_buf, sizeof(send_buf), 0); 
                    if(ret < 0) 
                    { 
                        // CLog() << "send point error" << std::endl;
                        fflush(stdout);
                        send_fail_flag = true;
                        break;
                    }

                    usleep(10);
                }

                if(send_fail_flag)
                {
                    send_fail_flag = false;
                    break;
                }
            }

            usleep(100);
        }
    } 

    close(sockfd); 
}



