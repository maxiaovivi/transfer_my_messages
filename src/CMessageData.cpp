#include "include/CMessageData.h"
#include "include/CGetFileSpace.h"
#include <chrono>

namespace LIOSAM
{

// #pragma comment(lib, "zlib.lib")

CMessageData::CMessageData()
{
	m_exit = false; // sai: exit safe
    // systemTime_flag = true;

    // if (systemTime_flag)
    // {
    //     t_init = chrono::system_clock::now();
    //     FrontLidarFilename_Dym = FrontLidarDataDirectory + std::to_string(FileNumFront) + "/";
    //     LeftLidarFilename_Dym = LeftLidarDataDirectory + std::to_string(FileNumLeft) + "/";
    //     odomFilename_Dym = OdomDataDirectory + std::to_string(static_cast<long long>(FileNumOdom)) + ".txt";
    //     FileNumFront = 1;
    //     FileNumLeft = 1;
    //     FileNumOdom = 1;
    //     boost::filesystem::create_directories(FrontLidarFilename_Dym);
    //     boost::filesystem::create_directories(LeftLidarFilename_Dym);
    //     File_ofs.open(odomFilename_Dym);
    //     File_ofs.close();
    //     pre_odomFilename_Dym = odomFilename_Dym;
    //     systemTime_flag = false
    // }

    
    std::string cmd;

    cmd = "rm -r " + saveMessageDirectory;
    std::cout << cmd.c_str() << std::endl;
    std::system(cmd.c_str());


    // cmd = "mkdir -p " + saveMessageDirectory;
    // std::cout << cmd.c_str() << std::endl;
    // std::system(cmd.c_str());

    cmd = "mkdir -p " + FrontLidarDataDirectory;
    std::cout << cmd.c_str() << std::endl;
    std::system(cmd.c_str());

    cmd = "mkdir -p " + LeftLidarDataDirectory;
    std::cout << cmd.c_str() << std::endl;
    std::system(cmd.c_str());

    // std::string m_save_globalMap_dir = "/userdata/huangph/rawlogData/globalMap";
    // cmd = "mkdir -p " + m_save_globalMap_dir;
    // std::cout << cmd.c_str() << std::endl;
    // std::system(cmd.c_str());
}

CMessageData::~CMessageData()
{
}

void CMessageData::runFileCtlFront()
{
    std::cout << __FUNCTION__ << __LINE__ << FrontLidarFilename_Dym << std::endl;
    std::string path_name = FrontLidarDataDirectory + std::to_string((FileNumFront - 1)) + "/";
    int s_num = getFileSize(path_name);
    std::cout << "check the path's" << path_name << "  files num " << s_num << std::endl;
    if (s_num > (threshNum_size - 1))
    {
        FrontLidarFilename_Dym = FrontLidarDataDirectory + std::to_string(FileNumFront) + "/";
        std::cout << "the new save path:" << FrontLidarFilename_Dym << std::endl;
        if (createNewDir(FrontLidarFilename_Dym))
        {
            std::cout << "new file creat success!Start compress file" << std::endl;
            // compressFile();
            tar_cmd = "tar cf " + FrontLidarDataDirectory + std::to_string((FileNumFront - 1)) + ".tar.xz" + " " + FrontLidar_prefix + std::to_string((FileNumFront - 1)) + "/";
            rm_oriData = "rm -rf " + path_name;
            std::cout << "now delete oridata path: " << rm_oriData.c_str() << std::endl;
            std::system(tar_cmd.c_str());
            std::system(rm_oriData.c_str());
            FileNumFront++;
        }
        else
        {
            std::cout << "creat new file failed!" << std::endl;
        }
    }

    t_sys = chrono::system_clock::now();
    t_used = chrono::duration_cast<chrono::duration<double>>(t_sys - t_init);
    std::cout << "time check=" << t_used.count() << std::endl;
    if (t_used.count() > save_duraTime)
    {
        removeFile(FileNumFront, FrontLidarDataDirectory); //delete lidar data's front

        //odom is a single file,first need creat it then delete before file
        odomFilename_Dym = OdomDataDirectory + std::to_string(static_cast<long long>(FileNumOdom)) + ".txt";
        File_ofs.open(odomFilename_Dym);
        File_ofs.close();
        const char *odom_str = pre_odomFilename_Dym.data();
        if (remove(odom_str) == 0)
        {
            std::cout << "remove odom filename: " << pre_odomFilename_Dym << std::endl;
        }

        watch_dog = true;
        std::cout << "$$$$$$$$$$$  " << watch_dog << std::endl;

        pre_odomFilename_Dym = odomFilename_Dym;
        FileNumOdom++;
        t_init = chrono::system_clock::now();
    }
    else
    {
        watch_dog = false;
        std::cout << "$$$$$$$$$$$  " << watch_dog << std::endl;
    }
}

void CMessageData::runFileCtlLeft()
{
    std::cout << __FUNCTION__ << __LINE__ << LeftLidarFilename_Dym << std::endl;
    std::string path_name = LeftLidarDataDirectory + std::to_string((FileNumLeft - 1)) + "/";
    int s_num = getFileSize(path_name);
    std::cout << "check the path's" << path_name << "  files num " << s_num << std::endl;
    if (s_num > (threshNum_size - 1))
    {
        LeftLidarFilename_Dym = LeftLidarDataDirectory + std::to_string(FileNumLeft) + "/";
        std::cout << "the new save path:" << LeftLidarFilename_Dym << std::endl;
        if (createNewDir(LeftLidarFilename_Dym))
        {
            std::cout << "new file creat success!Start compress file" << std::endl;
            tar_cmd = "tar cf " + LeftLidarDataDirectory + std::to_string((FileNumLeft - 1)) + ".tar.xz" + " " + LeftLidar_prefix + std::to_string((FileNumLeft - 1)) + "/";
            rm_oriData = "rm -rf " + path_name;
            std::cout << "now delete oridata path: " << rm_oriData.c_str() << std::endl;
            std::system(tar_cmd.c_str());
            std::system(rm_oriData.c_str());
            FileNumLeft++;
        }
        else
        {
            std::cout << "creat new file failed!" << std::endl;
        }
    }

    t_sys = chrono::system_clock::now();
    t_used = chrono::duration_cast<chrono::duration<double>>(t_sys - t_init);
    std::cout << "time check=" << t_used.count() << std::endl;
    if (t_used.count() > save_duraTime)
    {
        removeFile(FileNumLeft, LeftLidarDataDirectory);
        t_init = chrono::system_clock::now();
    }
}

void CMessageData::writeFrontLidar(pcl::PointCloud<pcl::PointXYZI>::Ptr tofCloudFront, double curCloudTime)
{
    std::cout << "[CMessageData] " << __FUNCTION__ << " start" << std::endl;

    std::ofstream tofFront_OUT;
    std::string save_filename = FrontLidarDataDirectory + "/" + std::to_string(curCloudTime) + ".bin";
    tofFront_OUT.open(save_filename, std::ios::out | std::ios::binary);
    int cloudSize = tofCloudFront->points.size();
    for (int i = 0; i < cloudSize; ++i)
    {
        float point_x = tofCloudFront->points[i].x;
        float point_y = tofCloudFront->points[i].y;
        float point_z = tofCloudFront->points[i].z;
        tofFront_OUT.write(reinterpret_cast<const char *>(&point_x), sizeof(float));
        tofFront_OUT.write(reinterpret_cast<const char *>(&point_y), sizeof(float));
        tofFront_OUT.write(reinterpret_cast<const char *>(&point_z), sizeof(float));
    }
    tofFront_OUT.close();
    std::cout << "save FrontLidar_file success,name = " << save_filename << std::endl;
}

void CMessageData::writeLeftLidar(pcl::PointCloud<pcl::PointXYZI>::Ptr tofCloudLeft, double curCloudTime)
{
    std::cout << "[CMessageData] " << __FUNCTION__ << " start" << std::endl;

    std::ofstream tofLeft_OUT;
    std::string save_filename = LeftLidarDataDirectory + "/" + std::to_string(curCloudTime) + ".bin";
    tofLeft_OUT.open(save_filename, std::ios::out | std::ios::binary);
    int cloudSize = tofCloudLeft->points.size();
    for (int i = 0; i < cloudSize; ++i)
    {
        float point_x = tofCloudLeft->points[i].x;
        float point_y = tofCloudLeft->points[i].y;
        float point_z = tofCloudLeft->points[i].z;
        tofLeft_OUT.write(reinterpret_cast<const char *>(&point_x), sizeof(float));
        tofLeft_OUT.write(reinterpret_cast<const char *>(&point_y), sizeof(float));
        tofLeft_OUT.write(reinterpret_cast<const char *>(&point_z), sizeof(float));
    }
    tofLeft_OUT.close();
    std::cout << "save LeftLidar_file success,name = " << save_filename << std::endl;
}

void CMessageData::writeOdom(odom_info odom)
{
    std::cout << "[CMessageData] " << __FUNCTION__ << " start" << std::endl;

    // std::cout << fixed;
    // setprecision(15);

    odomFilename_Dym = OdomDataDirectory + "/odom_rawlog.txt";
    std::ofstream odom_out;
    odom_out.open(odomFilename_Dym.c_str(), std::ios::app);
    odom_out << std::to_string(odom.odomTimestamp) << ","
             << odom.odomX << ","
             << odom.odomY << ","
             << odom.odomZ << ","
             << odom.odomRoll << ","
             << odom.odomPitch << ","
             << odom.odomYaw << std::endl;
    odom_out.close();
}

void CMessageData::deleteData(double fileName)
{
    std::string this_front = FrontLidarDataDirectory + "/" + std::to_string(fileName) + ".bin";
    std::string this_left = LeftLidarDataDirectory + "/" + std::to_string(fileName) + ".bin";

    std::string rm_cmd = "rm -rf " + this_front;
    std::cout << "delete front lidar: " << rm_cmd.c_str() << std::endl;
    std::system(rm_cmd.c_str());

    rm_cmd = "rm -rf " + this_left;
    std::cout << "delete left lidar: " << rm_cmd.c_str() << std::endl;
    std::system(rm_cmd.c_str());
}

int CMessageData::getFileSize(const std::string &path_name)
{
    // std::cout << __FUNCTION__ << __LINE__ << std::endl;
    std::vector<std::string> filenames;
    boost::filesystem::path path(path_name);
    // std::cout << "path_name=" << path_name << std::endl;
    // std::cout << "boost::filename=" << boost::filesystem::exists(path) << std::endl;
    if (!boost::filesystem::exists(path))
    {
        std::cout << "The data path error!" << std::endl;
        return false;
    }

    boost::filesystem::directory_iterator end_iter;
    for (boost::filesystem::directory_iterator iter(path); iter != end_iter; ++iter)
    {
        if (boost::filesystem::is_regular_file(iter->status()))
        {
            filenames.push_back(iter->path().string());
        }
        //允许存在子目录并搜素
        // if (boost::filesystem::is_directory(iter->status()))
        // {
        //     get_filenames(iter->path().string(), filenames); //是目录则递归
        // }
    }
    return filenames.size();
    // if (filenames.size() > (threshNum_size-1))
    // {
    //     FrontLidarFilename_Dym = FrontLidarDataDirectory + std::to_string(FileNum) + "/";
    //     if (createNewDir(FrontLidarFilename_Dym))
    //     {
    //         std::cout << "new file creat success!" << std::endl;
    //         FileNum++;
    //         return true;
    //     }
    //     else
    //     {
    //         std::cout << "creat new file failed!" << std::endl;
    //         return false;
    //     }
    // }
}

void CMessageData::compressFile(const std::string &path_name)
{
    // int num = getFileSize(path_name);

    Byte compr[200];
    uLong comprLen;
    const char *hello = "1234455566";
    uLong len = strlen(hello);
    comprLen = sizeof(compr) / sizeof(compr[0]);
    err = compress(compr, &comprLen, (const Bytef *)hello, len);
    if (err != 0)
    {
        std::cout << "compress error!" << std::endl;
    }
    else
    {
        //remove ori_data
        //------------------------------------------------
    }
}

void CMessageData::removeFile(const int &NUMF, const std::string &path_name)
{
    if ((NUMF - 1) < 0)
    {
        std::cout << "delete time is so min,retry bigger time!" << std::endl;
        return;
    }
    for (int i = 0; i < (NUMF - 1); ++i)
    {
        std::string rem_FileName = path_name + std::to_string(i) + FileType;
        if (boost::filesystem::exists(rem_FileName))
        {
            const char *str = rem_FileName.data();
            if (remove(str) == 0)
            {
                std::cout << "remove filename : " << rem_FileName << std::endl;
            }
        }
    }
}

bool CMessageData::isDirExist(const std::string &path_name)
{
    if (boost::filesystem::exists(path_name) &&
        boost::filesystem::is_directory(path_name))
    {
        return true;
    }
    return false;
}

bool CMessageData::createNewDir(const std::string &path_name)
{
    // if (isDirExist(path_name))
    // {
    //     return true;
    // }
    return boost::filesystem::create_directories(path_name);
}

bool CMessageData::getMapSaveFlg()
{
    return watch_dog;
}

void CMessageData::setLocalMapper(CLocalMapping* local_mapper)
{
    m_local_mapper = local_mapper;
}

void CMessageData::pushRawLogData(const pcl::PointCloud<PointType>::Ptr tofCloudFront, const pcl::PointCloud<PointType>::Ptr tofCloudLeft, const odom_info odomInfo)
{
    std::cout << __FUNCTION__ << " start" << std::endl;

    pcl::PointCloud<PointType>::Ptr this_cloud_front(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr this_cloud_left(new pcl::PointCloud<PointType>());
    odom_info this_odom;

    *this_cloud_front = *tofCloudFront;
    *this_cloud_left = *tofCloudLeft;
    this_odom = odomInfo;

    std::lock_guard<std::mutex> lock1(m_mtx_data);
    m_cloud_front_deque.push_back(this_cloud_front);
    m_cloud_left_deque.push_back(this_cloud_left);
    m_odom_deque.push_back(this_odom);

    std::cout << __FUNCTION__ << " end" << std::endl;
}

bool CMessageData::isDataEmpty()
{
    std::lock_guard<std::mutex> lock1(m_mtx_data);
    return m_cloud_front_deque.empty() || m_cloud_left_deque.empty() || m_odom_deque.empty();
}

void CMessageData::writeRawLogData()
{
    std::cout << "[CMessageData] " << __FUNCTION__ << " start" << std::endl;

    pcl::PointCloud<PointType>::Ptr this_cloud_front(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr this_cloud_left(new pcl::PointCloud<PointType>());
    odom_info this_odom;

    *this_cloud_front = *m_cloud_front_deque.front();
    *this_cloud_left = *m_cloud_left_deque.front();
    this_odom = m_odom_deque.front();

    m_cloud_front_deque.pop_front();
    m_cloud_left_deque.pop_front();
    m_odom_deque.pop_front();

    m_cur_frame_timestmp = this_odom.odomTimestamp;

    writeFrontLidar(this_cloud_front, m_cur_frame_timestmp);
    writeLeftLidar(this_cloud_left, m_cur_frame_timestmp);
    writeOdom(this_odom);

    m_history_frames.push_back(m_cur_frame_timestmp);

    // if(m_cur_global_map_timestmp == 0.0)
    //     m_cur_global_map_timestmp = m_cur_frame_timestmp;

    std::cout << "[CMessageData] " << __FUNCTION__ << " end" << std::endl;
}

void CMessageData::writeGlobalMap(double timestmp)
{
    std::cout << "[CMessageData] " << __FUNCTION__ << " start" << std::endl;

    m_cur_global_map_timestmp = timestmp;

    m_local_mapper->saveGlobalMap(m_cur_global_map_timestmp);
    m_history_map.push_back(m_cur_global_map_timestmp);

    if(abs(m_cur_global_map_timestmp - m_history_map.front()) > 10 * 1 * 2.2)
    {
        //rm first map
        static bool isFirst = true;
        if(isFirst)
            isFirst = false;

        if(!isFirst)
        {
            m_local_mapper->deleteGlobalMap(m_history_map.front());
            m_history_map.pop_front();
        }
            
        //rm cloud
        while(m_history_frames.front() < m_history_map.front())
        {
            deleteData(m_history_frames.front());
            m_history_frames.pop_front();
        }
    }
    
    std::cout << "[CMessageData] " << __FUNCTION__ << " end" << std::endl;
}

void CMessageData::run()
{
    std::cout << "[CMessageData] " << __FUNCTION__ << " start" << std::endl;

    double numMinute = 1;

    while(!m_exit)
    {
        if(!isDataEmpty())
        {
            writeRawLogData();

            if(m_cur_global_map_timestmp == 0.0)
            {
                std::cout << "detal time1 = " << abs(m_cur_frame_timestmp - m_history_frames.front()) << std::endl;
                if(abs(m_cur_frame_timestmp - m_history_frames.front()) > 10 * numMinute)
                    writeGlobalMap(m_cur_frame_timestmp); 
            }
            else
            {
                std::cout << "detal time2 = " << abs(m_cur_frame_timestmp - m_cur_global_map_timestmp) << std::endl;
                if(abs(m_cur_frame_timestmp - m_cur_global_map_timestmp) > 10 * numMinute)
                    writeGlobalMap(m_cur_frame_timestmp);
            } 
        }
            
        usleep(1000 * 10);
    }
}

}//namespace LIOSAM
