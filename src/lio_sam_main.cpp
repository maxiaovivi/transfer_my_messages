#include "include/CMapOptmization.h"
#include "ros/ros.h"
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <iomanip>

using namespace LIOSAM;

//
//                            _ooOoo_
//                           o8888888o
//                           88" . "88
//                           (| -_- |)
//                           O\  =  /O
//                        ____/`---'\____
//                      .'  \\|     |//  `.
//                     /  \\|||  :  |||//  \
//                    /  _||||| -:- |||||-  \
//                    |   | \\\  -  /// |   |
//                    | \_|  ''\---/''  |   |
//                    \  .-\__  `-`  ___/-. /
//                  ___`. .'  /--.--\  `. . __
//               ."" '<  `.___\_<|>_/___.'  >'"".
//              | | :  `- \`.;`\ _ /`;.`/ - ` : | |
//              \  \ `-.   \_ __\ /__ _/   .-` /  /
//         ======`-.____`-.___\_____/___.-`____.-'======
//                            `=---='
//        ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//                      佛祖保佑       永无BUG

void getBin2Point(std::string &path, pcl::PointCloud<pcl::PointXYZI>::Ptr Cloud)
{
    fstream input(path, ios::in | ios::binary);
    if (!input.good())
    {
        cerr << "Could not read file: " << path << endl;
        exit(EXIT_FAILURE);
    }
    input.seekg(0, ios::beg);

    // pcl::PointCloud<PointXYZI>::Ptr points(new pcl::PointCloud<PointXYZI>);

    int i;
    for (i = 0; input.good() && !input.eof(); i++)
    {
        pcl::PointXYZI point;
        // pcl::PointXYZ point;
        input.read((char *)&point.x, 3 * sizeof(float));
        // input.read((char *)&point.intensity, sizeof(float));
        // pointI.x = point.x;
        // pointI.y = point.y;
        // pointI.z = point.z;

        Cloud->push_back(point);
    }
    input.close();
    //CLog() << "point path/num " << path << " " << i << std::endl;
}

void decodePointCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloudIn, pcl::PointCloud<pcl::PointXYZI>::Ptr cloudOut, std::vector<int> index, int size)
{
    if ((int)cloudIn->size() < (int)index.size() ) {
        CLog() << "WRONG DECODE! what happened? cloudIn->size()/index.size() "
               << (int)cloudIn->size() << " " << (int)index.size() << std::endl;
    }
    int key = 0;
    cloudOut->clear();
    for (int i = 0; i < size; i++) {
        if (key < (int)index.size() && i == index[key]) {
            cloudOut->push_back(cloudIn->points[key]);
            key++;
        } else {
            pcl::PointXYZI dummyPoint;
            dummyPoint.x = 0.0;
            dummyPoint.y = 0.0;
            dummyPoint.z = 0.0;
            cloudOut->push_back(dummyPoint);
        }
    }
}

void getFileNames(string path, vector<string> &filenames)
{
    DIR *pDir;
    struct dirent *ptr;
    if (!(pDir = opendir(path.c_str())))
    {
        CLog() << "Folder doesn't Exist!" << std::endl;
        CLog() << path  << std::endl;
        return;
    }
    while ((ptr = readdir(pDir)) != 0)
    {
        if (strcmp(ptr->d_name, ".") != 0 && strcmp(ptr->d_name, "..") != 0)
        {
            filenames.push_back(path + "/" + ptr->d_name);
        }
    }
    closedir(pDir);

    std::sort(filenames.begin(), filenames.end(), [](string a, string b){
        vector<string> a_vec, b_vec;
        boost::split(a_vec, a, boost::is_any_of("/."), boost::token_compress_on);
        boost::split(b_vec, b, boost::is_any_of("/."), boost::token_compress_on);
        string a_timestamp_name = a_vec.at(a_vec.size()-3) + "." + a_vec.at(a_vec.size()-2);
        string b_timestamp_name = b_vec.at(b_vec.size()-3) + "." + b_vec.at(b_vec.size()-2);
        return std::atof(a_timestamp_name.c_str()) < std::atof(b_timestamp_name.c_str());
    });
}
double string2double(string c)
{
    double d;
    string res = c;
    stringstream ss;
    ss << res;
    ss >> d;
    return d;
}

void toEulerAngle(const Eigen::Quaterniond &q, double &roll, double &pitch, double &yaw)
{
    // roll (x-axis rotation)
    double sinr_cosp = +2.0 * (q.w() * q.x() + q.y() * q.z());
    double cosr_cosp = +1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y());
    roll = atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = +2.0 * (q.w() * q.y() - q.z() * q.x());
    if (fabs(sinp) >= 1)
        pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        pitch = asin(sinp);

    // yaw (z-axis rotation)
    double siny_cosp = +2.0 * (q.w() * q.z() + q.x() * q.y());
    double cosy_cosp = +1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());
    yaw = atan2(siny_cosp, cosy_cosp);
}

int main(int argc, char *argv[])
{
    CLog() << "3dslam test !!!!" << std::endl;
    ros::init(argc, argv, "liosam");
    ros::NodeHandle nh;

    CLog() << fixed;

    CMapOptimization lio_sam(nh);
    ros::Publisher pub_front = nh.advertise<sensor_msgs::PointCloud2>("pointFront", 1);
    ros::Publisher pub_scanMtch_result = nh.advertise<sensor_msgs::PointCloud2>("ScanMatch_result", 1);
    ros::Publisher pub_subMap = nh.advertise<sensor_msgs::PointCloud2>("SubMap2d", 1);
    ros::Publisher pub_dsSurf = nh.advertise<sensor_msgs::PointCloud2>("pointDsSurf", 1);
    ros::Publisher pub_cost_map = nh.advertise<nav_msgs::OccupancyGrid>("cost_map", 5);
    ros::Publisher pub_curPose = nh.advertise<sensor_msgs::PointCloud2>("pointcloud_current_pose", 10);
    // ros::Publisher pub_globalMap = nh.advertise<sensor_msgs::PointCloud2>("globalmap", 1);
    ros::Publisher pub_imu_data = nh.advertise<sensor_msgs::Imu>("imu_yaw", 1);
    usleep(1000000 * 5); 

    int index_of_all = -1;


    std::ifstream configFile("/home/maxiaovivi/doubletx-offline/doubletx-offline/catkin_lio/src/liosam/config/config.txt"); // 打开配置文件
    std::string path1;
    if (configFile) {
        std::getline(configFile, path1); // 读取路径
        std::cout << "Loaded path: " << path1 << std::endl;
    } else {
        std::cerr << "Unable to open config file." << std::endl;
        return 1; // 返回错误码
    }
    
    // sai: read by column
    for (int column_count = 0; column_count < 6; column_count++) {

        std::string topPath;
        if (column_count == 0) {
            topPath = path1;
        } else {
            topPath = path1 + std::to_string(column_count);
        }

        // if (column_count < 1) {
        //     continue;
        // }
        
        std::string pcdFilePathFront = topPath + "/FrontData";
        std::string pcdFilePathLeft = topPath + "/LeftData";
        std::string filePathOdmo = topPath + "/odom_rawlog.txt";
        std::string filePathImu = topPath + "/imu_rawlog.txt";
        std::string filePathFrontIndex = topPath + "/frontIndex_rawlog.txt";
        std::string filePathLeftIndex = topPath + "/leftIndex_rawlog.txt";

        std::vector<std::string> fileNamesFrint, fileNemesLeft, fileNamesFrontIndex, fileNamesLeftIndex, fileNameOdom, vStr, fileNameImu, vStr_imu, vStr_front, vStr_left;
        std::ifstream inFileOdom, inFileImu, inFileFrontIndex, inFileLeftIndex;
        std::string s, s_imu, s_front_index, s_left_index;
        double timestamp, x, y, z, curCloudTime_front, curCloudTime_left;
        Eigen::Quaterniond q;
        odom_info odomInfo;

        sensor_msgs::Imu imu_data;

        getFileNames(pcdFilePathFront, fileNamesFrint);
        getFileNames(pcdFilePathLeft, fileNemesLeft);

        // odo
        inFileOdom.open(filePathOdmo);
        if (!inFileOdom.is_open())
        {
            CLog() << "inFileOdom open file failure" << std::endl;
            CLog() << filePathOdmo << std::endl;

        }
        while (getline(inFileOdom, s))
        {
            fileNameOdom.push_back(s);
        }
        inFileOdom.close();

        // imu
        inFileImu.open(filePathImu);
        if (!inFileImu.is_open())
            CLog() << "inFIleImu open file failure" << std::endl;
        while (getline(inFileImu, s_imu))
        {
            fileNameImu.push_back(s_imu);
        }
        inFileImu.close();

        // front index
        inFileFrontIndex.open(filePathFrontIndex);
        if (!inFileFrontIndex.is_open())
            CLog() << "inFileFrontIndex open file failure" << std::endl;
        while (getline(inFileFrontIndex, s_front_index))
        {
            fileNamesFrontIndex.push_back(s_front_index);
        }
        inFileFrontIndex.close();

        // left index
        inFileLeftIndex.open(filePathLeftIndex);
        if (!inFileLeftIndex.is_open())
            CLog() << "inFileLeftIndex open file failure" << std::endl;
        while (getline(inFileLeftIndex, s_left_index))
        {
            fileNamesLeftIndex.push_back(s_left_index);
        }
        inFileLeftIndex.close();
 
        int index_odom = 0;
        for (int i = 0; i < fileNamesFrint.size(); i++)
        {
            index_of_all++;

            if (index_of_all >= 5285 && index_of_all <= 5405) {
                //continue;
            }

            std::string time_str_front = fileNamesFrint[i].substr(0, fileNamesFrint[i].length() - 4);
            time_str_front = time_str_front.erase(0, pcdFilePathFront.length() + 1);
            curCloudTime_front = std::stod(time_str_front);

            std::string time_str_left = fileNemesLeft[i].substr(0, fileNemesLeft[i].length() - 4);
            time_str_left = time_str_left.erase(0, pcdFilePathLeft.length() + 1);
            curCloudTime_left = std::stod(time_str_left);

            pcl::PointCloud<pcl::PointXYZI>::Ptr tofCloudFrint(new pcl::PointCloud<pcl::PointXYZI>);
            pcl::PointCloud<pcl::PointXYZI>::Ptr tofCloudLeft(new pcl::PointCloud<pcl::PointXYZI>);

            getBin2Point(fileNamesFrint[i], tofCloudFrint);
            getBin2Point(fileNemesLeft[i], tofCloudLeft);

            // sai: decode pointcloud
            std::vector<int> front_index, left_index;
            s_front_index = fileNamesFrontIndex[i];
            boost::split(vStr_front, s_front_index, boost::is_any_of(","), boost::token_compress_on);
            if (curCloudTime_front == std::stod(vStr_front[0])) {
                for (int ii = 1; ii < vStr_front.size(); ii++) {
                    front_index.push_back(std::stoi(vStr_front[ii]));
                }
            } else {
                bool found = false;
                for (int j = i + 1; j < fileNamesFrontIndex.size(); j++) {
                    s_front_index = fileNamesFrontIndex[j];
                    boost::split(vStr_front, s_front_index, boost::is_any_of(","), boost::token_compress_on);
                    if (curCloudTime_front == std::stod(vStr_front[0])) {
                        found = true;
                        break;
                    }
                }
                if (found) {
                    for (int ii = 1; ii < vStr_front.size(); ii++) {
                        front_index.push_back(std::stoi(vStr_front[ii]));
                    }
                } else {
                    continue;
                }
            }

            s_left_index = fileNamesLeftIndex[i];
            boost::split(vStr_left, s_left_index, boost::is_any_of(","), boost::token_compress_on);
            if (curCloudTime_left == std::stod(vStr_left[0])) {
                for (int ii = 1; ii < vStr_left.size(); ii++) {
                    left_index.push_back(std::stoi(vStr_left[ii]));
                }
            } else {
                bool found = false;
                for (int j = i + 1; j < fileNamesLeftIndex.size(); j++) {
                    s_left_index = fileNamesLeftIndex[j];
                    boost::split(vStr_left, s_left_index, boost::is_any_of(","), boost::token_compress_on);
                    if (curCloudTime_left == std::stod(vStr_left[0])) {
                        found = true;
                        break;
                    }
                }
                if (found) {
                    for (int ii = 1; ii < vStr_left.size(); ii++) {
                        left_index.push_back(std::stoi(vStr_left[ii]));
                    }
                } else {
                    continue;
                }
            }

            pcl::PointCloud<pcl::PointXYZI>::Ptr tofCloudFrintDecode(new pcl::PointCloud<pcl::PointXYZI>);
            pcl::PointCloud<pcl::PointXYZI>::Ptr tofCloudLeftDecode(new pcl::PointCloud<pcl::PointXYZI>);
            decodePointCloud(tofCloudFrint, tofCloudFrintDecode, front_index, 240*90);
            decodePointCloud(tofCloudLeft, tofCloudLeftDecode, left_index, 240*90);
            // CLog() << "file path front = " << pcdFilePathFront + fileNamesFrint[i] << std::endl
            //           << std::endl
            //           << "left = " << pcdFilePathLeft + fileNemesLeft[i] << std::endl
            //           << " cloud timestamp = " << curCloudTime_front << std::endl
            //           << " frint points = " << tofCloudFrint->size() << std::endl
            //           << " frint capacity = " << tofCloudFrint->points.capacity() << std::endl;

            // for (int i = index_odom; i < fileNameOdom.size(); i++)
            // {
            s = fileNameOdom[i];
            boost::split(vStr, s, boost::is_any_of(","), boost::token_compress_on);
            timestamp = std::stod(vStr[0]);

            // if (timestamp > curCloudTime_front)
            // {
            x = std::stod(vStr[1]);
            y = std::stod(vStr[2]);
            z = std::stod(vStr[3]);
            // q.x() = std::stod(vStr[4]);
            // q.y() = std::stod(vStr[5]);
            // q.z() = std::stod(vStr[6]);
            // q.w() = 0;
            double roll_debug = std::stod(vStr[4]);
            double pitch_debug = std::stod(vStr[5]);
            double yaw_debug = std::stod(vStr[6]);
            // CLog() << "DEBUG_LJH "
            //           << "________________" << vStr[6] <<" "<< b<< "  " << yaw_debug << "--" << atof(vStr[6].c_str()) << "~~~" << string2double(vStr[6]) << std::endl;
            // imu_data.header.stamp = ros::Time::now();
            // imu_data.header.frame_id = "map";
            // imu_data.orientation.x = q.x();
            // imu_data.orientation.y = q.y();
            // imu_data.orientation.z = q.z();
            // index_odom = i;
            // break;
            //     }
            // }

            odomInfo.odomTimestamp = timestamp;
            odomInfo.odomX = x;
            odomInfo.odomY = y;
            odomInfo.odomZ = z;
            // toEulerAngle(q, odomInfo.odomRoll, odomInfo.odomPitch, odomInfo.odomYaw);
            odomInfo.odomRoll = roll_debug;
            odomInfo.odomPitch = pitch_debug;
            odomInfo.odomYaw = yaw_debug;
            CLog() << "-----------------------------------------------" << '\n'
                << setprecision(20) << index_of_all << "  timestamp odom = " << odomInfo.odomTimestamp << " front_cloud = " << curCloudTime_front << " left_cloud = " << curCloudTime_left << std::endl;

            imu_info imuInfo, imuInfo_previous;
            std::deque<imu_info> imu_deque;

            int index = 0;
            if (curCloudTime_front < curCloudTime_left)
            {
                for (int i = 0; i < fileNameImu.size(); ++i)
                {
                    s_imu = fileNameImu[i];
                    boost::split(vStr_imu, s_imu, boost::is_any_of(","), boost::token_compress_on);
                    imuInfo.imuTimestamp = std::stod(vStr_imu[0]);
                    if (imuInfo.imuTimestamp < curCloudTime_left)
                    {
                        std::vector<std::string> vStr_imu_temp;
                        std::string s_imu_temp = fileNameImu[i];
                        boost::split(vStr_imu_temp, s_imu_temp, boost::is_any_of(","), boost::token_compress_on);
                        imuInfo_previous.imuTimestamp = std::stod(vStr_imu_temp[0]);
                        imuInfo_previous.imuRoll = std::stod(vStr_imu_temp[1]);
                        imuInfo_previous.imuPitch = std::stod(vStr_imu_temp[2]) - 0.044;
                        imuInfo_previous.imuYaw = std::stod(vStr_imu_temp[3]);
                        imuInfo_previous.imuXAcc = std::stod(vStr_imu_temp[4]);
                        imuInfo_previous.imuYAcc = std::stod(vStr_imu_temp[5]);
                        imuInfo_previous.imuZAcc = std::stod(vStr_imu_temp[6]);
                        imuInfo_previous.imuRollVel = std::stod(vStr_imu_temp[7]);
                        imuInfo_previous.imuPitchVel = std::stod(vStr_imu_temp[8]);
                        imuInfo_previous.imuYawVel = std::stod(vStr_imu_temp[9]);
                        imu_deque.push_back(imuInfo_previous);
                        index = i;
                    }
                    else
                    {
                        imuInfo.imuRoll = std::stod(vStr_imu[1]);
                        imuInfo.imuPitch = std::stod(vStr_imu[2]) - 0.044;
                        imuInfo.imuYaw = std::stod(vStr_imu[3]);
                        imuInfo.imuXAcc = std::stod(vStr_imu[4]);
                        imuInfo.imuYAcc = std::stod(vStr_imu[5]);
                        imuInfo.imuZAcc = std::stod(vStr_imu[6]);
                        imuInfo.imuRollVel = std::stod(vStr_imu[7]);
                        imuInfo.imuPitchVel = std::stod(vStr_imu[8]);
                        imuInfo.imuYawVel = std::stod(vStr_imu[9]);
                        imu_deque.push_back(imuInfo);
                        index = i;
                        break;
                    }
                }
            }
            else // left < front
            {
                for (int i = 0; i < fileNameImu.size(); ++i)
                {
                    s_imu = fileNameImu[i];
                    boost::split(vStr_imu, s_imu, boost::is_any_of(","), boost::token_compress_on);
                    imuInfo.imuTimestamp = std::stod(vStr_imu[0]);
                    if (imuInfo.imuTimestamp < curCloudTime_front)
                    {
                        std::vector<std::string> vStr_imu_temp;
                        std::string s_imu_temp = fileNameImu[i];
                        boost::split(vStr_imu_temp, s_imu_temp, boost::is_any_of(","), boost::token_compress_on);
                        imuInfo_previous.imuTimestamp = std::stod(vStr_imu_temp[0]);
                        imuInfo_previous.imuRoll = std::stod(vStr_imu_temp[1]);
                        imuInfo_previous.imuPitch = std::stod(vStr_imu_temp[2]) - 0.044;
                        imuInfo_previous.imuYaw = std::stod(vStr_imu_temp[3]);
                        imuInfo_previous.imuXAcc = std::stod(vStr_imu_temp[4]);
                        imuInfo_previous.imuYAcc = std::stod(vStr_imu_temp[5]);
                        imuInfo_previous.imuZAcc = std::stod(vStr_imu_temp[6]);
                        imuInfo_previous.imuRollVel = std::stod(vStr_imu_temp[7]);
                        imuInfo_previous.imuPitchVel = std::stod(vStr_imu_temp[8]);
                        imuInfo_previous.imuYawVel = std::stod(vStr_imu_temp[9]);
                        imu_deque.push_back(imuInfo_previous);
                        index = i;
                    }
                    else
                    {
                        imuInfo.imuRoll = std::stod(vStr_imu[1]);
                        imuInfo.imuPitch = std::stod(vStr_imu[2]) - 0.044;
                        imuInfo.imuYaw = std::stod(vStr_imu[3]);
                        imuInfo.imuXAcc = std::stod(vStr_imu[4]);
                        imuInfo.imuYAcc = std::stod(vStr_imu[5]);
                        imuInfo.imuZAcc = std::stod(vStr_imu[6]);
                        imuInfo.imuRollVel = std::stod(vStr_imu[7]);
                        imuInfo.imuPitchVel = std::stod(vStr_imu[8]);
                        imuInfo.imuYawVel = std::stod(vStr_imu[9]);
                        imu_deque.push_back(imuInfo);
                        index = i;
                        break;
                    }
                }
            }
            fileNameImu.erase(fileNameImu.begin(), fileNameImu.begin() + index);

            CLog() << "imu_deque_size = " << imu_deque.size() << std::endl;
            CLog() << "imu time: " << setprecision(20) << imu_deque.front().imuTimestamp << '\t' << imu_deque.back().imuTimestamp << std::endl;
            // pub_imu_data.publish(imu_data);
            ros::spinOnce();

            chrono::steady_clock::time_point tt1 = chrono::steady_clock::now();
            lio_sam.tofCloudInfoHandler(tofCloudFrintDecode, tofCloudLeftDecode, curCloudTime_front, curCloudTime_left, odomInfo, imu_deque, pub_scanMtch_result, pub_front, pub_subMap, pub_dsSurf, pub_cost_map, pub_curPose);
            chrono::steady_clock::time_point tt2 = chrono::steady_clock::now();
            chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(tt2 - tt1);
            // CLog() << "main 7 tofCloudInfoHandler cost = " << time_used.count() << " seconds. " << endl;

            if (!ros::ok())
            {
                break;
            }
            if (time_used.count() < 0.02)
            {
                double _sleep = 1000 * (20 - 1000 * time_used.count());
                usleep(_sleep);
            }
        }

        

    } // read by column

    return 0;
}
