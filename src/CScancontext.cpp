#include "include/CScancontext.h"
#include "include/nanoflann.hpp"
#include <experimental/filesystem>
#include <boost/filesystem.hpp>
#include <pcl/surface/poisson.h>
#include <pcl/octree/octree_pointcloud_voxelcentroid.h>
/********************************** Name space ************************************/

//using namespace std;
namespace fs = boost::filesystem;

/***********************************************************************************
Function:     CRobotCartoSlam
Description:  The constructor of Scancontext
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
CScancontext::CScancontext()
{
        m_polarcontextstimestamp.clear(); 
        m_polarcontexts.clear();
        m_polarcontext_invkeys.clear();
        m_polarcontext_vkeys.clear();

        m_polarcontext_invkeysmat.clear();
        m_polarcontext_invkeysto_search.clear();
        m_polarcontext_tree.reset(); 
        m_poses.clear();
        m_tof_cloud_saved_global_map.reset(new pcl::PointCloud<PointType>());
        m_global_scan.reset(new pcl::PointCloud<PointType>());
        loadSavedGlobalMap();

}

/***********************************************************************************
Function:     CScancontext
Description:  The destructor of Scancontext
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
CScancontext::~CScancontext()
{
        m_polarcontext_tree.reset(); 
}
/***********************************************************************************
Function:     xy2theta
Description:  xy2theta
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
float CScancontext::xy2theta( const float & _x, const float & _y )
{
    if ( (_x >= 0) & (_y >= 0)) 
        return (180/M_PI) * atan(_y / _x);

    if ( (_x < 0) & (_y >= 0)) 
        return 180 - ( (180/M_PI) * atan(_y / (-_x)) );

    if ( (_x < 0) & (_y < 0)) 
        return 180 + ( (180/M_PI) * atan(_y / _x) );

    if ( (_x >= 0) & (_y < 0))
        return 360 - ( (180/M_PI) * atan((-_y) / _x) );

    return -1;    
} 
/***********************************************************************************
Function:     rad2deg
Description:  rad2deg
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
float CScancontext::rad2deg(float radians)
{
    return radians * 180.0 / M_PI;
}
/***********************************************************************************
Function:     rad2deg
Description:  rad2deg
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
float CScancontext::deg2rad(float degrees)
{
    return degrees * M_PI / 180.0;
}
/***********************************************************************************
Function:     rad2deg
Description:  rad2deg
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
std::vector<float> CScancontext::eig2stdvec( Eigen::MatrixXd _eigmat )
{
    std::vector<float> vec( _eigmat.data(), _eigmat.data() + _eigmat.size() );
    return vec;
}
/***********************************************************************************
Function:     circshift
Description:  circshift
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
Eigen::MatrixXd CScancontext::circshift( Eigen::MatrixXd &_mat, int _num_shift )
{
    // 计算循环偏移量
    assert(_num_shift >= 0);

    if( _num_shift == 0 )
    {
        Eigen::MatrixXd shifted_mat( _mat );
        return shifted_mat; 
    }

    Eigen::MatrixXd shifted_mat = Eigen::MatrixXd::Zero( _mat.rows(), _mat.cols() );
    for ( int col_idx = 0; col_idx < _mat.cols(); col_idx++ )
    {
        int new_location = (col_idx + _num_shift) % _mat.cols();
        shifted_mat.col(new_location) = _mat.col(col_idx);
    }

    return shifted_mat;

}
/***********************************************************************************
Function:     distDirectSC
Description:  distDirectSC
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
double CScancontext::distDirectSC ( Eigen::MatrixXd &_sc1, Eigen::MatrixXd &_sc2 )
{

    //* 计算余弦偏移大小
    int num_eff_cols = 0; 
    double sum_sector_similarity = 0;
    for ( int col_idx = 0; col_idx < _sc1.cols(); col_idx++ )
    {
        Eigen::VectorXd col_sc1 = _sc1.col(col_idx);
        Eigen::VectorXd col_sc2 = _sc2.col(col_idx);
        
        if( (col_sc1.norm() == 0) | (col_sc2.norm() == 0) )
            continue;  

        double sector_similarity = col_sc1.dot(col_sc2) / (col_sc1.norm() * col_sc2.norm());

        sum_sector_similarity = sum_sector_similarity + sector_similarity;
        num_eff_cols = num_eff_cols + 1;
    }
    
    double sc_sim = sum_sector_similarity / num_eff_cols;
    return 1.0 - sc_sim;

}
/***********************************************************************************
Function:     makeScancontext
Description:  make scancontext
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
Eigen::MatrixXd CScancontext::makeScancontext( pcl::PointCloud<SCPointType> & _scan_down )
{

    int num_pts_scan_down = _scan_down.points.size();

    // main
    const int NO_POINT = -1000;
    Eigen::MatrixXd desc = NO_POINT * Eigen::MatrixXd::Ones(PC_NUM_RING, PC_NUM_SECTOR);

    SCPointType pt;
    float azim_angle, azim_range; // 极坐标
    int ring_idx, sctor_idx;
    for (int pt_idx = 0; pt_idx < num_pts_scan_down; pt_idx++)
    {
        pt.x = _scan_down.points[pt_idx].x; 
        pt.y = _scan_down.points[pt_idx].y;
        pt.z = _scan_down.points[pt_idx].z + LIDAR_HEIGHT; // 这个点的高度要加上雷达的高度

        //笛卡尔转极坐标
        azim_range = sqrt(pt.x * pt.x + pt.y * pt.y);
        azim_angle = xy2theta(pt.x, pt.y);

        if(azim_angle == -1)  
           // CLog::log(LogYyang, LogErro, "[CScancontext]  角度返回值异常! \n");

        // 过滤超出范围的点
        if( azim_range > PC_MAX_RADIUS )
            continue;

        ring_idx = std::max( std::min( PC_NUM_RING, int(ceil( (azim_range / PC_MAX_RADIUS) * PC_NUM_RING )) ), 1 );
        sctor_idx = std::max( std::min( PC_NUM_SECTOR, int(ceil( (azim_angle / 360.0) * PC_NUM_SECTOR )) ), 1 );

        // 找到最大的Z值，索引从0开始，所以这里-1
        if ( desc(ring_idx-1, sctor_idx-1) < pt.z )  
            desc(ring_idx-1, sctor_idx-1) = pt.z;
    }

    // 如果没有点的bin块需要补充为0，不然余弦距离会计算异常
    for ( int row_idx = 0; row_idx < desc.rows(); row_idx++ )
        for ( int col_idx = 0; col_idx < desc.cols(); col_idx++ )
            if( desc(row_idx, col_idx) == NO_POINT )
                desc(row_idx, col_idx) = 0;


    return desc;
}
/***********************************************************************************
Function:     makeRingkeyFromScancontext
Description:  makeRingkeyFromScancontext
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
Eigen::MatrixXd CScancontext::makeRingkeyFromScancontext( Eigen::MatrixXd &_desc )
{
    /* 
     * 加起来得到ring key算子
    */
    Eigen::MatrixXd invariant_key(_desc.rows(), 1);
    for ( int row_idx = 0; row_idx < _desc.rows(); row_idx++ )
    {
        Eigen::MatrixXd curr_row = _desc.row(row_idx);
        invariant_key(row_idx, 0) = curr_row.mean();
    }

    return invariant_key;
}
/***********************************************************************************
Function:     distanceBtnScanContext
Description:  distanceBtnScanContext
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
std::pair<double, int> CScancontext::distanceBtnScanContext( Eigen::MatrixXd &_sc1, Eigen::MatrixXd &_sc2 )
{
    //*  1. 快速暴力对齐
    Eigen::MatrixXd vkey_sc1 = makeSectorkeyFromScancontext( _sc1 );
    Eigen::MatrixXd vkey_sc2 = makeSectorkeyFromScancontext( _sc2 );
    int argmin_vkey_shift = fastAlignUsingVkey( vkey_sc1, vkey_sc2 );

    const int SEARCH_RADIUS = round( 0.5 * SEARCH_RATIO * _sc1.cols() ); // a half of search range 
    std::vector<int> shift_idx_search_space { argmin_vkey_shift };
    for ( int ii = 1; ii < SEARCH_RADIUS + 1; ii++ )
    {
        shift_idx_search_space.push_back( (argmin_vkey_shift + ii + _sc1.cols()) % _sc1.cols() );
        shift_idx_search_space.push_back( (argmin_vkey_shift - ii + _sc1.cols()) % _sc1.cols() );
    }
    std::sort(shift_idx_search_space.begin(), shift_idx_search_space.end());

    //*2. 计算余弦距离
    int argmin_shift = 0;
    double min_sc_dist = 10000000;
    for ( int num_shift: shift_idx_search_space )
    {
        Eigen::MatrixXd sc2_shifted = circshift(_sc2, num_shift);
        double cur_sc_dist = distDirectSC( _sc1, sc2_shifted );
        if( cur_sc_dist < min_sc_dist )
        {
            argmin_shift = num_shift;
            min_sc_dist = cur_sc_dist;
        }
    }

    return make_pair(min_sc_dist, argmin_shift);

}
/***********************************************************************************
Function:     makeSectorkeyFromScancontext
Description:  makeSectorkeyFromScancontext
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
Eigen::MatrixXd CScancontext::makeSectorkeyFromScancontext( Eigen::MatrixXd &_desc )
{
    /* 
     * 计算sector算子，用于快速对齐
    */
    Eigen::MatrixXd variant_key(1, _desc.cols());
    for ( int col_idx = 0; col_idx < _desc.cols(); col_idx++ )
    {
        Eigen::MatrixXd curr_col = _desc.col(col_idx);
        variant_key(0, col_idx) = curr_col.mean();
    }

    return variant_key;
} 
/***********************************************************************************
Function:     fastAlignUsingVkey
Description:  fastAlignUsingVkey
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
int CScancontext::fastAlignUsingVkey( Eigen::MatrixXd & _vkey1, Eigen::MatrixXd & _vkey2)
{
    int argmin_vkey_shift = 0;
    double min_veky_diff_norm = 10000000;
    for ( int shift_idx = 0; shift_idx < _vkey1.cols(); shift_idx++ )
    {
        Eigen::MatrixXd vkey2_shifted = circshift(_vkey2, shift_idx);

        Eigen::MatrixXd vkey_diff = _vkey1 - vkey2_shifted;

        double cur_diff_norm = vkey_diff.norm();
        if( cur_diff_norm < min_veky_diff_norm )
        {
            argmin_vkey_shift = shift_idx;
            min_veky_diff_norm = cur_diff_norm;
        }
    }

    return argmin_vkey_shift;

} 
/***********************************************************************************
Function:     makeAndSaveScancontextAndKeys
Description:  makeAndSaveScancontextAndKeys
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
void CScancontext::makeAndSaveScancontextAndKeys( pcl::PointCloud<SCPointType> & _scan_down )
{
    Eigen::MatrixXd sc = makeScancontext(_scan_down); // v1 
    Eigen::MatrixXd ringkey = makeRingkeyFromScancontext( sc );
    Eigen::MatrixXd sectorkey = makeSectorkeyFromScancontext( sc );
    std::vector<float> polarcontext_invkey_vec = eig2stdvec( ringkey );

    m_polarcontexts.push_back( sc ); 
    m_polarcontext_invkeys.push_back( ringkey );
    m_polarcontext_vkeys.push_back( sectorkey );
    m_polarcontext_invkeysmat.push_back( polarcontext_invkey_vec );

    // cout <<m_polarcontext_vkeys.size() << endl;

}
/***********************************************************************************
Function:     makeAndSaveScancontextAndKeys
Description:  makeAndSaveScancontextAndKeys
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
void CScancontext::popCurrentKey()
{

    m_polarcontexts.pop_back(); 
    m_polarcontext_invkeys.pop_back( );
    m_polarcontext_vkeys.pop_back();
    m_polarcontext_invkeysmat.pop_back( );

    // cout <<m_polarcontext_vkeys.size() << endl;

}
/***********************************************************************************
Function:     detectLoopClosureID
Description:  detectLoopClosureID
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/

std::pair<int, float> CScancontext::detectLoopClosureID ( void )
{
    int loop_id { -1 }; // 用于回环检测的

    auto curr_key  =  m_polarcontext_invkeysmat.back(); // 当前点云生成的算子
    auto curr_desc =  m_polarcontexts.back();           

    /* 
     * step 1: 通过Ring 算子挑选出候选者
     */
    if( (int)m_polarcontext_invkeysmat.size() < NUM_EXCLUDE_RECENT + 1)
    {
        std::pair<int, float> result {loop_id, 0.0};
        return result; // 错误返回
    }

    // KD树重建频率
    if( m_tree_making_period_conter % TREE_MAKING_PERIOD_ == 0) // to save computation cost
    {
        m_polarcontext_invkeysto_search.clear();
        m_polarcontext_invkeysto_search.assign( m_polarcontext_invkeysmat.begin(), m_polarcontext_invkeysmat.end() - NUM_EXCLUDE_RECENT ) ;

        m_polarcontext_tree.reset(); 
        m_polarcontext_tree = std::make_unique<InvKeyTree>(PC_NUM_RING, m_polarcontext_invkeysto_search, 20 ); // 维度 ， 待搜索数据 , 最大叶子节点
    }
    m_tree_making_period_conter = m_tree_making_period_conter + 1;
        
    double min_dist = 10000000; //初始化最小距离
    int nn_align = 0;
    int nn_idx = 0;

    // knn search
    std::vector<size_t>   candidate_indexes( NUM_CANDIDATES_FROM_TREE ); 
    std::vector<float>    out_dists_sqr( NUM_CANDIDATES_FROM_TREE );


    nanoflann::KNNResultSet<float> knnsearch_result( NUM_CANDIDATES_FROM_TREE );
    knnsearch_result.init( &candidate_indexes[0], &out_dists_sqr[0] );
    m_polarcontext_tree->index->findNeighbors( knnsearch_result, &curr_key[0] /* query */, nanoflann::SearchParams(1024) ); 


    /* 
     *  step 2: 找到最优余弦距离的Scancontext算子
     */

    for ( int candidate_iter_idx = 0; candidate_iter_idx < NUM_CANDIDATES_FROM_TREE; candidate_iter_idx++ )
    {
        Eigen::MatrixXd polarcontext_candidate = m_polarcontexts[ candidate_indexes[candidate_iter_idx] ];
        std::pair<double, int> sc_dist_result = distanceBtnScanContext( curr_desc, polarcontext_candidate ); 
        
        double candidate_dist = sc_dist_result.first;
        int candidate_align = sc_dist_result.second;

        if( candidate_dist < min_dist )
        {
            min_dist = candidate_dist;
            nn_align = candidate_align;

            nn_idx = candidate_indexes[candidate_iter_idx];
        }
    }


    /* 
     * 打印配对上的索引
     */
    if( min_dist < SC_DIST_THRES )
    {
        loop_id = nn_idx; 
    
        // std::cout.precision(3); 
        cout << "[Loop found] Nearest distance: " << min_dist << " btn " << m_polarcontexts.size()-1 << " and " << nn_idx << "." << endl;
        cout << "[Loop found] yaw diff: " << nn_align * PC_UNIT_SECTORANGLE << " deg." << endl;
    }
    else
    {
        std::cout.precision(3); 
        cout << "[Not loop] Nearest distance: " << min_dist << " btn " << m_polarcontexts.size()-1 << " and " << nn_idx << "." << endl;
        cout << "[Not loop] yaw diff: " << nn_align * PC_UNIT_SECTORANGLE << " deg." << endl;
    }

    // Todo: 返回暴力对齐的角度
    float yaw_diff_rad = deg2rad(nn_align * PC_UNIT_SECTORANGLE);
    std::pair<int, float> result {loop_id, yaw_diff_rad};

    return result;

}
/***********************************************************************************
Function:     detectLoopClosureIDqueue
Description:  detectLoopClosureIDqueue
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
std::deque<std::pair<int, float>> CScancontext::detectLoopClosureIDqueue()
{
    int loop_id { -1 }; // 用于回环检测的

    auto curr_key  =  m_polarcontext_invkeysmat.back(); // 当前点云生成的算子
    auto curr_desc =  m_polarcontexts.back();           
    std::deque<std::pair<int, float>>  result2;
    /* 
     * step 1: 通过Ring 算子挑选出候选者
     */
    if( (int)m_polarcontext_invkeysmat.size() < NUM_EXCLUDE_RECENT + 1)
    {
        std::pair<int, float> result {loop_id, 0.0};
        return result2; // 错误返回
    }

    // KD树重建频率
    if( m_tree_making_period_conter % TREE_MAKING_PERIOD_ == 0) // to save computation cost
    {
        m_polarcontext_invkeysto_search.clear();
        m_polarcontext_invkeysto_search.assign( m_polarcontext_invkeysmat.begin(), m_polarcontext_invkeysmat.end() - NUM_EXCLUDE_RECENT ) ;

        m_polarcontext_tree.reset(); 
        m_polarcontext_tree = std::make_unique<InvKeyTree>(PC_NUM_RING, m_polarcontext_invkeysto_search, 10 ); // 维度 ， 待搜索数据 , 最大叶子节点
    }
    m_tree_making_period_conter = m_tree_making_period_conter + 1;
        
    double min_dist = 10000000; //初始化最小距离
    int nn_align = 0;
    int nn_idx = 0;

    // knn search
    std::vector<size_t>   candidate_indexes( NUM_CANDIDATES_FROM_TREE ); 
    std::vector<float>    out_dists_sqr( NUM_CANDIDATES_FROM_TREE );


    nanoflann::KNNResultSet<float> knnsearch_result( NUM_CANDIDATES_FROM_TREE );
    knnsearch_result.init( &candidate_indexes[0], &out_dists_sqr[0] );
    m_polarcontext_tree->index->findNeighbors( knnsearch_result, &curr_key[0] /* query */, nanoflann::SearchParams(10) ); 


    /* 
     *  step 2: 找到最优余弦距离的Scancontext算子
     */

    for ( int candidate_iter_idx = 0; candidate_iter_idx < NUM_CANDIDATES_FROM_TREE; candidate_iter_idx++ )
    {
        Eigen::MatrixXd polarcontext_candidate = m_polarcontexts[ candidate_indexes[candidate_iter_idx] ];

        nn_idx = candidate_indexes[candidate_iter_idx];
        std::pair<int, float> result {nn_idx, 0};
        result2.push_back(result);
    }

    return result2;
}
/***********************************************************************************
Function:     saveSCD
Description:  saveSCD
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
void CScancontext::saveSCD(std::string fileName, Eigen::MatrixXd matrix, std::string delimiter)
{
    // delimiter: ", " or " " etc.

    int precision = 3; // or Eigen::FullPrecision, but SCD does not require such accruate precisions so 3 is enough.
    const static Eigen::IOFormat the_format(precision, Eigen::DontAlignCols, delimiter, "\n");
 
    std::ofstream file(fileName);
    if (file.is_open())
    {
        file << matrix.format(the_format);
        file.close();
    }
}
/***********************************************************************************
Function:     saveSCD
Description:  saveSCD
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
std::string CScancontext::padZeros(int val, int num_digits ) 
{
  std::ostringstream out;
  out << std::internal << std::setfill('0') << std::setw(num_digits) << val;
  return out.str();
}
/***********************************************************************************
Function:     saveSCD
Description:  saveSCD
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
void CScancontext::saveAllSccontext()
{
    for (int i = 0; i < m_polarcontexts.size(); i++)
    {
         std::string curr_scd_node_idx = padZeros(i);
         saveSCD(saveSCDDirectory + curr_scd_node_idx + ".scd", m_polarcontexts[i] );
    }
    
}
/***********************************************************************************
Function:     loadSCD
Description:  loadSCD
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
void CScancontext::loadSCD(std::string fileName, Eigen::MatrixXd& matrix, char delimiter )
{
    // delimiter: ", " or " " etc.
    std::vector<double> matrixEntries;
    std::ifstream matrixDataFile(fileName);
    if (!matrixDataFile.is_open())
    {
        std::cout << "读入SCD文件失败!!"<< std::endl;
        return;
    }

    std::string matrixRowString;
    std::string matrixEntry;
    int matrixRowNumber = 0;
    while (getline(matrixDataFile, matrixRowString)) 
    {
        std::stringstream matrixRowStringStream(matrixRowString);
 
        while (getline(matrixRowStringStream, matrixEntry,delimiter)) 
        {
            matrixEntries.push_back(stod(matrixEntry)); 
        }
        matrixRowNumber++; 
    }
    
    matrix =  Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(matrixEntries.data(), matrixRowNumber, matrixEntries.size() / matrixRowNumber);
    matrixDataFile.close();
}

/***********************************************************************************
Function:     loadAllSCD
Description:  loadAllSCD
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
void CScancontext::loadAllSCD(char delimiter)
{
    std::string loadSCDDirectory = "/home/maxiaovivi/doubletx-offline/doubletx-offline/allscd/global_scancontext/"; // 替换为你的目录
    int count = 0;

    fs::path directory(loadSCDDirectory);
    fs::directory_iterator end_itr;

    // 遍历目录计算文件数
    for (fs::directory_iterator itr(directory); itr != end_itr; ++itr) {
        if (fs::is_regular_file(itr->status())) { count++; }
    }

    std::cout << "maxiao  Total number of files: " << count << std::endl;

    for(int i = 0; i < count; ++i) {
        std::string filename = padZeros(i);
        std::string scd_path = loadSCDDirectory +"scContext"+ filename + ".scd";
        Eigen::MatrixXd load_sc;
        //std::cout<<"当前读入的文件名："<<scd_path <<std::endl;
        loadSCD(scd_path, load_sc);
        
        // 加载关键点
        Eigen::MatrixXd ringkey = makeRingkeyFromScancontext(load_sc);
        Eigen::MatrixXd sectorkey = makeSectorkeyFromScancontext(load_sc);
        std::vector<float> polarcontext_invkey_vec = eig2stdvec(ringkey);

        m_polarcontexts.push_back(load_sc);
        m_polarcontext_invkeys.push_back(ringkey);
        m_polarcontext_vkeys.push_back(sectorkey);
        m_polarcontext_invkeysmat.push_back(polarcontext_invkey_vec);
    }
}
/***********************************************************************************
Function:     loadAllSCD
Description:  loadAllSCD
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
const Eigen::MatrixXd &CScancontext::getConstRefRecentSCD(void)
{
    return m_polarcontexts.back();
}
/***********************************************************************************
Function:     loadPoses
Description:  loadPoses
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
void CScancontext::loadPoses()
{
    std::string filePath = "/home/maxiaovivi/doubletx-offline/doubletx-offline/allscd/scPose.txt";
    std::ifstream file(filePath);
    std::string line;
    while (std::getline(file, line)) {
        std::istringstream iss(line);
        PointTypePose pose;
        if (!(iss >> pose.x >> pose.y >> pose.z >> pose.roll >> pose.pitch >> pose.yaw)) {
            break;  // 发生错误或格式不匹配时停止读取
        }
        m_poses.push_back(pose);
    }
    
    file.close();

    std::cout<<"读入位姿数量" << m_poses.size()<<std::endl;
}
/***********************************************************************************
Function:     enoughPointCheck
Description:  enoughPointCheck
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
bool CScancontext::enoughPointCheck(pcl::PointCloud<PointType>::Ptr couldFrame,PointTypePose cloud_pose)
{
    // TODO  需要我传入SC成功时候的pose
    PointTypePose thisPose6D;
    thisPose6D = cloud_pose;
    pcl::PointCloud<PointType>::Ptr Trans2mScan = transformPointCloud(couldFrame, &thisPose6D);
    pcl::PointCloud<PointType>::Ptr this_scan_cloud(new pcl::PointCloud<PointType>());
    *this_scan_cloud = transform2Scan(Trans2mScan, 0.05, 0.6, true, 0.02); // 0.05  0.5
    std::cout << "this_scan_cloud->size() " << this_scan_cloud->size() << std::endl;

    return (int)this_scan_cloud->size() > 90;
}
/***********************************************************************************
Function:     relo_scan2MapOptimization
Description:  relo_scan2MapOptimization
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
bool CScancontext::relo_scan2MapOptimization(pcl::PointCloud<PointType>::Ptr local_cloud ,PointTypePose source_cloud_pose,PointTypePose target_cloud_pose)
{

    if (!enoughPointCheck(local_cloud, source_cloud_pose))
    {
        std::cout<<"maxiao 没有足够多的点"<<std::endl;
        return false;
    }
    // 计算从源位姿到目标位姿的变换矩阵
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.translation() << source_cloud_pose.x, source_cloud_pose.y, source_cloud_pose.z;
    // 根据欧拉角创建旋转（按照Z, Y, X顺序，即先偏航、再俯仰、最后滚转）
    transform.rotate(Eigen::AngleAxisf(source_cloud_pose.yaw, Eigen::Vector3f::UnitZ()) *
                     Eigen::AngleAxisf(source_cloud_pose.pitch, Eigen::Vector3f::UnitY()) *
                     Eigen::AngleAxisf(source_cloud_pose.roll, Eigen::Vector3f::UnitX()));
    Eigen::Affine3f inverseTransform = transform.inverse();

    Eigen::Affine3f targetTransform = Eigen::Affine3f::Identity();
    targetTransform.translation() << target_cloud_pose.x, target_cloud_pose.y, target_cloud_pose.z;
    targetTransform.rotate(Eigen::AngleAxisf(target_cloud_pose.yaw, Eigen::Vector3f::UnitZ()) *
                           Eigen::AngleAxisf(target_cloud_pose.pitch, Eigen::Vector3f::UnitY()) *
                           Eigen::AngleAxisf(target_cloud_pose.roll, Eigen::Vector3f::UnitX()));

    // 综合两个变换
    Eigen::Affine3f finalTransform = targetTransform * inverseTransform;

    // 应用变换到点云
    pcl::PointCloud<PointType>::Ptr transformed_cloud(new pcl::PointCloud<PointType>());
    pcl::transformPointCloud(*local_cloud, *transformed_cloud, finalTransform);

    // std::stringstream filename4;
    // static int count4 =0;
    // filename4 << "/home/maxiaovivi/doubletx-offline/doubletx-offline/icp_test/scPCD" << std::setw(6) << std::setfill('0') << count4 << ".pcd";
    // pcl::io::savePCDFileASCII(filename4.str(), *transformed_cloud);
    // std::cout <<"maxiao Saved " << filename4.str() <<std::endl;
    // // 此处执行对齐操作
    // count4++;

   // std::cout << "maxiao :   "<< target_cloud_pose.x << " "<<target_cloud_pose.y<< " "<<target_cloud_pose.z<<" "<<target_cloud_pose.roll
    //<<"  "<<target_cloud_pose.pitch<<"  "<<target_cloud_pose.yaw<<std::endl;
    return relo_bruceAlignment(transformed_cloud, target_cloud_pose);

}
/***********************************************************************************
Function:     relo_bruceAlignment
Description:  relo_bruceAlignment
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
bool CScancontext::relo_bruceAlignment(pcl::PointCloud<PointType>::Ptr local_cloud ,PointTypePose local_cloud_pose)
{
   return relo_angularBruceAlignment(local_cloud, local_cloud_pose) || relo_linearBruceAlignment(local_cloud, local_cloud_pose);
   //return relo_angularBruceAlignment(local_cloud, local_cloud_pose);
}
/***********************************************************************************
Function:     relo_angularBruceAlignment
Description:  relo_angularBruceAlignment
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
bool CScancontext::relo_angularBruceAlignment(pcl::PointCloud<PointType>::Ptr local_cloud ,PointTypePose local_cloud_pose)
{
    // sub_map
     pcl::PointCloud<PointType>::Ptr this_submap_cloud(new pcl::PointCloud<PointType>());
    // this_submap_cloud = m_global_scan;

    // set search window
    float transform_tobe[6];
    transform_tobe[0] = local_cloud_pose.roll;
    transform_tobe[1] = local_cloud_pose.pitch;
    transform_tobe[2] = local_cloud_pose.yaw;
    transform_tobe[3] = local_cloud_pose.x;
    transform_tobe[4] = local_cloud_pose.y;
    transform_tobe[5] = local_cloud_pose.z;

    Eigen::Affine3f transTobe = trans2Affine3f(transform_tobe);
    float angular_search_window = 60.0 * DEG_TO_ARC; // 4.5
    int step_count = 13; // 4

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
#pragma omp parallel for num_threads(2)
    for (int i = 0; i < (int)angular_candidate.size(); i++) {
        float temp_angular_candidate[6] = {0., 0., 0., 0., 0., 0.};
        pcl::getTranslationAndEulerAngles(angular_candidate[i], temp_angular_candidate[3], temp_angular_candidate[4], temp_angular_candidate[5],
                                            temp_angular_candidate[0], temp_angular_candidate[1], temp_angular_candidate[2]);
        double final_score = 0.0;
        float temp_angular_search_result[6] = {0., 0., 0., 0., 0., 0.};
        // std::cout << "-------------angular search " << i << " start-------------" << std::endl;
        bool is_good = relo_scanMatchForBruceAlignment(temp_angular_candidate, local_cloud, this_submap_cloud, temp_angular_search_result, final_score);
        // std::cout << "-------------angular search " << i << " finish:" << is_good << "-------------" << std::endl;
        angular_search_results[i] = std::make_pair(is_good, std::make_pair(final_score, trans2Affine3f(temp_angular_search_result) ) );
    }
    this_submap_cloud->clear();

    // set score theshold
    double angular_score_thres = 0.3;
    float angular_xy_thres = 1;

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

        // std::cout << "maxiao : angular search result "
        // <<"is_good "<<angular_search_results[i].first<<" "
        // <<"score "  << (angular_search_results[i].second).first<< " "
        // <<"pose "  
        //             << temp_debug[3] << " "
        //             << temp_debug[4] << " "
        //             << temp_debug[5] << " "
        //             << temp_debug[0] << " "
        //             << temp_debug[1] << " "
        //             << temp_debug[2] << " " << std::endl;
    }

    float angular_dummy[6];
    Eigen::Affine3f angular_transBetween = transTobe.inverse() * (angular_search_results.front().second).second;
    pcl::getTranslationAndEulerAngles(angular_transBetween, angular_dummy[3], angular_dummy[4], angular_dummy[5], angular_dummy[0], angular_dummy[1], angular_dummy[2]);
    float final_delta_xy = sqrt(pow(angular_dummy[3], 2) + pow(angular_dummy[4], 2) );


    std::stringstream filename5;
    static int count5 =0;
    // transport result
    if ((angular_search_results.front().second).first < angular_score_thres && final_delta_xy < angular_xy_thres)
    {
        pcl::getTranslationAndEulerAngles(
            (angular_search_results.front().second).second,
            m_pose_after_relo_BruceAlignment[3],
            m_pose_after_relo_BruceAlignment[4],
            m_pose_after_relo_BruceAlignment[5],
            m_pose_after_relo_BruceAlignment[0],
            m_pose_after_relo_BruceAlignment[1],
            m_pose_after_relo_BruceAlignment[2]
        );

        

        std::cout << "-------------maxiao angular search quit SUCCEEDED -------------" << std::endl;
        return true;
    }
    else
    {
        std::cout<< "maxiao icp 失败  -----------最佳得分为：    "<<(angular_search_results.front().second).first <<std::endl;
        std::cout<< "maxiao icp 失败  -----------产生位移为：    "<<final_delta_xy <<std::endl;
    }
    std::cout << "-------------maxiao angular search quit FAILED -------------" << std::endl;
    return false;
}
/***********************************************************************************
Function:     relo_ linearBruceAlignment
Description:  relo_ linearBruceAlignment
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
bool CScancontext::relo_linearBruceAlignment(pcl::PointCloud<PointType>::Ptr local_cloud ,PointTypePose local_cloud_pose)
{
    // sub_map
     pcl::PointCloud<PointType>::Ptr this_submap_cloud(new pcl::PointCloud<PointType>());
    // this_submap_cloud = m_global_scan;

    // set search window

    float transform_tobe[6];
    transform_tobe[0] = local_cloud_pose.roll;
    transform_tobe[1] = local_cloud_pose.pitch;
    transform_tobe[2] = local_cloud_pose.yaw;
    transform_tobe[3] = local_cloud_pose.x;
    transform_tobe[4] = local_cloud_pose.y;
    transform_tobe[5] = local_cloud_pose.z;

    Eigen::Affine3f transTobe = trans2Affine3f(transform_tobe);
    int step_count_H = 5; // 向左右,y
    int step_count_V = 5; // 向前后,x
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
#pragma omp parallel for num_threads(2)
    for (int i = 0; i < (int)linear_candidate.size(); i++) {
        float temp_linear_candidate[6] = {0., 0., 0., 0., 0., 0.};
        pcl::getTranslationAndEulerAngles(linear_candidate[i], temp_linear_candidate[3], temp_linear_candidate[4], temp_linear_candidate[5],
                                            temp_linear_candidate[0], temp_linear_candidate[1], temp_linear_candidate[2]);
        double final_score = 0.0;
        float temp_linear_search_result[6] = {0., 0., 0., 0., 0., 0.};
        // std::cout << "-------------linear search " << i << " start-------------" << std::endl;
        bool is_good = relo_scanMatchForBruceAlignment(temp_linear_candidate, local_cloud, this_submap_cloud, temp_linear_search_result, final_score);
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
    }

    float linear_dummy[6];
    Eigen::Affine3f linear_transBetween = transTobe.inverse() * (linear_search_results.front().second).second;
    pcl::getTranslationAndEulerAngles(linear_transBetween, linear_dummy[3], linear_dummy[4], linear_dummy[5], linear_dummy[0], linear_dummy[1], linear_dummy[2]);
    float final_delta_yaw = abs(linear_dummy[2]);

    // transport result
    if ((linear_search_results.front().second).first < linear_score_thres && final_delta_yaw < linear_yaw_thres) {
        pcl::getTranslationAndEulerAngles(
            (linear_search_results.front().second).second,
            m_pose_after_relo_BruceAlignment[3],
            m_pose_after_relo_BruceAlignment[4],
            m_pose_after_relo_BruceAlignment[5],
            m_pose_after_relo_BruceAlignment[0],
            m_pose_after_relo_BruceAlignment[1],
            m_pose_after_relo_BruceAlignment[2]
        );

        std::cout << "-------------maxiao linear search quit SUCCEEDED -------------" << std::endl;
        return true;
    }
    std::cout << "-------------maxiao linear search quit FAILED -------------" << std::endl;
    return false;
}
/***********************************************************************************
Function:     loadSavedGlobalMap
Description:  loadSavedGlobalMap
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
void CScancontext::loadSavedGlobalMap()
{
        // 文件路径
        std::string file_path = "/home/maxiaovivi/doubletx-offline/doubletx-offline/allscd/saved_global_map/global_map.pcd";

        // 读取PCD文件
        if (pcl::io::loadPCDFile<PointType>(file_path, *m_tof_cloud_saved_global_map) == -1) {
            std::cout<<" maxiao 读取全局地图失败"<<std::endl;
            return; // 根据你的函数返回类型，这里可能需要返回特定的值
        }
        std::cout << std::endl;
        std::cout << "Loaded "
                << m_tof_cloud_saved_global_map->width * m_tof_cloud_saved_global_map->height<< std::endl
                << "maxiao data points from test_pcd.pcd with the following fields: "
                << std::endl;

       // pcl::PointCloud<PointType>::Ptr m_global_scan(new pcl::PointCloud<PointType>());
        *m_global_scan = transform2Scan(m_tof_cloud_saved_global_map, 0.03, 0.6, true, 0.05);    
        std::cout<<"maxiao baocunhaole "<<std::endl;
        std::stringstream filename6;
        filename6 << "/home/maxiaovivi/doubletx-offline/doubletx-offline/allscd/saved_global_map/m_global_scan" << std::setw(6) << std::setfill('0') << 0 << ".pcd";
        pcl::io::savePCDFileASCII(filename6.str(), *m_global_scan);
        std::cout<<"maxiao zhen neng baocuna "<<std::endl;

}
/***********************************************************************************
Function:     transform2Scan
Description:  transform2Scan
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
pcl::PointCloud<pcl::PointXYZI> CScancontext::transform2Scan(pcl::PointCloud<pcl::PointXYZI>::Ptr cloudIn, float z_min, float z_max, bool ds_flag, float resolution)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZI>());

    // Filter points by z-axis limits
    for (const auto& point : *cloudIn) {
        if (point.z > z_min && point.z < z_max) {
            filtered_cloud->push_back(point);
        }
    }

    pcl::PointCloud<pcl::PointXYZI> downsampled_cloud;
    if (ds_flag) {
        // Using octree for downsampling
        pcl::octree::OctreePointCloudVoxelCentroid<pcl::PointXYZI> octree(resolution);
        octree.setInputCloud(filtered_cloud);
        octree.addPointsFromInputCloud();

        std::vector<pcl::PointXYZI, Eigen::aligned_allocator<pcl::PointXYZI>> centroids;
        octree.getVoxelCentroids(centroids);
        downsampled_cloud.points = centroids;  // Assign the centroids to the point cloud
        downsampled_cloud.width = centroids.size();
        downsampled_cloud.height = 1;
        downsampled_cloud.is_dense = false;
    } else {
        pcl::copyPointCloud(*filtered_cloud, downsampled_cloud);
    }

    return downsampled_cloud;
}
/***********************************************************************************
Function:     pclPointToAffine3f
Description:  pclPointToAffine3f
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
Eigen::Affine3f CScancontext::pclPointToAffine3f(PointTypePose thisPoint)
{ 
    return pcl::getTransformation(thisPoint.x, thisPoint.y, thisPoint.z, thisPoint.roll, thisPoint.pitch, thisPoint.yaw);
}
/***********************************************************************************
Function:     constraintTransformation
Description:  constraintTransformation
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
float CScancontext::constraintTransformation(float value, float limit)
{
    if (value < -limit)
        value = -limit;
    if (value > limit)
        value = limit;

    return value;
}
/***********************************************************************************
Function:     relo_ scanMatchForBruceAlignment
Description:  relo_ scanMatchForBruceAlignment
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
bool CScancontext::relo_scanMatchForBruceAlignment(const float transformIn[6], const pcl::PointCloud<PointType>::Ptr CloudInput, const pcl::PointCloud<PointType>::Ptr CloudTarget, float (&transformOut)[6], double &final_score)
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
    
    // std::cout<<"maxiao 输入 位姿结果： "
    // << transformIn[3] <<"  "<<transformIn[4]<<" "<<transformIn[5]<<"  "<<transformIn[0]
    // <<"  "<<transformIn[1]<<"  "<<transformIn[2]<<" "<<std::endl;
    // pcl::search::KdTree<PointType>::Ptr tree(new pcl::search::KdTree<PointType>);
    // pcl::MovingLeastSquares<PointType, PointType> mls;

    // mls.setInputCloud(CloudInput);
    // mls.setPolynomialFit(true);
    // mls.setSearchMethod(tree);
    // mls.setSearchRadius(0.03); // 搜索半径取决于数据的平均密度

    // pcl::PointCloud<PointType>::Ptr cloud_smoothed(new pcl::PointCloud<PointType>());
    // mls.process(*cloud_smoothed);

    

    pcl::PointCloud<PointType>::Ptr Trans2mScan = transformPointCloud(CloudInput, &thisPose6D);
    pcl::PointCloud<PointType>::Ptr this_scan_cloud(new pcl::PointCloud<PointType>());

    *this_scan_cloud = transform2Scan(Trans2mScan, 0.01, 0.6, true, 0.02); // 0.05  0.5

    pcl::PointCloud<PointType>::Ptr this_submap_cloud(new pcl::PointCloud<PointType>());
    pcl::copyPointCloud(*this_scan_cloud, *this_submap_cloud);   
 
    auto start = std::chrono::high_resolution_clock::now();

    pcl::IterativeClosestPoint<PointType, PointType> icp;
    pcl::registration::TransformationEstimation2D<PointType, PointType>::Ptr est;
    est.reset(new pcl::registration::TransformationEstimation2D<PointType, PointType>);
    icp.setTransformationEstimation(est);
    icp.setMaxCorrespondenceDistance(0.8);
    icp.setMaximumIterations(50);
    icp.setTransformationEpsilon(0.001);
    icp.setEuclideanFitnessEpsilon(0.1);
    icp.setRANSACIterations(0);

    pcl::PointCloud<PointType>::Ptr copied_cloud(new pcl::PointCloud<PointType>);
    pcl::copyPointCloud(*m_global_scan, *copied_cloud);

    // pcl::PointCloud<PointType>::Ptr copied_cloud(new pcl::PointCloud<PointType>);
    // pcl::copyPointCloud(*m_tof_cloud_saved_global_map, *copied_cloud);

    // Align clouds
    icp.setInputSource(Trans2mScan);
    icp.setInputTarget(copied_cloud);
    pcl::PointCloud<PointType>::Ptr unused_result(new pcl::PointCloud<PointType>());

    icp.align(*unused_result);
    score = icp.getFitnessScore();

    // 记录结束时间
    auto end = std::chrono::high_resolution_clock::now();

    // 计算持续时间
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();

    // 输出结果
    std::cout << "datatime Function took " << duration << " milliseconds to execute.\n";

    // Get pose transformation
    Eigen::Affine3f correctionLidarFrame;
    correctionLidarFrame = icp.getFinalTransformation();
    // Judge
    if (icp.hasConverged() == true && score < 0.3){
        // sai: TODO CHECK TRANSFORMATION -> icp.getFinalTransformation()
        // sai: TODO 在点云密集时，有大位移和大角度偏差的可能
        float delta_x, delta_y, delta_z, delta_roll, delta_pitch, delta_yaw;
        pcl::getTranslationAndEulerAngles(correctionLidarFrame, delta_x, delta_y, delta_z, delta_roll, delta_pitch, delta_yaw);
        //std::cout << "maxiao scanMatch score & delta x y yaw " << setprecision(6) << score << " " << delta_x << " " << delta_y << " " << delta_yaw << std::endl;
        // CLog::logAsync(LogSara, LogNormal, "[C3DSlam][CMapOptimization] scanMatch score %.6f delta x y z roll pitch yaw %.6f %.6f %.6f %.6f %.6f %.6f \n", icp.getFitnessScore(), delta_x, delta_y, delta_z, delta_roll, delta_pitch, delta_yaw);
        
        // 获取变换矩阵
        Eigen::Matrix4f transformation = icp.getFinalTransformation();
        pcl::PointCloud<PointType>::Ptr transformed_cloud1(new pcl::PointCloud<PointType>);
        pcl::transformPointCloud(*Trans2mScan, *transformed_cloud1, transformation);

        // std::stringstream filename7;
        // std::stringstream filename8;
        // static int count7 =0;
        // filename7 << "/home/maxiaovivi/doubletx-offline/doubletx-offline/allscd/icp_test/icp" << std::setw(6) << std::setfill('0') << count7 << "before.pcd";
        // pcl::io::savePCDFileASCII(filename7.str(), *Trans2mScan);
        // filename8 << "/home/maxiaovivi/doubletx-offline/doubletx-offline/allscd/icp_test/icp" << std::setw(6) << std::setfill('0') << count7 << "after.pcd";
        // pcl::io::savePCDFileASCII(filename8.str(), *transformed_cloud1);
        // // 此处执行对齐操作
        // count7++;
        
        if (sqrt(pow(delta_x,2)+pow(delta_y,2)) > 0.5 || abs(delta_yaw) > 0.2 ) {
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

    transformOut[0] = constraintTransformation(roll, 0.1);   // 0.05是旋转的限制约束
    transformOut[1] = constraintTransformation(pitch, 0.1);
    transformOut[2] = yaw;
    transformOut[3] = x;
    transformOut[4] = y;
    transformOut[5] = constraintTransformation(z, 0.02);   // z轴的最小约束

    // std::cout<<"maxiao icp直接分数 ： "<<score << " 位姿结果： "
    // << transformOut[3] <<"  "<<transformOut[4]<<" "<<transformOut[5]<<"  "<<transformOut[0]
    // <<"  "<<transformOut[1]<<"  "<<transformOut[2]<<" "<<std::endl;

    unused_result->clear();
    this_scan_cloud->clear();
    this_submap_cloud->clear();
    final_score = score;
    return scanmatch_sucess;
}
/***********************************************************************************
Function:     relo_ scanMatchForBruceAlignment
Description:  relo_ scanMatchForBruceAlignment
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
Eigen::Affine3f CScancontext::trans2Affine3f(float transformIn[])
{
    return pcl::getTransformation(transformIn[3], transformIn[4], transformIn[5], transformIn[0], transformIn[1], transformIn[2]);
}