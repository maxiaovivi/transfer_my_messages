#include "include/myicp.h"
#include "include/myicp_helpers.h"

MyICP::MyICP()
{

}

MyICP::~MyICP()
{

}

void MyICP::setSourceCloud(pcl::PointCloud<PointType>::Ptr cloud) 
{
	std::cout << "[MyICP] " << __FUNCTION__ << std::endl;
	source_cloud = cloud;
}

void MyICP::setTargetCloud(pcl::PointCloud<PointType>::Ptr cloud)
{
	std::cout << "[MyICP] " << __FUNCTION__ << std::endl;
	target_cloud = cloud;
}

void MyICP::setLeafSize(float size)
{
	std::cout << "[MyICP] " << __FUNCTION__ << std::endl;
	leaf_size = size;
}

void MyICP::setMinError(float error)
{
	std::cout << "[MyICP] " << __FUNCTION__ << std::endl;
	min_error = error;
}

void MyICP::setMaxIters(int iters)
{
	std::cout << "[MyICP] " << __FUNCTION__ << std::endl;
	max_iters = iters;
}

void MyICP::setEpsilon(float eps)
{
	epsilon = eps;
}

void MyICP::downsample()
{
	std::cout << "[MyICP] " << __FUNCTION__ << std::endl;
	// pcl::VoxelGrid<PointType> voxel_grid;
	// voxel_grid.setLeafSize(leaf_size, leaf_size, leaf_size);
	// voxel_grid.setInputCloud(source_cloud);
	// source_cloud_downsampled.reset(new pcl::PointCloud<PointType>);
	// voxel_grid.filter(*source_cloud_downsampled);
	// voxel_grid.setInputCloud(target_cloud);
	// target_cloud_downsampled.reset(new pcl::PointCloud<PointType>);
	// voxel_grid.filter(*target_cloud_downsampled);

	source_cloud_downsampled.reset(new pcl::PointCloud<PointType>);
	target_cloud_downsampled.reset(new pcl::PointCloud<PointType>);
	*source_cloud_downsampled = *source_cloud;
	*target_cloud_downsampled = *target_cloud;

	std::cout << "down size *cloud_src_o from " << source_cloud->size() << " to " << source_cloud_downsampled->size() << std::endl;
	std::cout << "down size *cloud_tgt_o from " << target_cloud->size() << " to " << target_cloud_downsampled->size() << std::endl;
}

void MyICP::registration()
{
	std::cout << "[MyICP] " << __FUNCTION__ << std::endl;
	std::cout << "icp registration start..." << std::endl;

	Eigen::Matrix3f R_12 = Eigen::Matrix3f::Identity();
	Eigen::Vector3f T_12 = Eigen::Vector3f::Zero();
	Eigen::Matrix4f H_12 = Eigen::Matrix4f::Identity();

	pcl::PointCloud<PointType>::Ptr target_cloud_mid(new pcl::PointCloud<PointType>());

	//建立kd树
	pcl::search::KdTree<PointType>::Ptr kdtree(new pcl::search::KdTree<PointType>);
	kdtree->setInputCloud(target_cloud_downsampled);

	double error = INT_MAX, score = INT_MAX;
	Eigen::Matrix4f H_final = H_12;
	int iters = 0;

	//开始迭代，直到满足条件
	while (error > min_error && iters < max_iters)
	{
		std::cout << "[MyICP] " << __FUNCTION__ << " error: " << error << " min_error: " << min_error << " iters: " << iters << " max_iters: " << max_iters << std::endl;
		iters++;
		double last_error = error;

		//计算最邻近点对
		calNearestPointPairs(H_12, source_cloud_downsampled, target_cloud_downsampled, target_cloud_mid, kdtree, error);

		// if (last_error - error < epsilon)
		// 	break;

		//计算点云中心坐标
		Eigen::Vector4f source_centroid, target_centroid_mid;
		pcl::compute3DCentroid(*source_cloud_downsampled, source_centroid);
		pcl::compute3DCentroid(*target_cloud_mid, target_centroid_mid);

		//去中心化
		Eigen::MatrixXf souce_cloud_demean, target_cloud_demean;
		pcl::demeanPointCloud(*source_cloud_downsampled, source_centroid, souce_cloud_demean);
		pcl::demeanPointCloud(*target_cloud_mid, target_centroid_mid, target_cloud_demean);

		//计算W=q1*q2^T
		Eigen::Matrix3f W = (souce_cloud_demean*target_cloud_demean.transpose()).topLeftCorner(3, 3);

		//SVD分解得到新的旋转矩阵和平移矩阵
		Eigen::JacobiSVD<Eigen::Matrix3f> svd(W, Eigen::ComputeFullU | Eigen::ComputeFullV);
		Eigen::Matrix3f U = svd.matrixU();
		Eigen::Matrix3f V = svd.matrixV();

		if (U.determinant()*V.determinant() < 0)
		{
			for (int x = 0; x < 3; ++x)
				V(x, 2) *= -1;
		}

		R_12 = V* U.transpose();
		T_12 = target_centroid_mid.head(3) - R_12*source_centroid.head(3);
		H_12 << R_12, T_12, 0, 0, 0, 1;
		H_final = H_12*H_final; //更新变换矩阵

		std::cout << "iters:"  << iters << "  "<< "error:" << error << std::endl;
	}
	transformation_matrix << H_final;
}

void MyICP::saveICPCloud(const std::string filename)
{
	icp_cloud.reset(new pcl::PointCloud<PointType>);
	pcl::transformPointCloud(*source_cloud, *icp_cloud, transformation_matrix); //点云变换
	// pcl::io::savePCDFileBinary(filename, *icp_cloud);
}

Eigen::Matrix4f MyICP::getTransformationMatrix()
{
	std::cout << "[MyICP] " << __FUNCTION__ << std::endl;
	std::cout << "transformation_matrix:" << std::endl << transformation_matrix << std::endl;
	return transformation_matrix;
}

double MyICP::getScore()
{
	std::cout << "[MyICP] " << __FUNCTION__ << std::endl;
	double fitness_score = 0.0;
	pcl::KdTreeFLANN <PointType> kdtree;
	kdtree.setInputCloud(target_cloud);

#pragma omp parallel for reduction(+:fitness_score) //采用openmmp加速
	for (int i = 0; i < icp_cloud->points.size(); ++i)
	{
		std::vector<int> nn_indices(1);
		std::vector<float> nn_dists(1);
		kdtree.nearestKSearch(icp_cloud->points[i], 1, nn_indices, nn_dists);
		fitness_score += nn_dists[0];
	}

	std::cout << "score:" << std::endl << fitness_score / icp_cloud->points.size() << std::endl;

	double score = fitness_score / icp_cloud->points.size();

	return score;
}

