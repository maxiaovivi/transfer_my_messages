#include "include/myicp_helpers.h"

void calNearestPointPairs(Eigen::Matrix4f H, pcl::PointCloud<PointType>::Ptr& source_cloud, const pcl::PointCloud<PointType>::Ptr& target_cloud,
	pcl::PointCloud<PointType>::Ptr& target_cloud_mid, pcl::search::KdTree<PointType>::Ptr kdtree, double &error)
{
	double err = 0.0;
	pcl::transformPointCloud(*source_cloud, *source_cloud, H);
	std::vector<int>indexs(source_cloud->size());

#pragma omp parallel for reduction(+:err)
	for (int i = 0; i < source_cloud->size(); ++i)
	{
		std::vector<int>index(1);
		std::vector<float>distance(1);
		kdtree->nearestKSearch(source_cloud->points[i], 1, index, distance);
		err = err + sqrt(distance[0]);
		indexs[i] = index[0];
	}

	pcl::copyPointCloud(*target_cloud, indexs, *target_cloud_mid);
	error = err / source_cloud->size();
}