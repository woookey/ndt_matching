#include "ndt_matching/ndt_lib.hpp"

namespace ndt_matching
{

NdtLib::NdtLib()
{
}

NdtLib::~NdtLib()
{
}

void NdtLib::register_map(pcl::PointCloud<pcl::PointXYZ>& point_cloud)
{
	point_cloud_ = point_cloud;
	map_registered = true;
}

geometry_msgs::msg::PoseStamped NdtLib::execute(pcl::PointCloud<pcl::PointXYZ> &lidar_point_cloud,
		  geometry_msgs::msg::PoseStamped &pose_guess)
{
	geometry_msgs::msg::PoseStamped estimate = pose_guess;
	(void)lidar_point_cloud;
	(void)pose_guess;

	//pcl::VoxelGrid

	return estimate;
}

}  // namespace ndt_matching
