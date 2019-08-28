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

	// create downsampled VoxelGrid for the map

	// get covariances and means for voxelgrid

	/**
	 * Loop until converges
	 *
	 * 	for all points
	 * 		transform pose_guess with PointCloud from filtered points
	 * 		find cell containing each point
	 * 		calculate score based on distribution
	 * 		calculate jacobian
	 * 		calculate hessian
	 *
	 * 	solve H*dp = -g
	 * 	move pose_guess by dp
	 */

	return estimate;
}

}  // namespace ndt_matching
