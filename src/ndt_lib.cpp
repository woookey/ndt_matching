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
	geometry_msgs::msg::PoseStamped estimate;
	(void)lidar_point_cloud;
	(void)pose_guess;

	estimate.header = pose_guess.header;
	estimate.pose.position.x = 1;
	estimate.pose.position.y = 2;
	estimate.pose.position.z = 10;
	estimate.pose.orientation.w = 0.1;
	estimate.pose.orientation.x = 0.2;
	estimate.pose.orientation.y = 0.3;
	estimate.pose.orientation.z = 0.4;

	return estimate;
}

}  // namespace ndt_matching
