#ifndef NDT_MATCHING__NDT_LIB_HPP_
#define NDT_MATCHING__NDT_LIB_HPP_

#include "ndt_matching/visibility_control.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <memory>

namespace ndt_matching
{

class NdtLib
{
public:
  NdtLib();
  virtual ~NdtLib();

  bool is_map_registered() const {return map_registered;}
  void register_map(pcl::PointCloud<pcl::PointXYZ>& point_cloud);
  geometry_msgs::msg::PoseStamped execute(pcl::PointCloud<pcl::PointXYZ> &lidar_point_cloud,
		  geometry_msgs::msg::PoseStamped &pose_guess);
private:
  bool map_registered = false;
  pcl::PointCloud<pcl::PointXYZ> point_cloud_;
};

}  // namespace ndt_matching

#endif  // NDT_MATCHING__NDT_LIB_HPP_
