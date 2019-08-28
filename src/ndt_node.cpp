#include <cstdio>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include "ndt_matching/ndt_lib.hpp"

#include <pcl_conversions/pcl_conversions.h>

// Create a Listener class that subclasses the generic rclcpp::Node base class.
// The main function below will instantiate the class as a ROS node.
class Listener : public rclcpp::Node
{
public:
  explicit Listener(const std::string& topic_cloud_points_name = "filtered_points",
		  const std::string& topic_cloud_map_name = "cloud_pcd",
		  const std::string& topic_pose_name = "current_pose",
		  const std::string& topic_estimated_pose_name = "estimated_pose")
  : Node("listener")
  {
    // Create a callback function for when messages are received.
    // Variations of this function also exist using, for example UniquePtr for zero-copy transport.
    auto callback_cloud_points =
      [this](const sensor_msgs::msg::PointCloud2::SharedPtr filtered_points_msg) -> void
      {
    	RCLCPP_INFO(this->get_logger(), "I heard filtered point: [%s]",
    			filtered_points_msg->header.frame_id.c_str());
        //TODO:
        // here you call NdtLib function and pass in the msg as input
        // return a pose message and publish it as https://github.com/ros2/common_interfaces/blob/master/geometry_msgs/msg/PoseStamped.msg

    	// Add new filtered_points as XYZ Point Cloud
    	pcl::PointCloud<pcl::PointXYZ> xyz_filtered_points;
    	pcl::fromROSMsg(*filtered_points_msg, xyz_filtered_points);
    	filtered_point_cloud_l_.push_back(xyz_filtered_points);

    	/**
    	 * Fetch firstly received /current_pose and /filtered_points data
    	 * and run NDT matching unless the pcd_cloud map was not received
    	 *
    	 * Note: There is still a problem of executing the remaining data
    	 * since callbacks are executed reactively. It could be solved by adding
    	 * a timer which triggers the program to execute NDT on remaining datapoints
    	 * once there is no inputs for a specified period of time
    	 */
    	if (!NDTMatching->is_map_registered())
    	{
    		RCLCPP_WARN(this->get_logger(), "No map received");
    	}
    	else if (!filtered_point_cloud_l_.empty() && !initial_pose_l_.empty())
    	{
    		pcl::PointCloud<pcl::PointXYZ> lidar_data =
    				(pcl::PointCloud<pcl::PointXYZ>)filtered_point_cloud_l_.front();
    		filtered_point_cloud_l_.pop_front();

    		geometry_msgs::msg::PoseStamped pose_guess =
    				(geometry_msgs::msg::PoseStamped)initial_pose_l_.front();
    		initial_pose_l_.pop_front();

    		// Execute NDT algorithm
    		geometry_msgs::msg::PoseStamped pose_estimate =
    				NDTMatching->execute(lidar_data, pose_guess);
    		pub_estimated_pose_->publish(pose_estimate);
    	}
      };

    auto callback_cloud_map =
    [this](const sensor_msgs::msg::PointCloud2::SharedPtr cloud_map_msg) -> void
      {
        RCLCPP_INFO(this->get_logger(), "I heard point cloud map: [%s]", cloud_map_msg->header.frame_id.c_str());

        // Move XYZ Point Cloud from the message
        pcl::fromROSMsg(*cloud_map_msg, xyz_cloud_map_);
        NDTMatching->register_map(xyz_cloud_map_);

      };

    auto callback_pose =
	[this](const geometry_msgs::msg::PoseStamped::SharedPtr pose_msg) -> void
	{
    	RCLCPP_INFO(this->get_logger(), "I heard current pose: [%s]", pose_msg->header.frame_id.c_str());
    	initial_pose_l_.push_back(*pose_msg);
	};

    // Initialise NDTLib Instance
	NDTMatching = std::make_unique<ndt_matching::NdtLib>();
	RCLCPP_INFO(this->get_logger(), "Started the 3D-NDT node...");

    // Create subscribers
    sub_cloud_points_ = create_subscription<sensor_msgs::msg::PointCloud2>(topic_cloud_points_name, callback_cloud_points);
    sub_cloud_map_ = create_subscription<sensor_msgs::msg::PointCloud2>(topic_cloud_map_name, callback_cloud_map);
    sub_initial_pose_ = create_subscription<geometry_msgs::msg::PoseStamped>(topic_pose_name, callback_pose);

    // Create estimated pose publisher
    pub_estimated_pose_ = create_publisher<geometry_msgs::msg::PoseStamped>(topic_estimated_pose_name);
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_cloud_points_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_cloud_map_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_initial_pose_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_estimated_pose_;

  // Data
  std::list <pcl::PointCloud<pcl::PointXYZ> > filtered_point_cloud_l_;
  std::list <geometry_msgs::msg::PoseStamped> initial_pose_l_;
  //std::vector<geometry_msgs::msg::PoseStamped> estimated_poses_v_;
  pcl::PointCloud<pcl::PointXYZ> xyz_cloud_map_;

  // NDT Matching
  std::unique_ptr<ndt_matching::NdtLib> NDTMatching;

  //void publish_estimated_pose(void);
};

int main(int argc, char * argv[])
{
  // Force flush of the stdout buffer.
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  // Initialize any global resources needed by the middleware and the client library.
  // You must call this before using any other part of the ROS system.
  // This should be called once per process.
  rclcpp::init(argc, argv);

  // Create a node.
  auto node = std::make_shared<Listener>();
  // spin will block until work comes in, execute work as it becomes available, and keep blocking.
  // It will only be interrupted by Ctrl-C.
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
