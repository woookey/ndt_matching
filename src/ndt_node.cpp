#include <cstdio>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rcutils/cmdline_parser.h"

#include "std_msgs/msg/string.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>

/*void print_usage()
{
  printf("Usage for listener app:\n");
  printf("listener [-t topic_name] [-h]\n");
  printf("options:\n");
  printf("-h : Print this help function.\n");
  printf("-t topic_name : Specify the topic on which to subscribe. Defaults to chatter.\n");
}*/

// Create a Listener class that subclasses the generic rclcpp::Node base class.
// The main function below will instantiate the class as a ROS node.
class Listener : public rclcpp::Node
{
public:
  explicit Listener(const std::string & topic_name = "filtered_points",
		  const std::string & topic_name2 = "cloud_pcd")
  : Node("listener")
  {
    // Create a callback function for when messages are received.
    // Variations of this function also exist using, for example UniquePtr for zero-copy transport.
    auto callback =
      [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) -> void
      {
    	RCLCPP_INFO(this->get_logger(), "I heard filtered point: [%s]", msg->header.frame_id.c_str());
        //TODO:
        // here you call NdtLib function and pass in the msg as input
        // return a pose message and publish it as https://github.com/ros2/common_interfaces/blob/master/geometry_msgs/msg/PoseStamped.msg

    	// TODO: Create a return PoseStampled message to publish
    	geometry_msgs::msg::PoseStamped estimated_pose;
    	estimated_pose.header.frame_id = "estimated_pose";
    	estimated_pose.pose.position.x = 1;
    	estimated_pose.pose.position.y = 2;
    	estimated_pose.pose.position.z = 3;
    	estimated_pose.pose.orientation.w = 0;
    	estimated_pose.pose.orientation.x = 0.5;
    	estimated_pose.pose.orientation.y = 0.3;
    	estimated_pose.pose.orientation.z = 0.2;
    	//estimated_pose.header.stamp = msg->
    	pub_->publish(estimated_pose);
      };

    auto callback2 =
    [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) -> void
      {
        RCLCPP_INFO(this->get_logger(), "I heard point cloud map: [%s]", msg->header.frame_id.c_str());
        //TODO: here you get your map point cloud (one time only)
      };

    RCLCPP_INFO(this->get_logger(), "Started the ndt_matching node...");
    // Create a subscription to the topic which can be matched with one or more compatible ROS
    // publishers.
    // Note that not all publishers on the same topic with the same type will be compatible:
    // they must have compatible Quality of Service policies.
    sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(topic_name, callback);
    sub2_ = create_subscription<sensor_msgs::msg::PointCloud2>(topic_name2, callback2);
    // TODO: create a pose publisher, see for reference
    // https://github.com/ros2/demos/blob/master/demo_nodes_cpp/src/topics/talker.cpp
    pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("estimated_pose");
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub2_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_;

  // Initial pose and vector of estimated poses
  // TODO: Move to ndt_matching class encapsulation
  std::unique_ptr<geometry_msgs::msg::Pose> initial_pose_;
  std::vector<geometry_msgs::msg::PoseStamped> estimated_poses_v_;
};

int main(int argc, char * argv[])
{
  // Force flush of the stdout buffer.
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  /*if (rcutils_cli_option_exist(argv, argv + argc, "-h")) {
    print_usage();
    return 0;
  }*/

  // Initialize any global resources needed by the middleware and the client library.
  // You must call this before using any other part of the ROS system.
  // This should be called once per process.
  rclcpp::init(argc, argv);

  // Parse the command line options.
  /*auto topic = std::string("points_raw");
  char * cli_option = rcutils_cli_get_option(argv, argv + argc, "-t");
  if (nullptr != cli_option) {
    topic = std::string(cli_option);
  }*/

  // Create a node.
  //auto node = std::make_shared<Listener>(topic);
  auto node = std::make_shared<Listener>();
  // spin will block until work comes in, execute work as it becomes available, and keep blocking.
  // It will only be interrupted by Ctrl-C.
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
