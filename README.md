## Description
Implementation of 3D-NDT algorithm running on ROS2 Dashing. Running full program requires ROS1 distribution and ROS2 Dashing. It passes messages published from ROS1 bag file and pcd map through ROS1/ROS2 bridge nad runs ROS2 ndt_node. Here it is assumed that ROS1 melodic and ROS2 dashing distributions are installed

## How to compile
Clone the repository into your ros2 workspace and build using colcon as below

```sh
source /opt/ros/dashing/setup.bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/woookey/ndt_matching.git
cd ~/ros2_ws
colcon build
```

## How to run
* Run roscore in terminal 1

```sh
source /opt/ros/melodic/setup.bash
roscore
```

* Run rosbridge in terminal 2

```sh
source /opt/ros/dashing/setup.bash
ros2 run ros1_bridge dynamic_bridge --bridge-all-topics
```

* Run ndt_node in terminal 3

```sh
source /opt/ros/dashing/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run ndt_matching ndt_node
```

* Publish map.pcd in terminal 4

```sh
source /opt/ros/melodic/setup.bash
rosrun pcl_ros pcd_to_pointcloud <path_to_map.pcd>/map.pcd
```

* Publish lidar_data.bag in terminal 5

```sh
source /opt/ros/melodic/setup.bash
rosbag play <path_to_lidar_data.bag>/lidar_data.bag
```

* Subscribe to /estimated_pose in ROS1 

```sh
source /opt/ros/melodic/setup.bash
rostopic echo /estimated_pose
```

* Subscribe to /estimated_pose in ROS2 

```sh
source /opt/ros/dashing/setup.bash
ros2 topic echo /estimated_pose
```

