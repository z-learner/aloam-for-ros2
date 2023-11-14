#ifndef ROS2_WS_ALOAM_LIDAR_MAPPING
#define ROS2_WS_ALOAM_LIDAR_MAPPING

#include <ceres/ceres.h>
#include <math.h>

#include <Eigen/Dense>
#include <array>
#include <iostream>
#include <mutex>
#include <queue>
#include <string>
#include <thread>
#include <vector>
// clang-format off
#include "aloam-for-ros2/point_type.hpp"
// clang-format on
#include "aloam-for-ros2/tools/narray.hpp"
// pcl
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

// ros2
#include <tf2_ros/transform_broadcaster.h>
#include <nav_msgs/msg/detail/odometry__struct.hpp>
#include <nav_msgs/msg/detail/path__struct.hpp>
#include <atomic>
#include <condition_variable>
#include <mutex>
#include <queue>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include "aloam-for-ros2/lidar_factor.hpp"
#include "aloam-for-ros2/tools/tic_tok.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

class LidarMapping : public rclcpp::Node {
 public:
  using PointType = PointXYZTIRLL;  // need write self defined point type
  static constexpr size_t kMapWidth = 21;
  static constexpr size_t kMapHeight = 21;
  static constexpr size_t kMapDepth = 11;

  LidarMapping(const std::string& name_space);
  virtual ~LidarMapping() = default;

  void process_mapping();

 private:
  std::string name_space_{"lidar"};

  ndarray<3, std::shared_ptr<pcl::PointCloud<PointType>>> pcl_corner_map_{kMapWidth, kMapHeight, kMapDepth};
  ndarray<3, std::shared_ptr<pcl::PointCloud<PointType>>> pcl_surf_map_{kMapWidth, kMapHeight, kMapDepth};
  using Dim = ndarray<3, std::shared_ptr<pcl::PointCloud<PointType>>>::Dim;
  std::array<Dim, 125> map_vaild_index_{};
  std::array<Dim, 125> map_surround_index_{};

  pcl::KdTreeFLANN<PointType> corner_map_kdtree_;
  pcl::KdTreeFLANN<PointType> surf_map_kdtree_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr full_points_sub_ptr_{nullptr};
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr corner_points_sub_ptr_{nullptr};
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr surf_points_sub_ptr_{nullptr};
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr lidar_odom_to_init_sub_ptr_{nullptr};

  std::mutex mutex_;
  std::condition_variable cond_var_;
  std::queue<std::shared_ptr<sensor_msgs::msg::PointCloud2>> full_points_queue_{};
  std::queue<std::shared_ptr<sensor_msgs::msg::PointCloud2>> corner_points_queue_{};
  std::queue<std::shared_ptr<sensor_msgs::msg::PointCloud2>> surf_points_queue_{};
  std::queue<std::shared_ptr<nav_msgs::msg::Odometry>> lidar_odom_to_init_queue_{};

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_ptr_{nullptr};
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_surround_pub_ptr_{nullptr};
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr full_points_pub_ptr_{nullptr};
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr map_to_init_pub_ptr_{nullptr};
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr map_to_init_high_freq_pub_ptr_{nullptr};
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr map_path_pub_ptr_{nullptr};

  pcl::VoxelGrid<PointType> corner_downsize_filter_{};
  pcl::VoxelGrid<PointType> surf_downsize_filter_{};

  nav_msgs::msg::Path mapping_path_{};

  // std::shared_ptr<pcl::PointCloud<PointType>> last_corner_points_{new pcl::PointCloud<PointType>()};
  // std::shared_ptr<pcl::PointCloud<PointType>> last_surf_points_{new pcl::PointCloud<PointType>()};

  float corner_downsize_leaf_size{0.4};
  float surf_downsize_leaf_size{0.8};

  Eigen::Quaterniond q_wmap_wodom{1.0, 0.0, 0.0, 0.0};
  Eigen::Vector3d t_wmap_wodom{0.0, 0.0, 0.0};

  // to optimi
  std::array<double, 7> parameters_{0, 0, 0, 1, 0, 0, 0};

  std::unique_ptr<std::thread> process_map_thread_{nullptr};

  std::atomic_bool is_running_{false};

  int map_center_width_{10};
  int map_center_height_{10};
  int map_center_depth_{5};
};

#endif