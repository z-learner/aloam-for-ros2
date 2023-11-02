#ifndef ROS2_WS_ALOAM_LIDAR_ODOMETRY
#define ROS2_WS_ALOAM_LIDAR_ODOMETRY

#include <array>
#include <atomic>
#include <cmath>
#include <memory>
#include <nav_msgs/msg/detail/odometry__struct.hpp>
#include <nav_msgs/msg/detail/path__struct.hpp>
#include <string>
#include <thread>
#include <vector>
// clang-format off
#include "aloam-for-ros2/point_type.hpp"
// clang-format on

#include "sensor_msgs/msg/point_cloud2.hpp"

// pcl
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

// ros2
#include "rclcpp/rclcpp.hpp"

#include <queue>
#include <atomic>
#include <mutex>
#include <condition_variable>

class LidarOdometry : public rclcpp::Node {
 public:
  using PointType = PointXYZTIRLL;  // need write self defined point type

  static constexpr size_t kMaxCloudSize = 200000;
  static constexpr double kDistanceThreshold = 25;

  LidarOdometry(const std::string& node_namespace);
  virtual ~LidarOdometry() = default;

 private:
  std::queue<std::shared_ptr<sensor_msgs::msg::PointCloud2>> corner_sharp_points_cloud_queue_{};
  std::queue<std::shared_ptr<sensor_msgs::msg::PointCloud2>> corner_less_sharp_points_cloud_queue_{};
  std::queue<std::shared_ptr<sensor_msgs::msg::PointCloud2>> surf_flat_points_cloud_queue_{};
  std::queue<std::shared_ptr<sensor_msgs::msg::PointCloud2>> surf_less_flat_points_cloud_queue_{};

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr corner_sharp_points_sub_ptr_{nullptr};
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr corner_less_sharp_points_sub_ptr_{nullptr};
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr surf_flat_points_sub_ptr_{nullptr};
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr surf_less_flat_points_sub_ptr_{nullptr};

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr corner_points_cloud_last_pub_ptr_{nullptr};
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr surf_points_cloud_last_pub_ptr_{nullptr};
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr lidar_odom_to_init_pub_ptr_{nullptr};
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr lidar_odom_path_pub_ptr_{nullptr};

  std::string node_namespace_{};

  std::atomic_bool is_running_{false};
  std::mutex mutex_;
  std::condition_variable cond_var_;
  std::unique_ptr<std::thread> process_odom_thread_ptr_{nullptr};

  std::array<double, 4> quaternion_{0, 0, 0, 1};
  std::array<double, 4> translation_{0, 0, 0};

  Eigen::Quaterniond q_w_curr_{1.0, 0.0, 0.0, 0.0};
  Eigen::Vector3d t_w_curr_{0.0, 0.0, 0.0};

  // only save less sharp
  pcl::KdTreeFLANN<PointType> last_corner_kd_tree_{};
  // only save less flat
  pcl::KdTreeFLANN<PointType> last_surf_kd_tree_{};

  std::shared_ptr<pcl::PointCloud<PointType>> last_corner_point_cloud_{std::make_shared<pcl::PointCloud<PointType>>()};
  std::shared_ptr<pcl::PointCloud<PointType>> last_surf_point_cloud_{std::make_shared<pcl::PointCloud<PointType>>()};

  nav_msgs::msg::Path lidar_path_{};

 private:
  void ProcessOdom();

  void UndistortLidarPointsToLastFrame(const PointType& point_origin, PointType& point, const Eigen::Map<Eigen::Quaterniond>& q,
                                       const Eigen::Map<Eigen::Vector3d>& t);
};

#endif