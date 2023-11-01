#ifndef ROS2_WS_ALOAM_SCAN_REGISTRATION
#define ROS2_WS_ALOAM_SCAN_REGISTRATION

/**
 * @file scan_registration.hpp
 * @author Zplearner@163.com
 * @brief
 * @version 0.1
 * @date 2023-10-17
 *
 * @copyright Copyright (c) 2023
 *
 */

#include <array>
#include <atomic>
#include <cmath>
#include <memory>
#include <string>
#include <thread>
#include <vector>

// clang-format off
#include "aloam-for-ros2/point_type.hpp"
// clang-format on

#include <sensor_msgs/msg/point_cloud2.hpp>

// pcl
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

// ros2
#include "rclcpp/rclcpp.hpp"

class ScanRegistration : public rclcpp::Node {
 public:
  static constexpr size_t kMaxCloudSize = 200000;
  static constexpr size_t kLinePartsSize = 6;
  static constexpr float kSharpCurvatureThreshold = 0.1;

  using PointType = PointXYZTIRLL;  // need write self defined point type

  enum class PointLabel { Normal = 0, Shrap, LessSharp, Flat, LessFlat };

  ScanRegistration(const std::string& node_namespace, bool is_need_calculate_scan_line = false, size_t scan_line_number = 64);
  ~ScanRegistration() = default;

 private:
  // Publisher
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr corner_sharp_points_cloud_pub_{nullptr};
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr corner_less_sharp_points_cloud_pub_{nullptr};
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr surf_flat_points_cloud_pub_{nullptr};
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr surf_less_flat_points_cloud_pub_{nullptr};
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr order_points_cloud_pub_{nullptr};

  std::vector<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr> line_points_cloud_pubs_{};  // ros line points cloud

  // Subscription
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr points_cloud_sub_{nullptr};

  bool is_need_calculate_scan_line_{false};
  size_t scan_line_number_{64};

  // system init
  bool system_init_done_{false};
  size_t system_skip_count_{0};
  size_t system_init_count_{5};

  std::string node_namespace_{"lidar"};

 private:
  void subscription_call_back(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void RemoveClosePoint(pcl::PointCloud<PointType>::Ptr, float);

 private:
  std::array<float, kMaxCloudSize> cloud_points_curvature_{0};
  std::array<size_t, kMaxCloudSize> cloud_points_sort_index_{0};
  std::array<bool, kMaxCloudSize> cloud_points_neighbor_picked_{0};
  std::array<PointLabel, kMaxCloudSize> cloud_points_label_{PointLabel::Normal};
};

#endif