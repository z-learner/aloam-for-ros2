#include "aloam-for-ros2/scan_registration.hpp"

#include <fmt/format.h>

#include <chrono>

#include "aloam-for-ros2/tools/tic_tok.hpp"

using std::placeholders::_1;

using namespace std::chrono_literals;

void ScanRegistration::RemoveClosePoint(pcl::PointCloud<PointType>::Ptr points_cloud_ptr, float min_range) {
  size_t count = 0;
  size_t total_points_cloud_size = points_cloud_ptr->size();
  auto min_range_pow_2 = min_range * min_range;
  for (size_t index = 0; index < points_cloud_ptr->size(); ++index) {
    float range_pow_2 = std::pow(points_cloud_ptr->data()[index].x, 2) + std::pow(points_cloud_ptr->data()[index].y, 2) +
                        std::pow(points_cloud_ptr->data()[index].z, 2);

    if (range_pow_2 < min_range_pow_2) {
      continue;
    }

    points_cloud_ptr->data()[count++] = points_cloud_ptr->data()[index];
  }

  // pcl header.stamp is us
  if (count * 2 < total_points_cloud_size) {
    RCLCPP_WARN(this->get_logger(), "this frame in %.3f s have too many close points, %llu -> %llu", points_cloud_ptr->header.stamp / 1e6,
                total_points_cloud_size, count);
  }

  if (count < total_points_cloud_size) {
    points_cloud_ptr->resize(count);
  }
  points_cloud_ptr->height = 1;
  points_cloud_ptr->width = static_cast<uint32_t>(count);
  points_cloud_ptr->is_dense = true;
}

ScanRegistration::ScanRegistration(const std::string& node_namespace, bool is_need_calculate_scan_line, size_t scan_line_number)
    : Node("scan_registration"),
      node_namespace_(node_namespace),
      is_need_calculate_scan_line_(is_need_calculate_scan_line),
      scan_line_number_(scan_line_number) {
  points_cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(fmt::format("/{}/points", node_namespace), 10,
                                                                               std::bind(&ScanRegistration::subscription_call_back, this, _1));
  corner_sharp_points_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(fmt::format("/{}/sharp_points", node_namespace), 10);
  corner_less_sharp_points_cloud_pub_ =
      this->create_publisher<sensor_msgs::msg::PointCloud2>(fmt::format("/{}/less_sharp_points", node_namespace), 10);
  surf_flat_points_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(fmt::format("/{}/flat_points", node_namespace), 10);
  surf_less_flat_points_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(fmt::format("/{}/less_flat_points", node_namespace), 10);
  order_points_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(fmt::format("/{}/order_points", node_namespace), 10);

  line_points_cloud_pubs_.resize(scan_line_number);
  for (size_t index = 0; index < scan_line_number; ++index) {
    line_points_cloud_pubs_[index] =
        this->create_publisher<sensor_msgs::msg::PointCloud2>(fmt::format("/{}/line{}_points", node_namespace, index), 10);
  }
}

void ScanRegistration::subscription_call_back(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  if (!system_init_done_) {
    ++system_skip_count_;
    RCLCPP_INFO(this->get_logger(), "system not init, skip %dth frame ing", system_skip_count_);
    if (system_skip_count_ >= system_init_count_) {
      system_init_done_ = true;
      RCLCPP_INFO(this->get_logger(), " system init done ");
    }
    return;
  }

  // whole cost
  TicToc whole_tic_tok;

  auto pcl_points_cloud_ptr = std::make_shared<pcl::PointCloud<PointType>>();
  pcl::fromROSMsg(*msg, *pcl_points_cloud_ptr);
  RemoveClosePoint(pcl_points_cloud_ptr, 0.5f);

  size_t cloud_points_size = pcl_points_cloud_ptr->size();

  assert(cloud_points_size < kMaxCloudSize);

  // Divide points from different lines into different point clouds
  std::vector<pcl::PointCloud<PointType>> scan_line_points_cloud(scan_line_number_);

  for (size_t index = 0; index < cloud_points_size; ++index) {
    auto point = pcl_points_cloud_ptr->data()[index];
    auto scan_line_id = point.ring;
    assert(scan_line_id < scan_line_number_);
    scan_line_points_cloud[scan_line_id].push_back(point);
  }

  // Create a point cloud in the order of line IDs and generate an index
  // filter start and end five points
  std::vector<std::pair<size_t, size_t>> scan_line_start_end_index(scan_line_number_, {0, 0});
  auto points_cloud_order = std::make_shared<pcl::PointCloud<PointType>>();

  for (size_t index = 0; index < scan_line_number_; ++index) {
    scan_line_start_end_index[index].first = points_cloud_order->size() + 5;
    *points_cloud_order += scan_line_points_cloud[index];

    size_t point_cloud_order_size = points_cloud_order->size();
    if (point_cloud_order_size > (scan_line_start_end_index[index].first + 5)) {
      scan_line_start_end_index[index].second = point_cloud_order_size - 5;
    } else {
      scan_line_start_end_index[index].second = scan_line_start_end_index[index].first;
    }
  }

  // Preprocessing Time for Calculating Curvature
  TicToc calculate_curvature_tic_tok;

  for (size_t index = 5; index < cloud_points_size - 5; ++index) {
    float diffX = points_cloud_order->points[index - 5].x + points_cloud_order->points[index - 4].x + points_cloud_order->points[index - 3].x +
                  points_cloud_order->points[index - 2].x + points_cloud_order->points[index - 1].x - 10 * points_cloud_order->points[index].x +
                  points_cloud_order->points[index + 1].x + points_cloud_order->points[index + 2].x + points_cloud_order->points[index + 3].x +
                  points_cloud_order->points[index + 4].x + points_cloud_order->points[index + 5].x;
    float diffY = points_cloud_order->points[index - 5].y + points_cloud_order->points[index - 4].y + points_cloud_order->points[index - 3].y +
                  points_cloud_order->points[index - 2].y + points_cloud_order->points[index - 1].y - 10 * points_cloud_order->points[index].y +
                  points_cloud_order->points[index + 1].y + points_cloud_order->points[index + 2].y + points_cloud_order->points[index + 3].y +
                  points_cloud_order->points[index + 4].y + points_cloud_order->points[index + 5].y;
    float diffZ = points_cloud_order->points[index - 5].z + points_cloud_order->points[index - 4].z + points_cloud_order->points[index - 3].z +
                  points_cloud_order->points[index - 2].z + points_cloud_order->points[index - 1].z - 10 * points_cloud_order->points[index].z +
                  points_cloud_order->points[index + 1].z + points_cloud_order->points[index + 2].z + points_cloud_order->points[index + 3].z +
                  points_cloud_order->points[index + 4].z + points_cloud_order->points[index + 5].z;

    cloud_points_curvature_[index] = diffX * diffX + diffY * diffY + diffZ * diffZ;
    cloud_points_sort_index_[index] = index;
    cloud_points_neighbor_picked_[index] = false;
    cloud_points_label_[index] = PointLabel::Normal;
  }

  auto corner_points_cloud_sharp = std::make_shared<pcl::PointCloud<PointType>>();
  auto corner_points_cloud_less_sharp = std::make_shared<pcl::PointCloud<PointType>>();
  auto surf_points_cloud_flat = std::make_shared<pcl::PointCloud<PointType>>();
  auto surf_points_cloud_less_flat = std::make_shared<pcl::PointCloud<PointType>>();

  // Compute feature points based on lines
  for (size_t line_id = 0; line_id < scan_line_number_; ++line_id) {
    auto surf_points_cloud_less_flat_temp_for_line = std::make_shared<pcl::PointCloud<PointType>>();

    if (scan_line_start_end_index[line_id].second - scan_line_start_end_index[line_id].first < 6) {
      continue;
    }

    // divided into six equal parts
    size_t part_all_points_size = scan_line_start_end_index[line_id].second - scan_line_start_end_index[line_id].first;
    for (size_t part_id = 0; part_id < kLinePartsSize; ++part_id) {
      int start_point_id = scan_line_start_end_index[line_id].first + part_id * (part_all_points_size / kLinePartsSize);
      int end_point_id = scan_line_start_end_index[line_id].first + (part_id + 1) * (part_all_points_size / kLinePartsSize);

      // RCLCPP_INFO(this->get_logger(), "---------------------------");
      // RCLCPP_INFO(this->get_logger(), " line_id              %llu ", line_id);
      // RCLCPP_INFO(this->get_logger(), " part_id              %llu ", part_id);
      // RCLCPP_INFO(this->get_logger(), " part_all_points_size %llu ", part_all_points_size);
      // RCLCPP_INFO(this->get_logger(), " start_point_id       %llu ", start_point_id);
      // RCLCPP_INFO(this->get_logger(), " end_point_id         %llu ", end_point_id);
      // RCLCPP_INFO(this->get_logger(), "---------------------------");

      // arrange in ascending order
      // TODO std::execution::par
      std::sort(cloud_points_sort_index_.data() + start_point_id, cloud_points_sort_index_.data() + end_point_id,
                [&](size_t left, size_t right) { return cloud_points_curvature_[left] < cloud_points_curvature_[right]; });

      // Descending order to find the maximum value, to locate edge points
      size_t sharp_packed_number = 0;
      for (size_t index = end_point_id - 1; index >= start_point_id; --index) {
        size_t point_index = cloud_points_sort_index_[index];
        // sharp
        if (!cloud_points_neighbor_picked_[index] && cloud_points_curvature_[index] > kSharpCurvatureThreshold) {
          ++sharp_packed_number;
          if (sharp_packed_number <= 2) {
            cloud_points_label_[point_index] = PointLabel::Shrap;
            corner_points_cloud_sharp->push_back(points_cloud_order->data()[point_index]);
            corner_points_cloud_less_sharp->push_back(points_cloud_order->data()[point_index]);
          } else if (sharp_packed_number <= 20) {
            cloud_points_label_[point_index] = PointLabel::LessSharp;
            corner_points_cloud_less_sharp->push_back(points_cloud_order->data()[point_index]);
          } else {
            // too much
            break;
          }

          cloud_points_neighbor_picked_[point_index] = true;
          // Filter out points with high curvature that have 5 nearby points in both directions removed to prevent feature point clustering,
          //     ensuring an even distribution of feature points in each direction as much as possible.

          for (int l = 1; l <= 5; ++l) {
            float diffX = points_cloud_order->points[point_index + l].x - points_cloud_order->points[point_index + l - 1].x;
            float diffY = points_cloud_order->points[point_index + l].y - points_cloud_order->points[point_index + l - 1].y;
            float diffZ = points_cloud_order->points[point_index + l].z - points_cloud_order->points[point_index + l - 1].z;

            // It should be a decision to skip the simple computation as it's unstable.
            if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05) {
              break;
            }
            cloud_points_neighbor_picked_[point_index + l] = true;
          }

          for (int l = -1; l <= -5; --l) {
            float diffX = points_cloud_order->points[point_index + l].x - points_cloud_order->points[point_index + l + 1].x;
            float diffY = points_cloud_order->points[point_index + l].y - points_cloud_order->points[point_index + l + 1].y;
            float diffZ = points_cloud_order->points[point_index + l].z - points_cloud_order->points[point_index + l + 1].z;

            // It should be a decision to skip the simple computation as it's unstable.
            if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05) {
              break;
            }
            cloud_points_neighbor_picked_[point_index + l] = true;
          }
        }
      }

      //  Ascending order to find the minimum value, to locate edge points
      size_t flat_packed_number = 0;
      for (size_t index = start_point_id; index < end_point_id; ++index) {
        size_t point_index = cloud_points_sort_index_[index];
        if (!cloud_points_neighbor_picked_[index] && cloud_points_curvature_[index] < kSharpCurvatureThreshold) {
          ++flat_packed_number;
          // only find 4
          if (flat_packed_number >= 4) {
            break;
          }
          cloud_points_label_[point_index] = PointLabel::Flat;
          surf_points_cloud_flat->push_back(points_cloud_order->data()[point_index]);
          cloud_points_neighbor_picked_[point_index] = true;
          for (int l = 1; l <= 5; ++l) {
            float diffX = points_cloud_order->points[point_index + l].x - points_cloud_order->points[point_index + l - 1].x;
            float diffY = points_cloud_order->points[point_index + l].y - points_cloud_order->points[point_index + l - 1].y;
            float diffZ = points_cloud_order->points[point_index + l].z - points_cloud_order->points[point_index + l - 1].z;

            // It should be a decision to skip the simple computation as it's unstable.
            if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05) {
              break;
            }
            cloud_points_neighbor_picked_[point_index + l] = true;
          }

          for (int l = -1; l <= -5; --l) {
            float diffX = points_cloud_order->points[point_index + l].x - points_cloud_order->points[point_index + l + 1].x;
            float diffY = points_cloud_order->points[point_index + l].y - points_cloud_order->points[point_index + l + 1].y;
            float diffZ = points_cloud_order->points[point_index + l].z - points_cloud_order->points[point_index + l + 1].z;

            // It should be a decision to skip the simple computation as it's unstable.
            if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05) {
              break;
            }
            cloud_points_neighbor_picked_[point_index + l] = true;
          }
        }
      }
      for (size_t index = start_point_id; index < end_point_id; ++index) {
        if (cloud_points_label_[index] == PointLabel::Normal || cloud_points_label_[index] == PointLabel::Flat) {
          surf_points_cloud_less_flat_temp_for_line->push_back(points_cloud_order->data()[index]);
        }
      }
    }
    // Since there are the most 'less flat' points,
    // perform voxel grid filtering on the 'less flat' points in each segment

    pcl::PointCloud<PointType> surf_points_less_flat_DS;
    pcl::VoxelGrid<PointType> downSize_filter;
    downSize_filter.setInputCloud(surf_points_cloud_less_flat_temp_for_line);
    downSize_filter.setLeafSize(0.2, 0.2, 0.2);
    downSize_filter.filter(surf_points_less_flat_DS);

    *surf_points_cloud_less_flat += surf_points_less_flat_DS;
  }
  RCLCPP_INFO(this->get_logger(), "system calculate curvature and find feature points cost  %.3f ms", calculate_curvature_tic_tok.toc() / 1e6);

  sensor_msgs::msg::PointCloud2 ros2_tmp_msg;
  pcl::toROSMsg(*corner_points_cloud_sharp, ros2_tmp_msg);
  ros2_tmp_msg.header.frame_id = "lidar";
  ros2_tmp_msg.header.stamp.sec = pcl_points_cloud_ptr->header.stamp / 1000000;
  ros2_tmp_msg.header.stamp.nanosec = pcl_points_cloud_ptr->header.stamp % 1000000 * 1000;
  // corner_sharp_points_cloud_pub_
  corner_sharp_points_cloud_pub_->publish(ros2_tmp_msg);
  // corner_less_sharp_points_cloud_pub_
  pcl::toROSMsg(*corner_points_cloud_less_sharp, ros2_tmp_msg);
  ros2_tmp_msg.header.frame_id = "lidar";
  ros2_tmp_msg.header.stamp.sec = pcl_points_cloud_ptr->header.stamp / 1000000;
  ros2_tmp_msg.header.stamp.nanosec = pcl_points_cloud_ptr->header.stamp % 1000000 * 1000;
  corner_less_sharp_points_cloud_pub_->publish(ros2_tmp_msg);
  // surf_flat_points_cloud_pub_
  pcl::toROSMsg(*surf_points_cloud_flat, ros2_tmp_msg);
  ros2_tmp_msg.header.frame_id = "lidar";
  ros2_tmp_msg.header.stamp.sec = pcl_points_cloud_ptr->header.stamp / 1000000;
  ros2_tmp_msg.header.stamp.nanosec = pcl_points_cloud_ptr->header.stamp % 1000000 * 1000;
  surf_flat_points_cloud_pub_->publish(ros2_tmp_msg);
  // surf_less_flat_points_cloud_pub_
  pcl::toROSMsg(*surf_points_cloud_less_flat, ros2_tmp_msg);
  ros2_tmp_msg.header.frame_id = "lidar";
  ros2_tmp_msg.header.stamp.sec = pcl_points_cloud_ptr->header.stamp / 1000000;
  ros2_tmp_msg.header.stamp.nanosec = pcl_points_cloud_ptr->header.stamp % 1000000 * 1000;
  surf_less_flat_points_cloud_pub_->publish(ros2_tmp_msg);
  // order_points_cloud_pub_
  pcl::toROSMsg(*points_cloud_order, ros2_tmp_msg);
  ros2_tmp_msg.header.frame_id = "lidar";
  ros2_tmp_msg.header.stamp.sec = pcl_points_cloud_ptr->header.stamp / 1000000;
  ros2_tmp_msg.header.stamp.nanosec = pcl_points_cloud_ptr->header.stamp % 1000000 * 1000;
  order_points_cloud_pub_->publish(ros2_tmp_msg);

  RCLCPP_INFO(this->get_logger(), "system calculate one frame and push cost %.3f ms", whole_tic_tok.toc() / 1e6);
  RCLCPP_INFO(this->get_logger(), "-----------------------info-----------------------");
  RCLCPP_INFO(this->get_logger(), "corner_points_cloud_sharp:       %llu ", corner_points_cloud_sharp->size());
  RCLCPP_INFO(this->get_logger(), "corner_points_cloud_less_sharp:  %llu ", corner_points_cloud_less_sharp->size());
  RCLCPP_INFO(this->get_logger(), "surf_points_cloud_flat:          %llu ", surf_points_cloud_flat->size());
  RCLCPP_INFO(this->get_logger(), "surf_points_cloud_less_flat:     %llu ", surf_points_cloud_less_flat->size());
  RCLCPP_INFO(this->get_logger(), "points_cloud_order:              %llu ", points_cloud_order->size());
  RCLCPP_INFO(this->get_logger(), "--------------------------------------------------");
}