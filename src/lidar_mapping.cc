#include "aloam-for-ros2/lidar_mapping.hpp"

#include <fmt/format.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <geometry_msgs/msg/transform_stamped.hpp>

#include "aloam-for-ros2/lidar_factor.hpp"
#include "ceres/ceres.h"
#include "ceres/rotation.h"
using namespace std::chrono_literals;

template <typename First, typename... Rest>
bool AreEqual(First first, Rest... rest) {
  return (... && (first == rest));
}

template <typename T>
T MaxOf(T t) {
  return t;
}

// 递归模板：首先比较第一个参数和其余参数的最大值
template <typename T, typename... Rest>
T MaxOf(T first, Rest... rest) {
  return std::max(first, MaxOf(rest...));
}

LidarMapping::LidarMapping(const std::string& name_space) : rclcpp::Node("lidar_mapping"), name_space_(name_space) {
  map_to_init_high_freq_pub_ptr_ = this->create_publisher<nav_msgs::msg::Odometry>(fmt::format("/{}/mapped_to_init_high_frec", name_space), 10);

  map_pub_ptr_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(fmt::format("/{}/map_points", name_space), 10);

  full_points_pub_ptr_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(fmt::format("/{}/full_points", name_space), 10);

  map_surround_pub_ptr_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(fmt::format("/{}/surround_map_points", name_space), 10);

  map_to_init_pub_ptr_ = this->create_publisher<nav_msgs::msg::Odometry>(fmt::format("/{}/mapped_to_init", name_space), 10);

  map_path_pub_ptr_ = this->create_publisher<nav_msgs::msg::Path>(fmt::format("/{}/map_path", name_space), 10);

  full_points_sub_ptr_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(fmt::format("/{}/order_points", name_space), 10,
                                                                                  [this](sensor_msgs::msg::PointCloud2::SharedPtr msg) {
                                                                                    {
                                                                                      std::lock_guard<std::mutex> lk(mutex_);
                                                                                      full_points_queue_.push(msg);
                                                                                    }
                                                                                    cond_var_.notify_one();
                                                                                  });
  corner_points_sub_ptr_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(fmt::format("/{}/corner_points_low_freq", name_space), 10,
                                                                                    [this](sensor_msgs::msg::PointCloud2::SharedPtr msg) {
                                                                                      {
                                                                                        std::lock_guard<std::mutex> lk(mutex_);
                                                                                        corner_points_queue_.push(msg);
                                                                                      }
                                                                                      cond_var_.notify_one();
                                                                                    });

  surf_points_sub_ptr_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(fmt::format("/{}/surf_points_low_freq", name_space), 10,
                                                                                  [this](sensor_msgs::msg::PointCloud2::SharedPtr msg) {
                                                                                    {
                                                                                      std::lock_guard<std::mutex> lk(mutex_);
                                                                                      surf_points_queue_.push(msg);
                                                                                    }
                                                                                    cond_var_.notify_one();
                                                                                  });
  lidar_odom_to_init_sub_ptr_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      fmt::format("/{}/surf_points_low_freq", name_space), 10, [this](nav_msgs::msg::Odometry::SharedPtr msg) {
        {
          nav_msgs::msg::Odometry::SharedPtr new_msg = std::make_shared<nav_msgs::msg::Odometry>();
          *new_msg = *msg;
          std::lock_guard<std::mutex> lk(mutex_);
          lidar_odom_to_init_queue_.push(new_msg);
        }
        cond_var_.notify_one();

        // get high freq odom
        Eigen::Quaterniond q_wodom_curr;
        Eigen::Vector3d t_wodom_curr;
        q_wodom_curr.x() = msg->pose.pose.orientation.x;
        q_wodom_curr.y() = msg->pose.pose.orientation.y;
        q_wodom_curr.z() = msg->pose.pose.orientation.z;
        q_wodom_curr.w() = msg->pose.pose.orientation.w;
        t_wodom_curr.x() = msg->pose.pose.position.x;
        t_wodom_curr.y() = msg->pose.pose.position.y;
        t_wodom_curr.z() = msg->pose.pose.position.z;
        // (map to odom) x ( odom to current )
        Eigen::Quaterniond q_w_curr = q_wmap_wodom * q_wodom_curr;
        Eigen::Vector3d t_w_curr = q_wmap_wodom * t_wodom_curr + t_wmap_wodom;

        nav_msgs::msg::Odometry odom_after_mapped;
        odom_after_mapped.header.frame_id = "lidar";
        odom_after_mapped.child_frame_id = "mapped";
        odom_after_mapped.header.stamp = msg->header.stamp;
        odom_after_mapped.pose.pose.orientation.x = q_w_curr.x();
        odom_after_mapped.pose.pose.orientation.y = q_w_curr.y();
        odom_after_mapped.pose.pose.orientation.z = q_w_curr.z();
        odom_after_mapped.pose.pose.orientation.w = q_w_curr.w();
        odom_after_mapped.pose.pose.position.x = t_w_curr.x();
        odom_after_mapped.pose.pose.position.y = t_w_curr.y();
        odom_after_mapped.pose.pose.position.z = t_w_curr.z();

        map_to_init_high_freq_pub_ptr_->publish(odom_after_mapped);
      });

  surf_downsize_filter_.setLeafSize(surf_downsize_leaf_size, surf_downsize_leaf_size, surf_downsize_leaf_size);
  corner_downsize_filter_.setLeafSize(corner_downsize_leaf_size, corner_downsize_leaf_size, corner_downsize_leaf_size);

  is_running_ = true;
  process_map_thread_ = std::make_unique<std::thread>(&LidarMapping::process_mapping, this);

  for (int i = 0; i < kMapWidth; ++i) {
    for (int j = 0; j < kMapHeight; ++j) {
      for (int k = 0; k < kMapDepth; ++k) {
        pcl_corner_map_(Dim{i, j, k}) = std::make_shared<pcl::PointCloud<PointType>>();
      }
    }
  }

  for (int i = 0; i < kMapWidth; ++i) {
    for (int j = 0; j < kMapHeight; ++j) {
      for (int k = 0; k < kMapDepth; ++k) {
        pcl_surf_map_(Dim{i, j, k}) = std::make_shared<pcl::PointCloud<PointType>>();
      }
    }
  }
}

void LidarMapping::process_mapping() {
  while (is_running_) {
    sensor_msgs::msg::PointCloud2::SharedPtr corner_points_msg{nullptr};
    sensor_msgs::msg::PointCloud2::SharedPtr full_points_msg{nullptr};
    sensor_msgs::msg::PointCloud2::SharedPtr surf_points_msg{nullptr};
    nav_msgs::msg::Odometry::SharedPtr odom_current_msg{nullptr};

    int64_t curr_lidar_points_utc_timestamp = 0;

    {
      std::unique_lock<std::mutex> lk(mutex_);

      cond_var_.wait_for(lk, 3s, [this]() {
        return !full_points_queue_.empty() && !corner_points_queue_.empty() && !surf_points_queue_.empty() && !lidar_odom_to_init_queue_.empty();
      });

      if (!is_running_.load()) {
        RCLCPP_INFO(this->get_logger(), "Lidar Odomtry Exit");
        break;
      }

      // Time Verification
      auto full_points_timestamp =
          static_cast<int64_t>(full_points_queue_.front()->header.stamp.sec) * 1000000000UL + full_points_queue_.front()->header.stamp.nanosec;

      auto corner_points_timestamp =
          static_cast<int64_t>(corner_points_queue_.front()->header.stamp.sec) * 1000000000UL + corner_points_queue_.front()->header.stamp.nanosec;

      auto surf_points_timestamp =
          static_cast<int64_t>(surf_points_queue_.front()->header.stamp.sec) * 1000000000UL + surf_points_queue_.front()->header.stamp.nanosec;

      auto odom_current_timestamp = static_cast<int64_t>(lidar_odom_to_init_queue_.front()->header.stamp.sec) * 1000000000UL +
                                    lidar_odom_to_init_queue_.front()->header.stamp.nanosec;

      curr_lidar_points_utc_timestamp = corner_points_timestamp;

      // AreEqual ?
      if (AreEqual(full_points_timestamp, corner_points_timestamp, surf_points_timestamp, odom_current_timestamp)) {
        full_points_msg = full_points_queue_.front();
        full_points_queue_.pop();
        corner_points_msg = corner_points_queue_.front();
        corner_points_queue_.pop();
        surf_points_msg = surf_points_queue_.front();
        surf_points_queue_.pop();
        odom_current_msg = lidar_odom_to_init_queue_.front();
        lidar_odom_to_init_queue_.pop();
      } else {
        // find max
        auto max_timestamp = MaxOf(full_points_timestamp, corner_points_timestamp, surf_points_timestamp, odom_current_timestamp);
        if (full_points_timestamp < max_timestamp) {
          full_points_queue_.pop();
        }
        if (corner_points_timestamp < max_timestamp) {
          corner_points_queue_.pop();
        }
        if (surf_points_timestamp < max_timestamp) {
          surf_points_queue_.pop();
        }
        if (odom_current_timestamp < max_timestamp) {
          lidar_odom_to_init_queue_.pop();
        }
        continue;
      }
    }

    assert(full_points_msg != nullptr);
    assert(corner_points_msg != nullptr);
    assert(surf_points_msg != nullptr);
    assert(odom_current_msg != nullptr);

    // whole cost
    TicToc whole_tic_tok;
    // get time sync frames
    std::shared_ptr<pcl::PointCloud<PointType>> pcl_full_points_ptr = std::make_shared<pcl::PointCloud<PointType>>();
    std::shared_ptr<pcl::PointCloud<PointType>> pcl_corner_points_ptr = std::make_shared<pcl::PointCloud<PointType>>();
    std::shared_ptr<pcl::PointCloud<PointType>> pcl_surf_points_ptr = std::make_shared<pcl::PointCloud<PointType>>();

    pcl::fromROSMsg(*full_points_msg, *pcl_full_points_ptr);
    pcl::fromROSMsg(*corner_points_msg, *pcl_corner_points_ptr);
    pcl::fromROSMsg(*surf_points_msg, *pcl_surf_points_ptr);

    Eigen::Quaterniond q_wodom_curr;
    Eigen::Vector3d t_wodom_curr;
    q_wodom_curr.x() = odom_current_msg->pose.pose.orientation.x;
    q_wodom_curr.y() = odom_current_msg->pose.pose.orientation.y;
    q_wodom_curr.z() = odom_current_msg->pose.pose.orientation.z;
    q_wodom_curr.w() = odom_current_msg->pose.pose.orientation.w;
    t_wodom_curr.x() = odom_current_msg->pose.pose.position.x;
    t_wodom_curr.y() = odom_current_msg->pose.pose.position.y;
    t_wodom_curr.z() = odom_current_msg->pose.pose.position.z;
    // Do something here

    Eigen::Map<Eigen::Quaterniond> q_wmap_curr(parameters_.data());
    Eigen::Map<Eigen::Vector3d> t_wmap_curr(parameters_.data() + 4);

    // ( map to odom )  x ( odom to init )
    q_wmap_curr = q_wmap_wodom * q_wodom_curr;
    t_wmap_curr = q_wmap_wodom * t_wodom_curr + t_wmap_wodom;

    TicToc t_shift;
    int center_cube_i = int((t_wmap_curr.x() + 25.0) / 50.0) + map_center_width_;
    int center_cube_j = int((t_wmap_curr.y() + 25.0) / 50.0) + map_center_height_;
    int center_cube_k = int((t_wmap_curr.z() + 25.0) / 50.0) + map_center_depth_;

    if (t_wmap_curr.x() + 25.0 < 0) center_cube_i--;
    if (t_wmap_curr.y() + 25.0 < 0) center_cube_j--;
    if (t_wmap_curr.z() + 25.0 < 0) center_cube_k--;

    while (center_cube_i < 3) {
      for (int j = 0; j < kMapHeight; ++j) {
        for (int k = 0; k < kMapDepth; ++k) {
          // save last points clond
          auto last_widht_corner_pcl = pcl_corner_map_(Dim{kMapWidth - 1, j, k});
          auto last_widht_surf_pcl = pcl_surf_map_(Dim{kMapWidth - 1, j, k});
          for (int i = kMapWidth - 1; i > 0; i--) {
            pcl_corner_map_(Dim{i, j, k}) = pcl_corner_map_(Dim{i - 1, j, k});
            pcl_surf_map_(Dim{i, j, k}) = pcl_surf_map_(Dim{i - 1, j, k});
          }
          // make start clear
          last_widht_corner_pcl->clear();
          last_widht_surf_pcl->clear();
          pcl_corner_map_(Dim{0, j, k}) = last_widht_corner_pcl;
          pcl_surf_map_(Dim{0, j, k}) = last_widht_surf_pcl;
        }
      }
      center_cube_i++;
      map_center_width_++;
    }

    while (center_cube_i >= kMapWidth - 3) {
      for (int j = 0; j < kMapHeight; ++j) {
        for (int k = 0; k < kMapDepth; ++k) {
          auto last_widht_corner_pcl = pcl_corner_map_(Dim{0, j, k});
          auto last_widht_surf_pcl = pcl_surf_map_(Dim{0, j, k});
          for (int i = 0; i < kMapWidth - 1; i++) {
            pcl_corner_map_(Dim{i, j, k}) = pcl_corner_map_(Dim{i + 1, j, k});
            pcl_surf_map_(Dim{i, j, k}) = pcl_surf_map_(Dim{i + 1, j, k});
          }
          last_widht_corner_pcl->clear();
          last_widht_surf_pcl->clear();
          pcl_corner_map_(Dim{kMapWidth - 1, j, k}) = last_widht_corner_pcl;
          pcl_surf_map_(Dim{kMapWidth - 1, j, k}) = last_widht_surf_pcl;
        }
      }
      center_cube_i--;
      map_center_width_--;
    }

    while (center_cube_j < 3) {
      for (int i = 0; i < kMapWidth; ++i) {
        for (int k = 0; k < kMapDepth; ++k) {
          auto last_widht_corner_pcl = pcl_corner_map_(Dim{i, kMapHeight - 1, k});
          auto last_widht_surf_pcl = pcl_surf_map_(Dim{i, kMapHeight - 1, k});
          for (int j = kMapHeight - 1; j > 0; j--) {
            pcl_corner_map_(Dim{i, j, k}) = pcl_corner_map_(Dim{i, j - 1, k});
            pcl_surf_map_(Dim{i, j, k}) = pcl_surf_map_(Dim{i, j - 1, k});
          }
          last_widht_corner_pcl->clear();
          last_widht_surf_pcl->clear();
          pcl_corner_map_(Dim{i, 0, k}) = last_widht_corner_pcl;
          pcl_surf_map_(Dim{i, 0, k}) = last_widht_surf_pcl;
        }
      }
      center_cube_j++;
      map_center_height_++;
    }

    while (center_cube_j >= kMapHeight - 3) {
      for (int i = 0; i < kMapWidth; ++i) {
        for (int k = 0; k < kMapDepth; ++k) {
          auto last_widht_corner_pcl = pcl_corner_map_(Dim{i, 0, k});
          auto last_widht_surf_pcl = pcl_surf_map_(Dim{i, 0, k});
          for (int j = 0; j < kMapHeight - 1; j++) {
            pcl_corner_map_(Dim{i, j, k}) = pcl_corner_map_(Dim{i, j + 1, k});
            pcl_surf_map_(Dim{i, j, k}) = pcl_surf_map_(Dim{i, j + 1, k});
          }
          last_widht_corner_pcl->clear();
          last_widht_surf_pcl->clear();
          pcl_corner_map_(Dim{i, kMapHeight - 1, k}) = last_widht_corner_pcl;
          pcl_surf_map_(Dim{i, kMapHeight - 1, k}) = last_widht_surf_pcl;
        }
      }
      center_cube_i--;
      map_center_height_--;
    }

    while (center_cube_k < 3) {
      for (int i = 0; i < kMapWidth; ++i) {
        for (int j = 0; j < kMapHeight; ++j) {
          auto last_widht_corner_pcl = pcl_corner_map_(Dim{i, j, kMapDepth - 1});
          auto last_widht_surf_pcl = pcl_surf_map_(Dim{i, j, kMapDepth - 1});
          for (int k = kMapDepth - 1; k > 0; k--) {
            pcl_corner_map_(Dim{i, j, k}) = pcl_corner_map_(Dim{i, j, k - 1});
            pcl_surf_map_(Dim{i, j, k}) = pcl_surf_map_(Dim{i, j, k - 1});
          }
          last_widht_corner_pcl->clear();
          last_widht_surf_pcl->clear();
          pcl_corner_map_(Dim{i, j, 0}) = last_widht_corner_pcl;
          pcl_surf_map_(Dim{i, j, 0}) = last_widht_surf_pcl;
        }
      }
      center_cube_k++;
      map_center_depth_++;
    }

    while (center_cube_k >= kMapDepth - 3) {
      for (int i = 0; i < kMapWidth; ++i) {
        for (int j = 0; j < kMapHeight; ++j) {
          auto last_widht_corner_pcl = pcl_corner_map_(Dim{i, j, 0});
          auto last_widht_surf_pcl = pcl_surf_map_(Dim{i, j, 0});
          for (int k = 0; k < kMapDepth - 1; k++) {
            pcl_corner_map_(Dim{i, j, k}) = pcl_corner_map_(Dim{i, j, k + 1});
            pcl_surf_map_(Dim{i, j, k}) = pcl_surf_map_(Dim{i, j, k + 1});
          }
          last_widht_corner_pcl->clear();
          last_widht_surf_pcl->clear();
          pcl_corner_map_(Dim{i, j, kMapDepth - 1}) = last_widht_corner_pcl;
          pcl_surf_map_(Dim{i, j, kMapDepth - 1}) = last_widht_surf_pcl;
        }
      }
      center_cube_k--;
      map_center_depth_--;
    }

    int points_cloud_valid_num = 0;
    int points_cloud_surround_num = 0;

    for (int i = center_cube_i - 2; i <= center_cube_i + 2; i++) {
      for (int j = center_cube_j - 2; j <= center_cube_j + 2; j++) {
        for (int k = center_cube_k - 1; k <= center_cube_k + 1; k++) {
          if (i >= 0 && i < kMapWidth && j >= 0 && j < kMapHeight && k >= 0 && k < kMapDepth)  // 如果坐标合法
          {
            // 记录submap中的所有cube的index，记为有效index
            map_vaild_index_[points_cloud_valid_num] = Dim{i, j, k};
            points_cloud_valid_num++;
            map_surround_index_[points_cloud_surround_num] = Dim{i, j, k};
            points_cloud_surround_num++;
          }
        }
      }
    }

    std::shared_ptr<pcl::PointCloud<PointType>> pcl_corner_from_map_ptr = std::make_shared<pcl::PointCloud<PointType>>();
    std::shared_ptr<pcl::PointCloud<PointType>> pcl_surf_from_map_ptr = std::make_shared<pcl::PointCloud<PointType>>();
    for (int i = 0; i < points_cloud_valid_num; i++) {
      // 将有效index的cube中的点云叠加到一起组成submap的特征点云
      *pcl_corner_from_map_ptr += *pcl_corner_map_[map_vaild_index_[i]];
      *pcl_surf_from_map_ptr += *pcl_surf_map_[map_vaild_index_[i]];
    }

    int pcl_corner_from_map_points_num = pcl_corner_from_map_ptr->points.size();
    int pcl_surf_from_map_points_num = pcl_surf_from_map_ptr->points.size();

    std::shared_ptr<pcl::PointCloud<PointType>> pcl_corner_points_stack_ptr = std::make_shared<pcl::PointCloud<PointType>>();
    corner_downsize_filter_.setInputCloud(pcl_corner_points_ptr);
    corner_downsize_filter_.filter(*pcl_corner_points_stack_ptr);
    int pcl_corner_stack_points_num = pcl_corner_points_stack_ptr->points.size();

    std::shared_ptr<pcl::PointCloud<PointType>> pcl_surf_points_stack_ptr = std::make_shared<pcl::PointCloud<PointType>>();
    surf_downsize_filter_.setInputCloud(pcl_surf_points_ptr);
    surf_downsize_filter_.filter(*pcl_surf_points_stack_ptr);
    int pcl_surf_stack_points_num = pcl_surf_points_stack_ptr->points.size();

    RCLCPP_INFO(this->get_logger(), "prepare cost %.3f ms", t_shift.toc() / 1e6);
    RCLCPP_INFO(this->get_logger(), "map corner num: %d, map surf num : %d", pcl_corner_from_map_points_num, pcl_surf_from_map_points_num);

    if (pcl_corner_from_map_points_num > 50 && pcl_surf_from_map_points_num > 50) {
      TicToc t_opt;
      TicToc t_tree;
      corner_map_kdtree_.setInputCloud(pcl_corner_from_map_ptr);
      surf_map_kdtree_.setInputCloud(pcl_surf_from_map_ptr);
      RCLCPP_INFO(this->get_logger(), "t_tree cost %.3f ms", t_tree.toc() / 1e6);

      for (int iter_count = 0; iter_count < 2; iter_count++) {
        ceres::LossFunction* loss_function = new ceres::HuberLoss(0.1);
        ceres::Manifold* q_parameterization = new ceres::QuaternionManifold();

        ceres::Problem::Options problem_options;

        ceres::Problem problem(problem_options);
        problem.AddParameterBlock(parameters_.data(), 4, q_parameterization);
        problem.AddParameterBlock(parameters_.data() + 4, 3);

        TicToc t_data;
        int corner_num = 0;

        for (int i = 0; i < pcl_corner_stack_points_num; ++i) {
          std::vector<int> point_search_index;
          std::vector<float> point_search_sqdis;

          auto point_ori = pcl_corner_points_stack_ptr->points[i];
          auto point_map = pcl_corner_points_stack_ptr->points[i];

          // transfrom to map coordinate system
          Eigen::Vector3d point_3d_curr(point_ori.x, point_ori.y, point_ori.z);
          Eigen::Vector3d point_3d_map = q_wmap_curr * point_3d_curr + t_wmap_curr;
          point_map.x = point_3d_map.x();
          point_map.y = point_3d_map.y();
          point_map.z = point_3d_map.z();

          corner_map_kdtree_.nearestKSearch(point_map, 5, point_search_index, point_search_sqdis);
          // If the farthest of the five points is less than 1 meter, the covariance matrix is solved
          if (point_search_sqdis.size() > 0 && point_search_sqdis[point_search_sqdis.size() - 1] < 1.0) {
            std::vector<Eigen::Vector3d> near_corners;
            Eigen::Vector3d center(0, 0, 0);

            for (size_t index = 0; index < point_search_sqdis.size(); ++index) {
              Eigen::Vector3d tmp{pcl_corner_points_stack_ptr->points[point_search_index[index]].x,
                                  pcl_corner_points_stack_ptr->points[point_search_index[index]].y,
                                  pcl_corner_points_stack_ptr->points[point_search_index[index]].z};

              center += tmp;
              near_corners.push_back(tmp);
            }

            center /= near_corners.size();

            // covariance matrix
            Eigen::Matrix3d cov_mat = Eigen::Matrix3d::Zero();
            for (int j = 0; j < 5; j++) {
              Eigen::Matrix<double, 3, 1> tmp = near_corners[j] - center;
              cov_mat = cov_mat + tmp * tmp.transpose();
            }

            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(cov_mat);
            Eigen::Vector3d unit_direction = saes.eigenvectors().col(2);
            Eigen::Vector3d curr_point(point_ori.x, point_ori.y, point_ori.z);
            // max eigenvector > 3 * min eigenvector
            if (saes.eigenvalues()[2] > 3 * saes.eigenvalues()[1]) {
              Eigen::Vector3d point_on_line = center;
              // Move 0.1m from the center point along the direction vector to both ends to construct two points on the line
              auto point_a = 0.1 * unit_direction + point_on_line;
              auto point_b = -0.1 * unit_direction + point_on_line;
              // Point-to-line distance
              ceres::CostFunction* cost_function = LidarEdgeFactor::Create(curr_point, point_a, point_b);
              problem.AddResidualBlock(cost_function, loss_function, parameters_.data(), parameters_.data() + 4);
              corner_num++;
            }
          }
        }

        int surf_num = 0;
        for (int i = 0; i < pcl_surf_from_map_points_num; ++i) {
          std::vector<int> point_search_index;
          std::vector<float> point_search_sqdis;

          auto point_ori = pcl_corner_points_stack_ptr->points[i];
          auto point_map = pcl_corner_points_stack_ptr->points[i];

          // transfrom to map coordinate system
          Eigen::Vector3d point_3d_curr(point_ori.x, point_ori.y, point_ori.z);
          Eigen::Vector3d point_3d_map = q_wmap_curr * point_3d_curr + t_wmap_curr;
          point_map.x = point_3d_map.x();
          point_map.y = point_3d_map.y();
          point_map.z = point_3d_map.z();

          surf_map_kdtree_.nearestKSearch(point_map, 5, point_search_index, point_search_sqdis);
          // Ax + By + Cz + 1 = 0
          Eigen::Matrix<double, 5, 3> matA0 = Eigen::Matrix<double, 5, 3>::Zero();
          Eigen::Matrix<double, 5, 1> matB0 = -1 * Eigen::Matrix<double, 5, 1>::Ones();

          // matA0 * norm（A, B, C） = matB0
          if (point_search_sqdis.size() == 5 && point_search_sqdis[point_search_sqdis.size() - 1] < 1.0) {
            for (int index = 0; index < 5; index++) {
              matA0(index, 0) = pcl_surf_from_map_ptr->points[point_search_index[index]].x;
              matA0(index, 1) = pcl_surf_from_map_ptr->points[point_search_index[index]].y;
              matA0(index, 2) = pcl_surf_from_map_ptr->points[point_search_index[index]].z;
            }

            Eigen::Vector3d norm = matA0.colPivHouseholderQr().solve(matB0);
            double negative_OA_dot_norm = 1 / norm.norm();
            norm.normalize();

            bool plane_valid = true;
            for (int index = 0; index < 5; index++) {
              if (std::abs(norm(0) * pcl_surf_from_map_ptr->points[point_search_index[index]].x +
                           norm(1) * pcl_surf_from_map_ptr->points[point_search_index[index]].y +
                           norm(2) * pcl_surf_from_map_ptr->points[point_search_index[index]].z + negative_OA_dot_norm) > 0.2) {
                plane_valid = false;
                break;
              }
            }

            Eigen::Vector3d curr_point(point_ori.x, point_ori.y, point_ori.z);
            if (plane_valid) {
              // Point-to-polygon distance
              ceres::CostFunction* cost_function = LidarPlaneNormFactor::Create(curr_point, norm, negative_OA_dot_norm);
              problem.AddResidualBlock(cost_function, loss_function, parameters_.data(), parameters_.data() + 4);
              surf_num++;
            }
          }
        }

        RCLCPP_INFO(this->get_logger(), "mapping data assosiation cost %.3f ms", t_data.toc() / 1e6);
        TicToc t_solver;
        ceres::Solver::Options options;
        options.linear_solver_type = ceres::DENSE_QR;
        options.max_num_iterations = 4;
        options.minimizer_progress_to_stdout = false;
        options.check_gradients = false;
        options.gradient_check_relative_precision = 1e-4;
        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);
        RCLCPP_INFO(this->get_logger(), "mapping solver cost %.3f ms", t_data.toc() / 1e6);
      }
      RCLCPP_INFO(this->get_logger(), "mapping optimization cost %.3f ms", t_opt.toc() / 1e6);
    } else {
      RCLCPP_WARN(this->get_logger(), "mapping points are not enough");
    }

    // update map to odom
    // ( init to  map ) x ( odom to init  )
    // TODO Don't understand
    q_wmap_wodom = q_wmap_curr * q_wodom_curr.inverse();
    t_wmap_wodom = t_wmap_curr - q_wmap_wodom * t_wodom_curr;

    TicToc t_add;
    for (int i = 0; i < pcl_corner_stack_points_num; i++) {
      auto point_ori = pcl_corner_points_stack_ptr->points[i];
      auto point_map = pcl_corner_points_stack_ptr->points[i];

      // transfrom to map coordinate system
      Eigen::Vector3d point_3d_curr(point_ori.x, point_ori.y, point_ori.z);
      Eigen::Vector3d point_3d_map = q_wmap_curr * point_3d_curr + t_wmap_curr;
      point_map.x = point_3d_map.x();
      point_map.y = point_3d_map.y();
      point_map.z = point_3d_map.z();

      int cubeI = int((point_map.x + 25.0) / 50.0) + map_center_width_;
      int cubeJ = int((point_map.y + 25.0) / 50.0) + map_center_height_;
      int cubeK = int((point_map.z + 25.0) / 50.0) + map_center_depth_;

      if (point_map.x + 25.0 < 0) cubeI--;
      if (point_map.y + 25.0 < 0) cubeJ--;
      if (point_map.z + 25.0 < 0) cubeK--;

      if (cubeI >= 0 && cubeI < kMapWidth && cubeJ >= 0 && cubeJ < kMapHeight && cubeK >= 0 && cubeK < kMapDepth) {
        pcl_corner_map_(Dim{cubeI, cubeJ, cubeK})->push_back(point_map);
      }
    }

    for (int i = 0; i < pcl_surf_stack_points_num; i++) {
      auto point_ori = pcl_surf_points_stack_ptr->points[i];
      auto point_map = pcl_surf_points_stack_ptr->points[i];

      // transfrom to map coordinate system
      Eigen::Vector3d point_3d_curr(point_ori.x, point_ori.y, point_ori.z);
      Eigen::Vector3d point_3d_map = q_wmap_curr * point_3d_curr + t_wmap_curr;
      point_map.x = point_3d_map.x();
      point_map.y = point_3d_map.y();
      point_map.z = point_3d_map.z();

      int cubeI = int((point_map.x + 25.0) / 50.0) + map_center_width_;
      int cubeJ = int((point_map.y + 25.0) / 50.0) + map_center_height_;
      int cubeK = int((point_map.z + 25.0) / 50.0) + map_center_depth_;

      if (point_map.x + 25.0 < 0) cubeI--;
      if (point_map.y + 25.0 < 0) cubeJ--;
      if (point_map.z + 25.0 < 0) cubeK--;

      if (cubeI >= 0 && cubeI < kMapWidth && cubeJ >= 0 && cubeJ < kMapHeight && cubeK >= 0 && cubeK < kMapDepth) {
        pcl_surf_map_(Dim{cubeI, cubeJ, cubeK})->push_back(point_map);
      }
    }

    RCLCPP_INFO(this->get_logger(), "mapping adding cost %.3f ms", t_add.toc() / 1e6);

    sensor_msgs::msg::PointCloud2 ros2_tmp_msg;

    TicToc t_pub;
    static size_t frame_count = 0;
    if (frame_count % 5 == 0) {
      pcl::PointCloud<PointType> surround_points;

      for (size_t index = 0; index < points_cloud_surround_num; ++index) {
        surround_points += *pcl_corner_map_(map_surround_index_[index]);
        surround_points += *pcl_surf_map_(map_surround_index_[index]);
      }

      pcl::toROSMsg(surround_points, ros2_tmp_msg);
      ros2_tmp_msg.header.frame_id = "map";
      ros2_tmp_msg.header.stamp.sec = curr_lidar_points_utc_timestamp / 1000000;
      ros2_tmp_msg.header.stamp.nanosec = curr_lidar_points_utc_timestamp % 1000000;
      map_surround_pub_ptr_->publish(ros2_tmp_msg);
    }

    if (frame_count % 20 == 0) {
      pcl::PointCloud<PointType> map_points;

      for (int i = 0; i < kMapWidth; ++i) {
        for (int j = 0; j < kMapHeight; ++j) {
          for (int k = 0; k < kMapDepth; ++k) {
            map_points += *pcl_corner_map_[{i, j, k}];
            map_points += *pcl_surf_map_[{i, j, k}];
          }
        }
      }
      pcl::toROSMsg(map_points, ros2_tmp_msg);
      ros2_tmp_msg.header.frame_id = "map";
      ros2_tmp_msg.header.stamp.sec = curr_lidar_points_utc_timestamp / 1000000;
      ros2_tmp_msg.header.stamp.nanosec = curr_lidar_points_utc_timestamp % 1000000;
      map_pub_ptr_->publish(ros2_tmp_msg);
    }

    // the frame point in map
    int full_points_number = pcl_full_points_ptr->points.size();
    for (int i = 0; i < full_points_number; i++) {
      Eigen::Vector3d point_3d_curr(pcl_full_points_ptr->points[i].x, pcl_full_points_ptr->points[i].y, pcl_full_points_ptr->points[i].z);
      Eigen::Vector3d point_3d_map = q_wmap_curr * point_3d_curr + t_wmap_curr;
      pcl_full_points_ptr->points[i].x = point_3d_map.x();
      pcl_full_points_ptr->points[i].y = point_3d_map.y();
      pcl_full_points_ptr->points[i].z = point_3d_map.z();
    }
    pcl::toROSMsg(*pcl_full_points_ptr, ros2_tmp_msg);
    ros2_tmp_msg.header.frame_id = "map";
    ros2_tmp_msg.header.stamp.sec = curr_lidar_points_utc_timestamp / 1000000;
    ros2_tmp_msg.header.stamp.nanosec = curr_lidar_points_utc_timestamp % 1000000;
    full_points_pub_ptr_->publish(ros2_tmp_msg);

    nav_msgs::msg::Odometry odomAftMapped;
    odomAftMapped.header.frame_id = "lidar";
    odomAftMapped.child_frame_id = "mapped";
    odomAftMapped.header.stamp.sec = curr_lidar_points_utc_timestamp / 1000000;
    odomAftMapped.header.stamp.nanosec = curr_lidar_points_utc_timestamp % 1000000;
    odomAftMapped.pose.pose.orientation.x = q_wmap_curr.x();
    odomAftMapped.pose.pose.orientation.y = q_wmap_curr.y();
    odomAftMapped.pose.pose.orientation.z = q_wmap_curr.z();
    odomAftMapped.pose.pose.orientation.w = q_wmap_curr.w();
    odomAftMapped.pose.pose.position.x = t_wmap_curr.x();
    odomAftMapped.pose.pose.position.y = t_wmap_curr.y();
    odomAftMapped.pose.pose.position.z = t_wmap_curr.z();
    map_to_init_pub_ptr_->publish(odomAftMapped);

    geometry_msgs::msg::PoseStamped pose;
    pose.header = odomAftMapped.header;
    pose.pose = odomAftMapped.pose.pose;
    mapping_path_.poses.push_back(pose);
    map_path_pub_ptr_->publish(mapping_path_);

    static tf2_ros::TransformBroadcaster br(*this);
    geometry_msgs::msg::TransformStamped static_transformStamped;

    static_transformStamped.header.stamp.sec = curr_lidar_points_utc_timestamp / 1000000;
    static_transformStamped.header.stamp.nanosec = curr_lidar_points_utc_timestamp % 1000000;
    static_transformStamped.header.frame_id = "lidar";
    static_transformStamped.child_frame_id = "map";

    static_transformStamped.transform.translation.x = t_wmap_curr(0);
    static_transformStamped.transform.translation.y = t_wmap_curr(1);
    static_transformStamped.transform.translation.z = t_wmap_curr(2);
    static_transformStamped.transform.rotation.w = q_wmap_curr.w();
    static_transformStamped.transform.rotation.x = q_wmap_curr.x();
    static_transformStamped.transform.rotation.y = q_wmap_curr.y();
    static_transformStamped.transform.rotation.z = q_wmap_curr.z();

    br.sendTransform(static_transformStamped);

    frame_count++;
  }
}