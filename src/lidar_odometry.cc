#include "aloam-for-ros2/lidar_odometry.hpp"

#include <ceres/loss_function.h>
#include <fmt/format.h>

#include <chrono>

#include "aloam-for-ros2/lidar_factor.hpp"
#include "aloam-for-ros2/tools/tic_tok.hpp"
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

LidarOdometry::LidarOdometry(const std::string& node_namespace) : Node("lidar_odometry"), node_namespace_(node_namespace) {
  corner_sharp_points_sub_ptr_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(fmt::format("/{}/sharp_points", node_namespace), 10,
                                                                                          [this](sensor_msgs::msg::PointCloud2::SharedPtr msg) {
                                                                                            {
                                                                                              std::lock_guard<std::mutex> lk(mutex_);
                                                                                              corner_sharp_points_cloud_queue_.push(msg);
                                                                                            }
                                                                                            cond_var_.notify_one();
                                                                                          });

  corner_less_sharp_points_sub_ptr_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      fmt::format("/{}/less_sharp_points", node_namespace), 10, [this](sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        {
          std::lock_guard<std::mutex> lk(mutex_);
          corner_less_sharp_points_cloud_queue_.push(msg);
        }
        cond_var_.notify_one();
      });

  surf_flat_points_sub_ptr_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(fmt::format("/{}/flat_points", node_namespace), 10,
                                                                                       [this](sensor_msgs::msg::PointCloud2::SharedPtr msg) {
                                                                                         {
                                                                                           std::lock_guard<std::mutex> lk(mutex_);
                                                                                           surf_flat_points_cloud_queue_.push(msg);
                                                                                         }
                                                                                         cond_var_.notify_one();
                                                                                       });

  surf_less_flat_points_sub_ptr_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(fmt::format("/{}/less_flat_points", node_namespace), 10,
                                                                                            [this](sensor_msgs::msg::PointCloud2::SharedPtr msg) {
                                                                                              {
                                                                                                std::lock_guard<std::mutex> lk(mutex_);
                                                                                                surf_less_flat_points_cloud_queue_.push(msg);
                                                                                              }
                                                                                              cond_var_.notify_one();
                                                                                            });

  // pub
  corner_points_cloud_last_pub_ptr_ =
      this->create_publisher<sensor_msgs::msg::PointCloud2>(fmt::format("/{}/corner_points_last", node_namespace), 10);
  surf_points_cloud_last_pub_ptr_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(fmt::format("/{}/surf_points_last", node_namespace), 10);
  lidar_odom_to_init_pub_ptr_ = this->create_publisher<nav_msgs::msg::Odometry>(fmt::format("/{}/lidar_odom_to_init", node_namespace), 10);
  lidar_odom_path_pub_ptr_ = this->create_publisher<nav_msgs::msg::Path>(fmt::format("/{}/lidar_odom_path", node_namespace), 10);

  is_running_.store(true);
  // process thread
  process_odom_thread_ptr_ = std::make_unique<std::thread>(&LidarOdometry::ProcessOdom, this);
}

void LidarOdometry::UndistortLidarPointsToLastFrame(const PointType& point_origin, PointType& point, const Eigen::Map<Eigen::Quaterniond>& q,
                                                    const Eigen::Map<Eigen::Vector3d>& t) {
  Eigen::Vector3d point_vector(point_origin.x, point_origin.y, point_origin.z);
  Eigen::Vector3d un_point = q * point_vector + t;

  point = point_origin;
  point.x = un_point.x();
  point.y = un_point.y();
  point.z = un_point.z();
}

void LidarOdometry::ProcessOdom() {
  while (is_running_.load()) {
    static bool static_system_init_done{false};

    std::shared_ptr<sensor_msgs::msg::PointCloud2> corner_sharp_points_msg{nullptr};
    std::shared_ptr<sensor_msgs::msg::PointCloud2> corner_less_sharp_points_msg{nullptr};
    std::shared_ptr<sensor_msgs::msg::PointCloud2> surf_flat_points_msg{nullptr};
    std::shared_ptr<sensor_msgs::msg::PointCloud2> surf_less_flat_points_msg{nullptr};

    int64_t curr_lidar_points_utc_timestamp = 0;

    {
      std::unique_lock<std::mutex> lk(mutex_);

      auto success = cond_var_.wait_for(lk, 100ms, [this]() {
        return !corner_sharp_points_cloud_queue_.empty() && !corner_less_sharp_points_cloud_queue_.empty() &&
               !surf_flat_points_cloud_queue_.empty() && !surf_less_flat_points_cloud_queue_.empty();
      });

      if (!is_running_.load()) {
        RCLCPP_INFO(this->get_logger(), "Lidar Odomtry Exit");
      }

      if (!success) {
        RCLCPP_WARN(this->get_logger(), "No data in 100ms, this is unnormal!!");
        continue;
      }

      // Time Verification
      auto corner_sharp_timestamp = static_cast<int64_t>(corner_sharp_points_cloud_queue_.front()->header.stamp.sec) * 1000000000UL +
                                    corner_sharp_points_cloud_queue_.front()->header.stamp.nanosec;

      auto corner_less_sharp_timestamp = static_cast<int64_t>(corner_less_sharp_points_cloud_queue_.front()->header.stamp.sec) * 1000000000UL +
                                         corner_less_sharp_points_cloud_queue_.front()->header.stamp.nanosec;

      auto surf_flat_timestamp = static_cast<int64_t>(surf_flat_points_cloud_queue_.front()->header.stamp.sec) * 1000000000UL +
                                 surf_flat_points_cloud_queue_.front()->header.stamp.nanosec;

      auto surf_less_flat_timestamp = static_cast<int64_t>(surf_less_flat_points_cloud_queue_.front()->header.stamp.sec) * 1000000000UL +
                                      surf_less_flat_points_cloud_queue_.front()->header.stamp.nanosec;

      curr_lidar_points_utc_timestamp = corner_sharp_timestamp;

      // AreEqual ?
      if (AreEqual(corner_sharp_timestamp, corner_less_sharp_timestamp, surf_flat_timestamp, surf_less_flat_timestamp)) {
        corner_sharp_points_msg = corner_sharp_points_cloud_queue_.front();
        corner_sharp_points_cloud_queue_.pop();
        corner_less_sharp_points_msg = corner_less_sharp_points_cloud_queue_.front();
        corner_less_sharp_points_cloud_queue_.pop();
        surf_flat_points_msg = surf_flat_points_cloud_queue_.front();
        surf_flat_points_cloud_queue_.pop();
        surf_less_flat_points_msg = surf_less_flat_points_cloud_queue_.front();
        surf_less_flat_points_cloud_queue_.pop();
      } else {
        // find max
        auto max_timestamp = MaxOf(corner_sharp_timestamp, corner_less_sharp_timestamp, surf_flat_timestamp, surf_less_flat_timestamp);
        if (corner_sharp_timestamp < max_timestamp) {
          corner_sharp_points_cloud_queue_.pop();
        }
        if (corner_less_sharp_timestamp < max_timestamp) {
          corner_less_sharp_points_cloud_queue_.pop();
        }
        if (surf_flat_timestamp < max_timestamp) {
          surf_flat_points_cloud_queue_.pop();
        }
        if (surf_less_flat_timestamp < max_timestamp) {
          surf_less_flat_points_cloud_queue_.pop();
        }
      }
      continue;
    }

    assert(corner_sharp_points_msg != nullptr);
    assert(corner_less_sharp_points_msg != nullptr);
    assert(surf_flat_points_msg != nullptr);
    assert(surf_less_flat_points_msg != nullptr);

    // whole cost
    TicToc whole_tic_tok;

    auto pcl_corner_sharp_points_ptr = std::make_shared<pcl::PointCloud<PointType>>();
    auto pcl_corner_less_sharp_points_ptr = std::make_shared<pcl::PointCloud<PointType>>();
    auto pcl_surf_flat_points_ptr = std::make_shared<pcl::PointCloud<PointType>>();
    auto pcl_surl_less_points_ptr = std::make_shared<pcl::PointCloud<PointType>>();

    pcl::fromROSMsg(*corner_sharp_points_msg, *pcl_corner_sharp_points_ptr);
    pcl::fromROSMsg(*corner_less_sharp_points_msg, *pcl_corner_less_sharp_points_ptr);
    pcl::fromROSMsg(*surf_flat_points_msg, *pcl_surf_flat_points_ptr);
    pcl::fromROSMsg(*surf_less_flat_points_msg, *pcl_surl_less_points_ptr);

    if (!static_system_init_done) {
      static_system_init_done = true;
      RCLCPP_INFO(this->get_logger(), "system init done");
    } else {
      // quaternion_ & translation_
      // It represents the increment between the poses P in two world coordinate systems
      Eigen::Map<Eigen::Quaterniond> q_last_curr(quaternion_.data());
      Eigen::Map<Eigen::Vector3d> t_last_curr(translation_.data());

      size_t corner_sharp_points_number = pcl_corner_sharp_points_ptr->points.size();
      size_t surf_flat_points_number = pcl_surf_flat_points_ptr->points.size();
      TicToc tic_tok;
      // Point-to-line and point-to-plane ICP, iterated 2 times
      for (size_t opti_count = 0; opti_count < 2; ++opti_count) {
        // number of matches
        size_t corner_correspondence_number = 0;
        size_t plane_correspondence_number = 0;

        ceres::LossFunction* loss_function = new ceres::HuberLoss(0.1);
        ceres::Manifold* q_parameterization = new ceres::QuaternionManifold();

        ceres::Problem::Options problem_options;

        ceres::Problem problem(problem_options);

        problem.AddParameterBlock(quaternion_.data(), 4, q_parameterization);
        problem.AddParameterBlock(translation_.data(), 3);

        PointType point_select;
        std::vector<int> points_select_indexs;
        std::vector<float> point_search_sqr_distance;

        TicToc search_tic_tok;

        // find corner feature
        for (size_t i = 0; i < corner_sharp_points_number; ++i) {
          UndistortLidarPointsToLastFrame(pcl_corner_sharp_points_ptr->points[i], point_select, q_last_curr, t_last_curr);
          last_corner_kd_tree_.nearestKSearch(point_select, 1, points_select_indexs, point_search_sqr_distance);

          int closest_point_idx = -1;
          int second_feature_point_idx = -1;

          if (!point_search_sqr_distance.empty() && point_search_sqr_distance[0] < kDistanceThreshold) {
            closest_point_idx = points_select_indexs[0];

            double second_freature_point_sqr_distance = kDistanceThreshold;

            // find another point
            for (int j = closest_point_idx + 1; j < static_cast<int>(last_corner_point_cloud_->points.size()); ++j) {
              if (last_corner_point_cloud_->points[j].ring == last_corner_point_cloud_->points[closest_point_idx].ring) {
                // same line id, skip
                continue;
              }

              // line id too far away
              if ((last_corner_point_cloud_->points[j].ring > (last_corner_point_cloud_->points[closest_point_idx].ring + 3)) ||
                  ((last_corner_point_cloud_->points[closest_point_idx].ring >= 3) &&
                   (last_corner_point_cloud_->points[j].ring < (last_corner_point_cloud_->points[closest_point_idx].ring - 3)))) {
                break;
              }

              double point_sqr_dis =
                  (last_corner_point_cloud_->points[j].x - point_select.x) * (last_corner_point_cloud_->points[j].x - point_select.x) +
                  (last_corner_point_cloud_->points[j].y - point_select.y) * (last_corner_point_cloud_->points[j].y - point_select.y) +
                  (last_corner_point_cloud_->points[j].z - point_select.z) * (last_corner_point_cloud_->points[j].z - point_select.z);

              if (point_sqr_dis < second_freature_point_sqr_distance) {
                // find nearer point
                second_freature_point_sqr_distance = point_sqr_dis;
                second_feature_point_idx = j;
              }
            }

            // another direction
            for (int j = closest_point_idx - 1; j >= 0; --j) {
              if (last_corner_point_cloud_->points[j].ring == last_corner_point_cloud_->points[closest_point_idx].ring) {
                // same line id, skip
                continue;
              }

              // line id too far away
              if ((last_corner_point_cloud_->points[j].ring > (last_corner_point_cloud_->points[closest_point_idx].ring + 3)) ||
                  ((last_corner_point_cloud_->points[closest_point_idx].ring >= 3) &&
                   (last_corner_point_cloud_->points[j].ring < (last_corner_point_cloud_->points[closest_point_idx].ring - 3)))) {
                break;
              }

              double point_sqr_dis =
                  (last_corner_point_cloud_->points[j].x - point_select.x) * (last_corner_point_cloud_->points[j].x - point_select.x) +
                  (last_corner_point_cloud_->points[j].y - point_select.y) * (last_corner_point_cloud_->points[j].y - point_select.y) +
                  (last_corner_point_cloud_->points[j].z - point_select.z) * (last_corner_point_cloud_->points[j].z - point_select.z);

              if (point_sqr_dis < second_freature_point_sqr_distance) {
                // find nearer point
                second_freature_point_sqr_distance = point_sqr_dis;
                second_feature_point_idx = j;
              }
            }
          }

          // find two feature point
          if (second_feature_point_idx >= 0) {
            Eigen::Vector3d curr_point(pcl_corner_sharp_points_ptr->points[i].x, pcl_corner_sharp_points_ptr->points[i].y,
                                       pcl_corner_sharp_points_ptr->points[i].z);
            Eigen::Vector3d last_point_a(last_corner_point_cloud_->points[closest_point_idx].x, last_corner_point_cloud_->points[closest_point_idx].y,
                                         last_corner_point_cloud_->points[closest_point_idx].z);
            Eigen::Vector3d last_point_b(last_corner_point_cloud_->points[second_feature_point_idx].x,
                                         last_corner_point_cloud_->points[second_feature_point_idx].y,
                                         last_corner_point_cloud_->points[second_feature_point_idx].z);

            ceres::CostFunction* cost_function = LidarEdgeFactor::Create(curr_point, last_point_a, last_point_b);
            problem.AddResidualBlock(cost_function, loss_function, quaternion_.data(), translation_.data());
            corner_correspondence_number++;
          }
        }

        // surf freature
        points_select_indexs.clear();
        point_search_sqr_distance.clear();
        for (size_t i = 0; i < surf_flat_points_number; ++i) {
          UndistortLidarPointsToLastFrame(pcl_surf_flat_points_ptr->points[i], point_select, q_last_curr, t_last_curr);
          last_surf_kd_tree_.nearestKSearch(point_select, 1, points_select_indexs, point_search_sqr_distance);

          int closest_point_idx = -1;
          int second_feature_point_idx = -1;
          int third_feature_point_idx = -1;

          if (!point_search_sqr_distance.empty() && point_search_sqr_distance[0] < kDistanceThreshold) {
            closest_point_idx = points_select_indexs[0];

            double second_freature_point_sqr_distance = kDistanceThreshold;
            double third_freature_point_sqr_distance = kDistanceThreshold;

            for (int j = closest_point_idx + 1; j < static_cast<int>(last_surf_point_cloud_->points.size()); ++j) {
              // line id too far away
              if ((last_surf_point_cloud_->points[j].ring > (last_surf_point_cloud_->points[closest_point_idx].ring + 3)) ||
                  ((last_surf_point_cloud_->points[closest_point_idx].ring >= 3) &&
                   (last_surf_point_cloud_->points[j].ring < (last_surf_point_cloud_->points[closest_point_idx].ring - 3)))) {
                break;
              }

              double point_sqr_dis = (last_surf_point_cloud_->points[j].x - point_select.x) * (last_surf_point_cloud_->points[j].x - point_select.x) +
                                     (last_surf_point_cloud_->points[j].y - point_select.y) * (last_surf_point_cloud_->points[j].y - point_select.y) +
                                     (last_surf_point_cloud_->points[j].z - point_select.z) * (last_surf_point_cloud_->points[j].z - point_select.z);

              // same line id or low scan
              if (last_surf_point_cloud_->points[j].ring <= last_surf_point_cloud_->points[closest_point_idx].ring &&
                  (point_sqr_dis < second_freature_point_sqr_distance)) {
                second_freature_point_sqr_distance = point_sqr_dis;
                second_feature_point_idx = j;
              }

              // higher scan
              if (last_surf_point_cloud_->points[j].ring > last_surf_point_cloud_->points[closest_point_idx].ring &&
                  (point_sqr_dis < third_freature_point_sqr_distance)) {
                third_freature_point_sqr_distance = point_sqr_dis;
                third_feature_point_idx = j;
              }
            }

            for (int j = closest_point_idx - 1; j >= 0; --j) {
              // line id too far away
              if ((last_surf_point_cloud_->points[j].ring > (last_surf_point_cloud_->points[closest_point_idx].ring + 3)) ||
                  ((last_surf_point_cloud_->points[closest_point_idx].ring >= 3) &&
                   (last_surf_point_cloud_->points[j].ring < (last_surf_point_cloud_->points[closest_point_idx].ring - 3)))) {
                break;
              }

              double point_sqr_dis = (last_surf_point_cloud_->points[j].x - point_select.x) * (last_surf_point_cloud_->points[j].x - point_select.x) +
                                     (last_surf_point_cloud_->points[j].y - point_select.y) * (last_surf_point_cloud_->points[j].y - point_select.y) +
                                     (last_surf_point_cloud_->points[j].z - point_select.z) * (last_surf_point_cloud_->points[j].z - point_select.z);

              // same line id or higher scan
              if (last_surf_point_cloud_->points[j].ring >= last_surf_point_cloud_->points[closest_point_idx].ring &&
                  (point_sqr_dis < second_freature_point_sqr_distance)) {
                second_freature_point_sqr_distance = point_sqr_dis;
                second_feature_point_idx = j;
              }

              // low scan
              if (last_surf_point_cloud_->points[j].ring < last_surf_point_cloud_->points[closest_point_idx].ring &&
                  (point_sqr_dis < third_freature_point_sqr_distance)) {
                third_freature_point_sqr_distance = point_sqr_dis;
                third_feature_point_idx = j;
              }
            }
          }

          if (second_feature_point_idx >= 0 && third_feature_point_idx >= 0) {
            Eigen::Vector3d curr_point(pcl_surf_flat_points_ptr->points[i].x, pcl_surf_flat_points_ptr->points[i].y,
                                       pcl_surf_flat_points_ptr->points[i].z);
            Eigen::Vector3d last_point_a(last_surf_point_cloud_->points[second_feature_point_idx].x,
                                         last_surf_point_cloud_->points[second_feature_point_idx].y,
                                         last_surf_point_cloud_->points[second_feature_point_idx].z);
            Eigen::Vector3d last_point_b(last_surf_point_cloud_->points[second_feature_point_idx].x,
                                         last_surf_point_cloud_->points[second_feature_point_idx].y,
                                         last_surf_point_cloud_->points[second_feature_point_idx].z);
            Eigen::Vector3d last_point_c(last_surf_point_cloud_->points[third_feature_point_idx].x,
                                         last_surf_point_cloud_->points[third_feature_point_idx].y,
                                         last_surf_point_cloud_->points[third_feature_point_idx].z);

            ceres::CostFunction* cost_function = LidarPlaneFactor::Create(curr_point, last_point_a, last_point_b, last_point_c);
            problem.AddResidualBlock(cost_function, loss_function, quaternion_.data(), translation_.data());
            plane_correspondence_number++;
          }
        }

        RCLCPP_INFO(this->get_logger(), "data association cost %.3f ms", search_tic_tok.toc() / 1e6);

        if ((corner_correspondence_number + plane_correspondence_number) < 10) {
          printf("less correspondence! *************************************************\n");
        }

        TicToc solver_tic_tok;
        ceres::Solver::Options options;
        options.linear_solver_type = ceres::DENSE_QR;
        options.max_num_iterations = 4;
        options.minimizer_progress_to_stdout = false;
        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);
        RCLCPP_INFO(this->get_logger(), "solver cost %.3f ms", solver_tic_tok.toc() / 1e6);
      }
      RCLCPP_INFO(this->get_logger(), "optimization twice cost %.3f ms", tic_tok.toc() / 1e6);

      // Using the latest computed pose increment, update the pose from the previous frame to obtain the pose for the current frame. Note that the
      // poses mentioned here refer to the poses in the world coordinate system.
      t_w_curr_ = t_w_curr_ + q_w_curr_ * t_last_curr;
      q_w_curr_ = q_w_curr_ * q_last_curr;
    }
    // pub
    TicToc pub_tic_tok;

    // TODO
    // publish odometry
    nav_msgs::msg::Odometry odometry;
    odometry.header.frame_id = "init";
    odometry.child_frame_id = "lidar";
    odometry.header.stamp.sec = curr_lidar_points_utc_timestamp / 1000000000UL;
    odometry.header.stamp.nanosec = curr_lidar_points_utc_timestamp % 1000000000UL;
    odometry.pose.pose.orientation.x = q_w_curr_.x();
    odometry.pose.pose.orientation.y = q_w_curr_.y();
    odometry.pose.pose.orientation.z = q_w_curr_.z();
    odometry.pose.pose.orientation.w = q_w_curr_.w();
    odometry.pose.pose.position.x = t_w_curr_.x();
    odometry.pose.pose.position.y = t_w_curr_.y();
    odometry.pose.pose.position.z = t_w_curr_.z();
    lidar_odom_to_init_pub_ptr_->publish(odometry);

    // publish geometry
    geometry_msgs::msg::PoseStamped pose;
    pose.header = odometry.header;
    pose.pose = odometry.pose.pose;
    lidar_path_.poses.push_back(pose);
    lidar_path_.header.frame_id = "lidar";
    lidar_odom_path_pub_ptr_->publish(lidar_path_);

    // kt tree
    last_corner_kd_tree_.setInputCloud(pcl_corner_less_sharp_points_ptr);
    last_surf_kd_tree_.setInputCloud(pcl_surl_less_points_ptr);

    // save last points
    last_corner_point_cloud_ = pcl_corner_less_sharp_points_ptr;
    last_surf_point_cloud_ = pcl_surl_less_points_ptr;

    RCLCPP_INFO(this->get_logger(), "publish cost %.3f ms", pub_tic_tok.toc() / 1e6);
  }
}