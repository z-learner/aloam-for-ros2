#ifndef ROS2_WS_ALOAM_LIDAR_FACTOR
#define ROS2_WS_ALOAM_LIDAR_FACTOR

#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <eigen3/Eigen/Dense>

class LidarEdgeFactor {
 public:
  LidarEdgeFactor(const Eigen::Vector3d& curr_point, const Eigen::Vector3d& last_point_a, const Eigen::Vector3d& last_point_b)
      : curr_point_(curr_point), last_point_a_(last_point_a), last_point_b_(last_point_b) {}
  ~LidarEdgeFactor() = default;

  template <typename T>
  bool operator()(const T* q, const T* t, T* residual) const {
    Eigen::Matrix<T, 3, 1> cp{T{curr_point_.x()}, T{curr_point_.y()}, T{curr_point_.z()}};
    Eigen::Matrix<T, 3, 1> lpa{T{last_point_a_.x()}, T{last_point_a_.y()}, T{last_point_a_.z()}};
    Eigen::Matrix<T, 3, 1> lpb{T{last_point_b_.x()}, T{last_point_b_.y()}, T{last_point_b_.z()}};

    Eigen::Quaternion<T> q_last_curr{q[3], q[0], q[1], q[2]};
    Eigen::Quaternion<T> q_identity{T{1}, T{0}, T{0}, T{0}};
    Eigen::Matrix<T, 3, 1> t_last_curr{t[0], t[1], t[2]};

    Eigen::Matrix<T, 3, 1> lp = q_last_curr * cp + t_last_curr;

    Eigen::Matrix<T, 3, 1> nu = (lp - lpa).cross(lp - lpb);
    Eigen::Matrix<T, 3, 1> de = lpa - lpb;

    residual[0] = nu.x() / de.norm();
    residual[1] = nu.y() / de.norm();
    residual[2] = nu.z() / de.norm();
    return true;
  }

  static ceres::CostFunction* Create(const Eigen::Vector3d& curr_point, const Eigen::Vector3d& last_point_a, const Eigen::Vector3d& last_point_b) {
    return (new ceres::AutoDiffCostFunction<LidarEdgeFactor, 3, 4, 3>(new LidarEdgeFactor(curr_point, last_point_a, last_point_b)));
  }

 private:
  const Eigen::Vector3d curr_point_;
  const Eigen::Vector3d last_point_a_;
  const Eigen::Vector3d last_point_b_;
};

class LidarPlaneFactor {
 public:
  LidarPlaneFactor(const Eigen::Vector3d& curr_point, const Eigen::Vector3d& last_point_a, const Eigen::Vector3d& last_point_b,
                   const Eigen::Vector3d& last_point_c)
      : curr_point_(curr_point), last_point_a_(last_point_a), last_point_b_(last_point_b), last_point_c_(last_point_c) {
    last_norm_abc_ = (last_point_a_ - last_point_b_).cross(last_point_a_ - last_point_c_);
    last_norm_abc_.normalize();
  }
  ~LidarPlaneFactor() = default;

  template <typename T>
  bool operator()(const T* q, const T* t, T* residual) const {
    Eigen::Matrix<T, 3, 1> cp{T{curr_point_.x()}, T{curr_point_.y()}, T{curr_point_.z()}};
    Eigen::Matrix<T, 3, 1> lpa{T{last_point_a_.x()}, T{last_point_a_.y()}, T{last_point_a_.z()}};

    Eigen::Matrix<T, 3, 1> norm_abc{T{last_norm_abc_.x()}, T{last_norm_abc_.y()}, T{last_norm_abc_.z()}};

    Eigen::Quaternion<T> q_last_curr{q[3], q[0], q[1], q[2]};
    Eigen::Matrix<T, 3, 1> t_last_curr{t[0], t[1], t[2]};

    Eigen::Matrix<T, 3, 1> lp;
    lp = q_last_curr * cp + t_last_curr;

    residual[0] = (lp - lpa).dot(norm_abc);
    return true;
  }

  static ceres::CostFunction* Create(const Eigen::Vector3d& curr_point, const Eigen::Vector3d& last_point_a, const Eigen::Vector3d& last_point_b,
                                     const Eigen::Vector3d& last_point_c) {
    return (new ceres::AutoDiffCostFunction<LidarPlaneFactor, 1, 4, 3>(new LidarPlaneFactor(curr_point, last_point_a, last_point_b, last_point_c)));
  }

 private:
  const Eigen::Vector3d curr_point_;
  const Eigen::Vector3d last_point_a_;
  const Eigen::Vector3d last_point_b_;
  const Eigen::Vector3d last_point_c_;
  Eigen::Vector3d last_norm_abc_;
};

#endif