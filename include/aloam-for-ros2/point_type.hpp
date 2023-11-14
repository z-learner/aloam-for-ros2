#ifndef ROS2_WS_ALOAM_POINT_TYPE
#define ROS2_WS_ALOAM_POINT_TYPE

#define PCL_NO_PRECOMPILE

#include <pcl/filters/passthrough.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/filters/impl/passthrough.hpp>

struct EIGEN_ALIGN16 PointXYZTIRLL {
  PCL_ADD_POINT4D;
  double timestamp;
  float intensity;
  uint16_t lidar_id;
  uint16_t ring;
  uint32_t label;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZTIRLL, (float, x, x)(float, y, y)(float, z, z)(double, timestamp, timestamp)(float, intensity, intensity)(
                                                     uint16_t, lidar_id, lidar_id)(uint16_t, ring, ring)(uint32_t, label, label))

#endif