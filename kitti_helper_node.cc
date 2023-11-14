#include <fmt/format.h>
#include <math.h>
#include <pcl_conversions/pcl_conversions.h>

#include <array>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <memory>
#include <regex>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "aloam-for-ros2/point_type.hpp"
#include "aloam-for-ros2/tools/cxxopts.hpp"
#include "rclcpp/rclcpp.hpp"

bool PraseBytesToPoint(const std::array<uint8_t, 16>& bytes, PointXYZTIRLL& point, double timestamp) {
  point.x = *reinterpret_cast<const float*>(bytes.data());
  point.y = *reinterpret_cast<const float*>(bytes.data() + 4);
  point.z = *reinterpret_cast<const float*>(bytes.data() + 8);
  point.intensity = *reinterpret_cast<const float*>(bytes.data() + 12);

  float ori = std::atan2(point.y, point.x) * 180 / M_PI;
  ori += (ori < 0) ? 360 : 0;
  point.timestamp = timestamp - 0.1 * (180 - ori) / 360;

  // to to calculate scan id
  float distance = std::sqrt(std::pow(point.x, 2) + std::pow(point.y, 2));

  auto ori_z = std::atan2(point.z, distance) * 180 / M_PI;

  // if (max_ori_z < ori_z) {
  //   max_ori_z = ori_z;
  //   std::cout << "max_ori_z :" << max_ori_z << std::endl;
  // }

  // if (min_ori_z > ori_z) {
  //   min_ori_z = ori_z;
  //   std::cout << "min_ori_z : " << min_ori_z << std::endl;
  // }

  if (ori_z >= -8.83) {
    point.ring = int((2 - ori_z) * 3.0 + 0.5);
  } else {
    point.ring = 64 / 2 + int((-8.83 - ori_z) * 2.0 + 0.5);
  }
  // use [0 50]  > 50 remove outlies
  if (ori_z > 2 || ori_z < -24.33 || point.ring >= 64 || point.ring < 0) {
    return false;
  }

  // std::cout << "x : " << point.x << ", y : " << point.y << ", z : " << point.z << ", ori : " << ori << ", ori_z : " << ori_z
  //           << ", timestamp : " << static_cast<int64_t>(point.timestamp * 1e9) << std::endl;

  return true;
}

pcl::PointCloud<PointXYZTIRLL> GetOneFrameFromFile(const std::string& file_name, double timestamp) {
  static size_t seq = 0;

  std::array<uint8_t, 16> buff;
  std::ifstream file(file_name, std::ios::binary);

  pcl::PointCloud<PointXYZTIRLL> cld;

  if (!file.is_open()) {
    std::cout << "open " << file_name << " failed" << std::endl;
    return cld;
  }

  while (file.read(reinterpret_cast<char*>(buff.data()), buff.size())) {
    PointXYZTIRLL point;
    if (PraseBytesToPoint(buff, point, timestamp)) {
      cld.push_back(point);
    }
  }

  cld.header.frame_id = "lidar";
  cld.header.seq = seq++;
  // us
  cld.header.stamp = static_cast<uint64_t>(timestamp * 1000000);
  return cld;
}

class KittiPublishNode : public rclcpp::Node {
 public:
  KittiPublishNode(const std::string node_namespace) : Node("kitti_pushlish_node"), node_namespace_(node_namespace) {
    dataset_path_ = this->declare_parameter<std::string>(
        "dataset_path", "/root/Code/kitti_dataset/raw_data/2011_10_03_drive_0034_sync/2011_10_03/2011_10_03_drive_0034_sync/");

    points_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(fmt::format("/{}/points", node_namespace), 10);

    // get timestamp file path
    std::string timestamp_path = dataset_path_ + "/velodyne_points/timestamps.txt";
    timestamp_file_ = std::make_shared<std::ifstream>(timestamp_path, std::ifstream::in);
    // parse points timer

    parse_poinst_timer_ = this->create_wall_timer(std::chrono::milliseconds(100), [this]() {
      std::string line;
      if (std::getline(*timestamp_file_, line)) {
        std::tm tm = {};
        std::istringstream ss(line);

        if (ss.fail()) {
          RCLCPP_ERROR(this->get_logger(), "Can't parase timestamp str : %s", line.c_str());
          return;
        }

        ss >> std::get_time(&tm, "%Y-%m-%d %H:%M:%S");

        std::string nanoseconds_str;
        getline(ss, nanoseconds_str, '.');
        getline(ss, nanoseconds_str);

        auto time_c = std::mktime(&tm);
        if (time_c == -1) {
          RCLCPP_ERROR(this->get_logger(), "Can't parase timestamp str : %s", line.c_str());
          return;
        }

        std::int64_t timestamp_nanoseconds = static_cast<std::int64_t>(time_c) * 1000000000;
        timestamp_nanoseconds += std::stoll(nanoseconds_str);

        double timestamp = timestamp_nanoseconds / 1e9;

        // RCLCPP_INFO(this->get_logger(), "line : %s", (line.c_str()));
        // RCLCPP_INFO(this->get_logger(), "timestamp_nanoseconds : %.3f", (timestamp_nanoseconds / 1e9));

        std::stringstream lidar_data_path;
        lidar_data_path << dataset_path_ + "/velodyne_points/data/" << std::setfill('0') << std::setw(10) << line_num_ << ".bin";
        auto pcl = GetOneFrameFromFile(lidar_data_path.str(), timestamp);
        PushlishOneFrame(pcl);
        RCLCPP_INFO(this->get_logger(), "PushlishOneFrame point number : %d", pcl.points.size());

        line_num_++;
      }
    });

    statistic_timer_ = this->create_wall_timer(std::chrono::seconds(5), [this]() {
      static size_t last_line_number = 0;
      size_t line_number = line_num_;
      RCLCPP_INFO(this->get_logger(), "kitti pushlish points frame req : %.3f", (line_number - last_line_number) / 5.0);
      last_line_number = line_number;
    });
  }

  ~KittiPublishNode() = default;

  bool PushlishOneFrame(const pcl::PointCloud<PointXYZTIRLL>& pcl_points) {
    sensor_msgs::msg::PointCloud2 ros2_tmp_msg;
    pcl::toROSMsg(pcl_points, ros2_tmp_msg);
    ros2_tmp_msg.header.frame_id = "lidar";
    ros2_tmp_msg.header.stamp.sec = pcl_points.header.stamp / 1000000;
    ros2_tmp_msg.header.stamp.nanosec = pcl_points.header.stamp % 1000000 * 1000;
    points_cloud_pub_->publish(ros2_tmp_msg);
    return true;
  }

 private:
  std::string node_namespace_{};
  std::string dataset_path_{};
  std::shared_ptr<std::ifstream> timestamp_file_{nullptr};
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr points_cloud_pub_{nullptr};

  // parse time
  rclcpp::TimerBase::SharedPtr parse_poinst_timer_{nullptr};
  rclcpp::TimerBase::SharedPtr statistic_timer_{nullptr};

  std::atomic<size_t> line_num_{0};
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<KittiPublishNode>("lidar"));
  rclcpp::shutdown();
}