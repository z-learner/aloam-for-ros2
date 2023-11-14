#include "aloam-for-ros2/lidar_mapping.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarMapping>("lidar"));
  rclcpp::shutdown();
}