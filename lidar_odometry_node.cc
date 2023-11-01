#include "aloam-for-ros2/lidar_odometry.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarOdometry>("lidar40"));
  rclcpp::shutdown();
}