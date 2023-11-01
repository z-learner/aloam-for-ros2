#include "aloam-for-ros2/scan_registration.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ScanRegistration>("lidar40", false, 64));
  rclcpp::shutdown();
}