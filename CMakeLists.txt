cmake_minimum_required(VERSION 3.20)
project(aloam-for-ros2)


set(CMAKE_BUILD_TYPE "Release")

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(PCL REQUIRED)
find_package(fmt REQUIRED)
find_package(Ceres REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)


# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
ament_export_dependencies(sensor_msgs)
ament_export_dependencies(nav_msgs)
ament_export_dependencies(pcl_conversions)
ament_export_dependencies(rcutils)
ament_export_dependencies(rcl_interfaces)
ament_export_dependencies(pcl_conversions)

include_directories(include 
                    ${ament_INCLUDE_DIRS} 
                    ${PCL_INCLUDE_DIRS} 
                    ${nav_msgs_INCLUDE_DIRS}
                    ${sensor_msgs_INCLUDE_DIRS})



file(GLOB_RECURSE SOURCES src/*.cc src/tool/*.cc)
add_library(a_loam ${SOURCES})
target_link_libraries(a_loam ${PCL_LIBRARIES} fmt ${CERES_LIBRARIES})
ament_target_dependencies(a_loam pcl_conversions rcl_interfaces rclcpp rcutils sensor_msgs std_msgs nav_msgs tf2 tf2_ros)


add_executable(kitti_helper_node kitti_helper_node.cc)
target_link_libraries(kitti_helper_node a_loam )

add_executable(scan_registration_node scan_registration_node.cc)
target_link_libraries(scan_registration_node a_loam )


add_executable(lidar_odometry_node lidar_odometry_node.cc)
target_link_libraries(lidar_odometry_node a_loam )

add_executable(lidar_mapping_node lidar_mapping_node.cc)
target_link_libraries(lidar_mapping_node a_loam )

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

install(
        TARGETS a_loam
        EXPORT a_loam
        ARCHIVE DESTINATION lib/aloam-for-ros2
        LIBRARY DESTINATION lib/aloam-for-ros2
        RUNTIME DESTINATION lib/aloam-for-ros2
)

install(
        TARGETS kitti_helper_node
        EXPORT kitti_helper_node
        ARCHIVE DESTINATION lib/aloam-for-ros2
        LIBRARY DESTINATION lib/aloam-for-ros2
        RUNTIME DESTINATION lib/aloam-for-ros2
)

install(
        TARGETS scan_registration_node
        EXPORT scan_registration_node
        ARCHIVE DESTINATION lib/aloam-for-ros2
        LIBRARY DESTINATION lib/aloam-for-ros2
        RUNTIME DESTINATION lib/aloam-for-ros2
)
install(
        TARGETS lidar_odometry_node
        EXPORT lidar_odometry_node
        ARCHIVE DESTINATION lib/aloam-for-ros2
        LIBRARY DESTINATION lib/aloam-for-ros2
        RUNTIME DESTINATION lib/aloam-for-ros2
)
install(
        TARGETS lidar_mapping_node
        EXPORT lidar_mapping_node
        ARCHIVE DESTINATION lib/aloam-for-ros2
        LIBRARY DESTINATION lib/aloam-for-ros2
        RUNTIME DESTINATION lib/aloam-for-ros2
)

ament_package()
