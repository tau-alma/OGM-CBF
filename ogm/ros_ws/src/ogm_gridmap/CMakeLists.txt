cmake_minimum_required(VERSION 3.8)
project(ogm_gridmap)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


set(ogm_gridmap_MAJOR_VERSION 0)
set(ogm_gridmap_MINOR_VERSION 0)
set(ogm_gridmap_PATCH_VERSION 0)
set(ogm_gridmap_VERSION
  ${ogm_gridmap_MAJOR_VERSION}.${ogm_gridmap_MINOR_VERSION}.${ogm_gridmap_PATCH_VERSION})

find_package(ament_cmake REQUIRED)
find_package(message_filters REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(gridmap REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
include_directories( ${OpenCV_INCLUDE_DIRS} )


install(DIRECTORY
    include/
    DESTINATION include
  )

add_executable(ogm_laserscan src/laserscan_node.cpp)
ament_target_dependencies(ogm_laserscan rclcpp std_msgs sensor_msgs nav_msgs gridmap OpenCV cv_bridge)
target_include_directories(ogm_laserscan PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
install(TARGETS
ogm_laserscan
    DESTINATION lib/${PROJECT_NAME})

add_executable(ogm_pointcloud src/pointcloud_node.cpp)
ament_target_dependencies(ogm_pointcloud rclcpp std_msgs sensor_msgs nav_msgs gridmap OpenCV cv_bridge)
target_include_directories(ogm_pointcloud PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
install(TARGETS
ogm_pointcloud
  DESTINATION lib/${PROJECT_NAME})

add_executable(pose_2_odom src/main_pose_2_odom.cpp)
target_include_directories(pose_2_odom PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(pose_2_odom rclcpp std_msgs sensor_msgs nav_msgs geometry_msgs)
install(TARGETS
pose_2_odom
  DESTINATION lib/${PROJECT_NAME})

add_executable(laserscan_transformer src/main_laserscan_transformer.cpp)
target_include_directories(laserscan_transformer PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(laserscan_transformer rclcpp std_msgs sensor_msgs nav_msgs geometry_msgs message_filters)
install(TARGETS
laserscan_transformer
  DESTINATION lib/${PROJECT_NAME})

ament_export_include_directories(include)

ament_package()

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
