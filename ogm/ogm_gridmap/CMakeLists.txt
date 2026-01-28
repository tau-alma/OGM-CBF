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
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL 1.10 REQUIRED)
find_package(pcl_ros REQUIRED)

include_directories( ${OpenCV_INCLUDE_DIRS} )




install(DIRECTORY
    include/
    DESTINATION include
  )

add_executable(offset_odom src/main_offset_odom.cpp)
target_include_directories(offset_odom PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(offset_odom
  rclcpp
  std_msgs
  std_srvs
  sensor_msgs
  nav_msgs
  geometry_msgs
  pcl_ros
)
install(TARGETS
offset_odom
  DESTINATION lib/${PROJECT_NAME})

add_executable(scan2pcd src/main_scan2pcd.cpp)
target_include_directories(scan2pcd PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(scan2pcd
	rclcpp
	std_msgs
	std_srvs
	sensor_msgs
	nav_msgs
	geometry_msgs
	pcl_ros
	pcl_conversions
	)
install(TARGETS
scan2pcd
  DESTINATION lib/${PROJECT_NAME})

add_executable(pcd_transformer src/main_pcd_transformer.cpp)
target_include_directories(pcd_transformer PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(pcd_transformer
	rclcpp
	std_msgs
	sensor_msgs
	nav_msgs
	geometry_msgs
	pcl_ros
	pcl_conversions
	)
install(TARGETS
pcd_transformer
  DESTINATION lib/${PROJECT_NAME})

add_executable(occgrid src/main_gridmap.cpp)
target_include_directories(occgrid PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(occgrid
	rclcpp
	std_msgs
	std_srvs
	sensor_msgs
  nav_msgs
	pcl_ros
	pcl_conversions
  OpenCV
  cv_bridge
	)
install(TARGETS
occgrid
  DESTINATION lib/${PROJECT_NAME})

add_executable(elevgrid src/main_elevation_gridmap.cpp)
target_include_directories(elevgrid PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(elevgrid
	rclcpp
	std_msgs
	std_srvs
	sensor_msgs
  nav_msgs
	pcl_ros
	pcl_conversions
  OpenCV
  cv_bridge
	)
install(TARGETS
elevgrid
  DESTINATION lib/${PROJECT_NAME})


ament_export_include_directories(include)

ament_package()

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
