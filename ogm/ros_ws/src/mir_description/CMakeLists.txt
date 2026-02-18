cmake_minimum_required(VERSION 3.8)
project(mir_gazebo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


set(mir_gazebo_MAJOR_VERSION 0)
set(mir_gazebo_MINOR_VERSION 0)
set(mir_gazebo_PATCH_VERSION 0)
set(mir_gazebo_VERSION
  ${mir_gazebo_MAJOR_VERSION}.${mir_gazebo_MINOR_VERSION}.${mir_gazebo_PATCH_VERSION})

find_package(ament_cmake REQUIRED)


install(DIRECTORY
  rviz
  meshes
  urdf
  launch
  DESTINATION share/mir_gazebo
)

ament_export_include_directories(include)
ament_export_include_directories(share)

ament_package()

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
