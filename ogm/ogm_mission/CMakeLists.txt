cmake_minimum_required(VERSION 3.8)
project(ogm_mission)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


set(ogm_mission_MAJOR_VERSION 0)
set(ogm_mission_MINOR_VERSION 0)
set(ogm_mission_PATCH_VERSION 0)
set(ogm_mission_VERSION
  ${ogm_mission_MAJOR_VERSION}.${ogm_mission_MINOR_VERSION}.${ogm_mission_PATCH_VERSION})

find_package(ament_cmake REQUIRED)

ament_export_include_directories(include)

ament_package()

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
