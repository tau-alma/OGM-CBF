cmake_minimum_required(VERSION 3.8)  # CMake version check
project(GridMap
    VERSION 0.0.1)               # Create project "simple_example"
set(CMAKE_CXX_STANDARD 14)            # Enable c++14 standard

# Add main.cpp file of the project root directory as a source file
set(SOURCE_FILES main.c)
set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

include_directories(${OPENGL_INCLUDE_DIRS} ${GLUT_INCLUDE_DIRS})
include_directories(${PROJECT_SOURCE_DIR}/src/)

# enable and disable the visualization according to the needs
option(ENABLE_VISUALIZATION "Enable visualization" ON)
option(LARGE_DATASET "Enable the large dataset to the header" ON)

# Print messages
message(STATUS "ENABLE_VISUALIZATION: ${ENABLE_VISUALIZATION}")
message(STATUS "LARGE_DATASET: ${LARGE_DATASET}")

if(ENABLE_VISUALIZATION)
    add_definitions(-DENABLE_VISUALIZATION=1)
    message(STATUS "Visualization is enabled.")
    # sdl for visualizing 2d grid
    find_package(SDL2 REQUIRED)
    # opengl/glut for voxel visualization
    find_package(OpenGL REQUIRED)
    find_package(GLUT REQUIRED)
else()
    message(STATUS "Visualization is disabled.")
endif()

if(LARGE_DATASET)
    add_definitions(-DLARGE_DATASET=1)
    add_library(dataset src/datasets/large_dataset.h)
    set_target_properties(dataset PROPERTIES LINKER_LANGUAGE C)
    message(STATUS "Using the large dataset.")
else()
    add_library(dataset src/datasets/dataset.h)
    set_target_properties(dataset PROPERTIES LINKER_LANGUAGE C)
    message(STATUS "Using the regular dataset.")
endif()
# Add executable target with source files listed in SOURCE_FILES variable

# exetables/demo files for usage
add_executable(simple_example examples/C/${SOURCE_FILES})
add_executable(simple_example_voxel examples/C/voxel_main.c)
add_executable(map_builder examples/C++/map_builder.cpp)
add_executable(local_map examples/C++/local_map.cpp)
add_executable(voxel_map examples/C++/voxel_map.cpp)



# Libraries built for gridmap/matrix operations/rendering/parsing
add_library(gridmap src/gridmap.h)
add_library(datatypes src/datatypes.h)

add_library(render src/utils/render.h)
add_library(matrix src/utils/matrix.h)
add_library(render_voxel src/utils/render_voxel.h)
add_library(IO src/utils/io.h)


set_target_properties(gridmap PROPERTIES LINKER_LANGUAGE C)
set_target_properties(datatypes PROPERTIES LINKER_LANGUAGE C)
set_target_properties(render PROPERTIES LINKER_LANGUAGE C)
set_target_properties(matrix PROPERTIES LINKER_LANGUAGE C)
set_target_properties(render_voxel PROPERTIES LINKER_LANGUAGE C)
set_target_properties(IO PROPERTIES LINKER_LANGUAGE CXX)

# link all files

# libraries
target_link_libraries(gridmap datatypes)
target_link_libraries(matrix m)
target_link_libraries(render_voxel ${OPENGL_LIBRARIES} ${GLUT_LIBRARY} -lGL -lGLU -lglut)

# C demos
target_link_libraries(simple_example SDL2 gridmap datatypes m dataset render)
target_link_libraries(simple_example_voxel gridmap datatypes matrix render_voxel m)

# C++ demos
target_link_libraries(map_builder gridmap datatypes render SDL2 matrix IO)
target_link_libraries(local_map gridmap datatypes render SDL2 matrix IO)
target_link_libraries(voxel_map gridmap datatypes matrix render_voxel IO)

include(CMakePackageConfigHelpers)
write_basic_package_version_file(
    "${PROJECT_BINARY_DIR}/gridmapConfigVersion.cmake"
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY AnyNewerVersion
)
configure_package_config_file(
    "${PROJECT_SOURCE_DIR}/cmake/gridmapConfig.cmake.in"
    "${PROJECT_BINARY_DIR}/gridmapConfig.cmake"
    INSTALL_DESTINATION lib/cmake/gridmap
)

install(TARGETS gridmap datatypes
    EXPORT gridmapTargets
    LIBRARY DESTINATION lib COMPONENT Runtime
    ARCHIVE DESTINATION lib COMPONENT Development
    RUNTIME DESTINATION bin COMPONENT Runtime
    PUBLIC_HEADER DESTINATION include COMPONENT Development
    BUNDLE DESTINATION bin COMPONENT Runtime
)
install(EXPORT gridmapTargets DESTINATION lib/cmake/gridmap)
install(FILES "${PROJECT_BINARY_DIR}/gridmapConfigVersion.cmake"
              "${PROJECT_BINARY_DIR}/gridmapConfig.cmake"
        DESTINATION lib/cmake/gridmap)
install(DIRECTORY ${PROJECT_SOURCE_DIR}/src/ DESTINATION include/gridmap)