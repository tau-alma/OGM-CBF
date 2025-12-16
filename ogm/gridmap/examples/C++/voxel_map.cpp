/**
 * @file voxel_map.cpp
 * @author Teemu Mökkönen (teemu.mokkonen@tuni.com)
 * @brief This file contains the C++ implementation of parsing pointcloud from a C header and
 *        creating a OVG (Occupancy Voxel Grid) from it, from the input dataset in csv
 * @version 0.1
 * @date 2024-02-02
 * 
 * @copyright Copyright (c) 2024
 * 
 */


#include <iostream>
#include <vector>
#include <cstring>
#include <chrono>
#include <sstream>
#include <cstdlib>
#include "math.h"
#include "utils/io.h"

#include "gridmap.h"

#ifdef ENABLE_VISUALIZATION
// Include visualization-related headers and code
   #include "utils/render_voxel.h"
#endif

int main(int argc, char* argv[]) {

    double xy_resolution = 0.5;
    double min_x = -25.0;
    double min_y = -25.0;
    double min_z = -3;
    double max_x = 25.0;
    double max_y = 25.0;
    double max_z = 10;

    if (argc < 2) {
        std::cout << "Usage: " << argv[0] << " needs to arguments <path_to_lidar_data> " << std::endl;
        return 1; // Return an error code
    }

    std::string pointcloud_path = argv[1];  // Replace with your CSV file's name
    std::ifstream file(pointcloud_path);

    // init window if required according 
    #ifdef ENABLE_VISUALIZATION
        render_init(argc, argv);
    #endif

    std::vector<LidarData> filteredCloud;
    std::string line;
    std::string gt_line;
    int xw = (int)round((max_x - min_x) / xy_resolution);
    int yw = (int)round((max_y - min_y) / xy_resolution);
    int zw = (int)round((max_z - min_z) / xy_resolution);

    int N = xw;
    int center_x = (int)round(xw * xy_resolution);
    int center_y = (int)round(yw * xy_resolution);
    int center_z = (int)round(zw * xy_resolution);
        

    // read and skip the first lines
    std::getline(file, line); 
    std::getline(file, line);
    
    // init map
    VoxelGrid map;
    map.width = xw;
    map.height = yw;
    map.depth = zw;
    map.resolution = xy_resolution;
    map.min_x = min_x;
    map.min_y = min_y;
    map.min_z = min_z;
    allocateVoxel(&map);
    Cell center_cell;
    center_cell.x = center_x;
    center_cell.y = center_y;
    center_cell.z = center_z;


    while (std::getline(file, line)) {
        for (int i = 0; i < N; i++) {
            for (int j = 0; j < N; j++) {
                for (int k = 0; k < zw; k++) {
                    //printf("element found");
                    //printf("element at %i, %i, %i \n", i, j, k);
                    map.grid[i][j][k] = log(0.5/0.5); // Initialize to 0.5 (unknown)
                    }
                }
            }     
        /*
        parse GT
        */
        std::vector<LidarData> pointCloud = parse_cloud(line);
        if (pointCloud.at(0).timestamp < 2.0) {
            continue;
        }
        for (int i = 0; i < pointCloud.size(); i ++) {
            float distance = std::hypot(pointCloud[i].x, pointCloud[i].y);
            if (0.2 <= distance && distance <= 25){
                filteredCloud.push_back(pointCloud[i]);
            }
        }
        std::cout << filteredCloud.size() << std::endl;

        printf("current stamp of the cloud is %lf \n", pointCloud[0].timestamp);
        //occupancy_map = allocateGrid(occupancy_map, xw, yw);

        PointCloud cloud;
        cloud.size = filteredCloud.size();
        cloud.cloud = &filteredCloud[0];

        generate_ray_casting_voxel_grid(cloud, &map, center_cell);
        #ifdef ENABLE_VISUALIZATION
        // init visualization windows for cloud and map
        voxelGrid = map.grid;
        size_x = xw;
        size_y = yw;
        size_z = zw;
        display();
            
        #endif

        pointCloud.clear();
        filteredCloud.clear();        
        
    }
    

    file.close();

    #ifdef ENABLE_VISUALIZATION
    #endif

    return 0;
}
