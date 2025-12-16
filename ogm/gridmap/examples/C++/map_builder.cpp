/**
 * @file voxel_map.cpp
 * @author Teemu Mökkönen (teemu.mokkonen@tuni.com)
 * @brief This file contains the C++ implementation of parsing pointcloud from a C header and
 *        creating a OGM (Occupancy Grid Map) from it, from the input dataset and pose in csvs
 * @version 0.1
 * @date 2024-02-02
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include <iostream>
#include <fstream>
#include <vector>
#include <cstring>
#include <chrono>
#include <fstream>
#include <sstream>
#include <cstdlib>

#include "gridmap.h"
#include "utils/matrix.h"
#include "utils/io.h"

#ifdef ENABLE_VISUALIZATION
// Include visualization-related headers and code
    #include "utils/render.h"
#endif


int main(int argc, char* argv[]) {

    float xy_resolution = 0.5;
    int min_x = -125.0;
    int min_y = -125.0;
    int max_x = 125.0;
    int max_y = 125.0;

    if (argc < 2) {
        std::cout << "Usage: " << argv[0] << " needs to arguments <path_to_lidar_data> <path_to_gt_data" << std::endl;
        return 1; // Return an error code
    }

    std::string pointcloud_path = argv[1];  // Replace with your CSV file's name
    std::string gt_path = argv[2];  // Replace with your CSV file's name
    std::ifstream file(pointcloud_path);
    std::ifstream gt_file(gt_path);


    if (argv[3] != NULL && argc > 2) {
        xy_resolution = std::stod(argv[3]);
        std::cout << "map resolution is: " << xy_resolution << std::endl;
    }

    if (argv[4] != NULL && argc > 3) {
        min_x = -std::stod(argv[4]) / 2;
        min_y = -std::stod(argv[4]) / 2;
        max_x = std::stod(argv[4]) / 2;
        max_y = std::stod(argv[4]) / 2;
        std::cout << "map size is: " << argv[4] << "x" << argv[4] << std::endl;
    }
    // init window if required according 
    #ifdef ENABLE_VISUALIZATION
        // init visualization windows for cloud and map
        SDL_Window* cloud_window = NULL;
        const char* window_name = "SDL Window";
        SDL_Renderer* cloud_renderer = NULL;
        SDL_Thread* thread = NULL;
        SDL_Window* gridmap_window = NULL;
        const char* window_name_grid = "SDL Window";
        SDL_Renderer* gridmap_renderer = NULL;
        SDL_Thread* thread_grid = NULL;
        void* data[6];
        data[0] = (void*)window_name_grid;
        data[1] = &cloud_window;
        data[2] = &cloud_renderer;
        data[3] = (void*)window_name;
        data[4] = &gridmap_window;
        data[5] = &gridmap_renderer;

        thread = SDL_CreateThread(threadFunction, "CreateSDLWindowThread", (void*)data);
    #endif

    std::string line;
    std::string gt_line;
    int xw = (int)round((max_x - min_x) / xy_resolution);
    int yw = (int)round((max_y - min_y) / xy_resolution);

    int N = xw + 1;
    float state[15];

    // read and skip the first lines
    std::getline(file, line); 
    std::getline(file, line);
    std::getline(gt_file, gt_line);
    std::getline(gt_file, gt_line);

    float initial_state[15];
    parse_state(gt_line, initial_state);
    float init_matrix[4][4];
    float cur_state_matrix[4][4];
    float init_matrix_rot[4][4];
    float init_matrix_trans[4][4];
    float init_matrix_rot_inv[4][4];
    float init_matrix_trans_inv[4][4];
    float init_matrix_inv[4][4];
    
    state_matrix(0.0, 0.0, -initial_state[6], init_matrix_rot);
    state_matrix(initial_state[1], -initial_state[2], 0.0, init_matrix_trans);
    
    // init map
    OccypancyGrid map;
    map.width = N;
    map.height = N;
    map.resolution = xy_resolution;
    map.min_x = min_x;
    map.min_y = min_y;
    allocateGrid(&map);
   
    printf("init  matrix rot \n");
    printMatrix(init_matrix_rot);
    printf("init  matrix translation \n");
    printMatrix(init_matrix_trans);
    rowOperations(init_matrix_rot, init_matrix_rot_inv);
    rowOperations(init_matrix_trans, init_matrix_trans_inv);
    multiplication(init_matrix_rot_inv, init_matrix_trans_inv, init_matrix_inv);
    
    printMatrix(init_matrix_inv);

    for (int i = 0; i < N; i++) {
        for (int j = 0; j < N; j++) {
            map.grid[i][j] = log(0.5/0.5); // Initialize to 0.5 (unknown)
            }
        }     

    while (std::getline(file, line) && std::getline(gt_file, gt_line)) {

        /*
        parse GT
        */
        parse_state(gt_line, state);
        std::vector<LidarData> pointCloud = parse_cloud(line);
        if (pointCloud.at(0).timestamp < 2.0) {
            continue;
        }
        state_matrix(state[1], -state[2], -(state[6]), cur_state_matrix);
        printf("cur matrix \n");
        float cur[4][4];
        multiplication(init_matrix_inv, cur_state_matrix, cur);
        printMatrix(cur);
        
        auto begin = std::chrono::high_resolution_clock::now();
        std::cout << pointCloud.size() << std::endl;
        int numPoints = pointCloud.size();
        std::vector<LidarData> filteredCloud;
        for (int i = 0; i < numPoints; i ++) {
            float distance = std::hypot(pointCloud[i].x, pointCloud[i].y);
            if (pointCloud[i].z > -1.8 && 0.2 <= distance && distance < 75){
                LidarData entry;
                entry.x = cur[0][3] + cur[0][0] * pointCloud.at(i).x + (-cur[0][1]*pointCloud.at(i).y);  //  arrays containing the valid array data
                entry.y = cur[1][3] + cur[1][0] * pointCloud.at(i).x + (-cur[1][1]*pointCloud.at(i).y); //  arrays containing the valid array data
                entry.intensity = pointCloud.at(i).intensity; //  normalized intensity for obstacle probability <- normalized since intensity is not valid unit (device dependant)
                filteredCloud.push_back(entry);
            }
        }
        Cell cur_cell;
        cur_cell.x = (int)round((cur[0][3] - min_x) / xy_resolution);
        cur_cell.y = (int)round((cur[1][3] - min_y) / xy_resolution);
        
        printf("current stamp of the cloud is %lf \n", pointCloud[0].timestamp);
        PointCloud cloud;
        cloud.size = filteredCloud.size();
        cloud.cloud = &filteredCloud[0];

        generate_ray_casting_grid_map(cloud, &map, cur_cell);
        #ifdef ENABLE_VISUALIZATION
        // init visualization windows for cloud and map
            render_cloud(cloud_renderer, cloud.size, cloud.cloud);
            int scale = round((1000 * xy_resolution) / (xw * xy_resolution));
            //render_gridmap(gridmap_renderer, xw, yw, scale, occupancy_map, center_x, center_y);
            render_gridmap_with_lidar(cloud_renderer, xw, yw, scale, map.grid, cur_cell.x, cur_cell.y, cloud.size, cloud.cloud);
        #endif

        pointCloud.clear();
        filteredCloud.clear();

        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - begin);

        std::cout << "Processing took this amount of time " << duration.count() << std::endl;
   
    }
    

    file.close();
    gt_file.close();

    #ifdef ENABLE_VISUALIZATION
        SDL_DestroyRenderer(cloud_renderer);
        SDL_DestroyWindow(cloud_window);

        SDL_DestroyRenderer(gridmap_renderer);
        SDL_DestroyWindow(gridmap_window);
        SDL_Quit();
    #endif

    return 0;
}
