/**
 * @file main.c
 * @author Teemu Mökkönen (teemu.mokkonen@tuni.com)
 * @brief This file contains the pure C implementation of parsing pointcloud from a C header and
 *        creating a OGM from it. With the current datasets and the nature that they are in float array, the processing is too fast to
 *        enable visualization without any sleep statements, hence there is a delay in the visualization
 * @version 0.1
 * @date 2024-02-02
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h> 

// localinlcudes
#include "gridmap.h"

#ifdef ENABLE_VISUALIZATION
    #pragma message("Visualization enabled")
    #include "utils/render.h"
#endif

#ifdef LARGE_DATASET
    //printf("Using the large dataset");  
    #pragma message("Using the large dataset")
    int lines = 49;
    #include "datasets/large_dataset.h"
#else
    //printf("Using the small dataset");
    #pragma message("Using the small dataset")
    #include "datasets/dataset.h"
    int lines = 4   ;
#endif

int main() {
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
        void* data_r[6];
        data_r[0] = (void*)window_name_grid;
        data_r[1] = &cloud_window;
        data_r[2] = &cloud_renderer;
        data_r[3] = (void*)window_name;
        data_r[4] = &gridmap_window;
        data_r[5] = &gridmap_renderer;
        
        thread = SDL_CreateThread(threadFunction, "CreateSDLWindowThread", (void*)data_r);
    #endif
    float xy_resolution = 0.5;
    float min_x = -25.0;
    float min_y = -25.0;
    float max_x = 25.0;
    float max_y = 25.0;

    int xw = (int)round((max_x - min_x) / xy_resolution);
    int yw = (int)round((max_y - min_y) / xy_resolution);

    int N = xw;
    int center_x = (int)round(xw * xy_resolution);
    int center_y = (int)round(yw * xy_resolution);

    int numPoints = 400001; // Adjust this value as needed


    // init map
    OccypancyGrid map;
    map.width = xw;
    map.height = yw;
    map.resolution = xy_resolution;
    map.min_x = min_x;
    map.min_y = min_y;
    allocateGrid(&map);
    Cell center_cell;
    center_cell.x = center_x;
    center_cell.y = center_y;

    for(int i = 0; i < lines; i++) {
        LidarData *filteredArray = (LidarData *)malloc(0 * sizeof(LidarData));
        for (int i = 0; i < N; i++) {
            for (int j = 0; j < N; j++) {
                map.grid[i][j] = logf(0.5/0.5); // Initialize to 0.5 (unknown)
                }
            }   
        // Initialize the array to store lidar data points
        LidarData LidarPoint;
        float timestamp = data[i][0];
        int size = 0;
        printf("current stamp %f \n", timestamp);
        int pointCount = 1;
        LidarPoint.timestamp = timestamp; // Set the timestamp
        while (pointCount < numPoints) {
            LidarPoint.x = data[i][pointCount++];
            LidarPoint.y = data[i][pointCount++];
            LidarPoint.z = data[i][pointCount++];
            LidarPoint.intensity = data[i][pointCount];
            

            float dist = sqrtf(powf(LidarPoint.x, 2) + powf(LidarPoint.y, 2));
            if (LidarPoint.z > -1.8 && dist < 25 && LidarPoint.x != 0 && LidarPoint.y != 0) {
                size++;
                filteredArray = realloc(filteredArray, (size) * sizeof(LidarData));
                filteredArray[size - 1] = LidarPoint;
            }
            pointCount++;


        }

        printf("cloud points size after filtering %d \n", size);
        PointCloud cloud;
        cloud.size = size;
        cloud.cloud = filteredArray;
    
        generate_ray_casting_grid_map(cloud, &map, center_cell);
        #ifdef ENABLE_VISUALIZATION
            //render_cloud(cloud_renderer, size, filteredArray);
            int scale = round((1000 * xy_resolution) / (xw * xy_resolution));
            render_gridmap_with_lidar(cloud_renderer, xw, yw, scale, map.grid, 50, 50, size, filteredArray);
            sleep(1);
        #endif  
        free(filteredArray);
        
    }


    // Close the file when done


    return 0;
}
