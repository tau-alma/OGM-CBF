/**
 * @file voxel_main.c
 * @author Teemu Mökkönen (teemu.mokkonen@tuni.com)
 * @brief This file contains the pure C implementation of parsing pointcloud from a C header and
 *        creating a OVG (Occupancy Voxel Grid) from it.
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
#include "math.h"    

// localinlcudes
#include "gridmap.h"

#ifdef ENABLE_VISUALIZATION
    #pragma message("Visualization enabled")
    #include "utils/render_voxel.h"
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
    int lines = 4;
#endif

int main(int argc, char* argv[]) {
    #ifdef ENABLE_VISUALIZATION
        render_init(argc, argv);
    #endif
    float xy_resolution = 0.5;
    float min_x = -25.0;
    float min_y = -25.0;
    double min_z = -3;
    float max_x = 25.0;
    float max_y = 25.0;
    double max_z = 10.0;

    int xw = (int)round((max_x - min_x) / xy_resolution);
    int yw = (int)round((max_y - min_y) / xy_resolution);
    int zw = (int)round((max_z - min_z) / xy_resolution);

    int N = xw;
    int center_x = (int)round(xw * xy_resolution);
    int center_y = (int)round(yw * xy_resolution);
    int center_z = (int)round(zw * xy_resolution);

    int numPoints = 400001; // Adjust this value as needed

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

    for(int i = 0; i < lines; i++) {
        printf("parsing the file \n");
        LidarData *filteredArray = (LidarData *)malloc(0 * sizeof(LidarData));
        for (int i = 0; i < N; i++) {
            for (int j = 0; j < N; j++) {
                for (int k = 0; k < zw; k++) {
                    //printf("element found");
                    //printf("element at %i, %i, %i \n", i, j, k);
                    map.grid[i][j][k] = log(0.5/0.5); // Initialize to 0.5 (unknown)
                    }
                }
            }   
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
            if ( 0.2 < dist && dist < 25) {
                size++;
                filteredArray = realloc(filteredArray, (size) * sizeof(LidarData));
                filteredArray[size - 1] = LidarPoint;
            }
            pointCount++;
        }
        PointCloud cloud;
        cloud.size = size;
        cloud.cloud = filteredArray;
        printf("cloud points size after filtering %d \n", size);
    
        generate_ray_casting_voxel_grid(cloud, &map, center_cell);
        #ifdef ENABLE_VISUALIZATION
            //render_cloud(cloud_renderer, size, filteredArray);
            // init visualization windows for cloud and map
            voxelGrid = map.grid;
            size_x = map.width;
            size_y = map.height;
            size_z = map.depth;
            display();
            //sleep(1);
        #endif  
        printf("freeing memory");
        free(filteredArray);
    }

    return 0;
}
