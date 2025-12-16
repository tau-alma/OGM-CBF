/**
 * @file gridmap.h
 * @author Teemu Mökkönen (teemu.mokkonen@tuni.com)
 * @brief This is the library file for the gridmap generation from a filtered lidar scan in 2D and 3D
 * @version 0.1
 * @date 2024-02-01
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "datatypes.h"

#define EXTEND_AREA 2
#define OCC -1
#define FREE 10
#define PROB_FREE log(0.40/0.45)
#define PROB_OCC log(0.80/0.20)


/**
 * @brief Calculate the log probalility of the obstacle in the scan point according to the
 *        intensity of the scan (normalized)
 * 
 * @param intensity (Normalized [0, 1])
 * @return float log probability
 */
float occupied_with_intensity(float intensity) {
    return log(intensity / (1.0 - intensity));
}

void clear_map(OccypancyGrid * map_target) {
    for (int i = 0; i < map_target->width; ++i) {
        for (int j = 0; j < map_target->height; ++j) {
                map_target->grid[i][j] = log(0.5/0.5);
        }
    }
}

void free_map(OccypancyGrid *map_target) {
    if (map_target->grid != NULL) {
        for (int i = 0; i < map_target->width; ++i) {
            if (map_target->grid[i] != NULL) {
                free(map_target->grid[i]);
                map_target->grid[i] = NULL;
            }
        }
        free(map_target->grid);
        map_target->grid = NULL;
    }
}

/**
 * @brief Memory allocation to allocate memory to the grid according to the saved width and heigh
 * @param pointer to the OccypancyGrid that has populated width and heigh
 * @return if the allocation was succesful
 */
int allocateGrid(OccypancyGrid *grid) {
    grid->grid = (float **)malloc(sizeof(float*) * grid->width); // allocate memory for each row which is pointer to a pointer
    if (grid->grid == NULL) {
        return 0;
    }
    for (int x = 0; x < grid->width; x++) {
        grid->grid[x] = (float *)malloc(sizeof(float) * grid->height); // now allocate memory block for each row as one dimension array
        if (grid->grid[x] == NULL) {
            return 0;
        }
    }
    return 1;
} 


/**
 * @brief Memory allocation to allocate memory to the grid according to the saved width, heigh, and depth
 * @param pointer to the VoxelGrid that has populated width, height and depth
 * @return if the allocation was succesful
 */
int allocateVoxel(VoxelGrid *grid) {
    grid->grid = (float ***) malloc(sizeof(float**) * grid->width); // allocate memory for each row which is pointer to a pointer
    for (int x = 0; x < grid->width; x++) {
        grid->grid[x] = (float **)malloc(sizeof(float*) * grid->height); // now allocate memory block for each row as one dimension array
        for (int y = 0; y < grid->height; y++) {
            grid->grid[x][y] = (float*)malloc(sizeof(float)* grid->depth);
        }
    }
    return 1;
} 


/**
 * @brief Calculates the configuration of the gridmap based on the lidar scan
 * 
 * @param arr Scan of the curren lidar frame
 * @param xy_resolution desired resolution of the map
 * @param result Resulting configuration array (size of 6)
 * @param cloudsize Size of the input cloud
 */
void calc_grid_map_config(LidarData* arr, float xy_resolution, int* result, int cloudsize) {
    float min_x = arr[0].x;
    float max_x = arr[0].x;
    float min_y = arr[0].y;
    float max_y = arr[0].y;

    for (int i = 0; i < cloudsize; i++) {
        min_x = fmin(min_x, arr[i].x);
        max_x = fmax(max_x, arr[i].x);
        min_y = fmin(min_y, arr[i].y);
        max_y = fmax(max_y, arr[i].y);
    }

    min_x = round(min_x - EXTEND_AREA / 2.0);
    min_y = round(min_y - EXTEND_AREA / 2.0);
    max_x = round(max_x + EXTEND_AREA / 2.0);
    max_y = round(max_y + EXTEND_AREA / 2.0);
    int xw = (int)round((max_x - min_x) / xy_resolution);
    int yw = (int)round((max_y - min_y) / xy_resolution);
    
    //printf("The grid map is %dx%d.\n", xw, yw);
    
    result[0] = (int)min_x;
    result[1] = (int)min_y;
    result[2] = (int)max_x;
    result[3] = (int)max_y;
    result[4] = xw;
    result[5] = yw;
    
}

/**
 * @brief Line drawing algorithm in 3D for voxelgrid 
 * 
 * @param start Start point (lidar frame)
 * @param end End point (scan frame)
 * @param num_of_points number of points in the resulting line
 * @return PointLidar Returned points generated
 */
PointLidar* bresenham3D(PointLidar start, PointLidar end, int *num_of_points) {
    PointLidar *points = (PointLidar*)malloc(0 * sizeof(PointLidar));
    int dx = end.x - start.x;
    int dy = end.y - start.y;
    int dz = end.z - start.z;
    int xs;
    int ys;
    int zs;
    if (end.x > start.x)
        xs = 1;
    else
        xs = -1;
    if (end.y > start.y)
        ys = 1;
    else
        ys = -1;
    if (end.z > start.z)
        zs = 1;
    else
        zs = -1;


    // Z-driving axis
    int num = 0;
    if (dx >= dy && dx >= dz) {
        int p1 = 2 * dx - dy;
        int p2 = 2 * dz - dy;

        while (start.x != end.x) {
            start.x += xs;
            if (p1 >= 0) {
                start.y += ys;
                p1 -= 2 * dx;
            }
            if (p2 >= 0) {
                start.z += zs;
                p2 -= 2 * dx;
            }
            p1 += 2 * dy;
            p2 += 2 * dz;
            points = (PointLidar*)realloc(points, (num + 1) * sizeof(PointLidar));
            PointLidar coord;
            coord.x = start.x;
            coord.y = start.y;
            coord.z = start.z;
            points[num] = coord;
            num++;
        } // while
    } // if

    else if (dy >= dx && dy >= dz) {
        int p1 = 2 * dx - dy;
        int p2 = 2 * dz - dy;
        while (start.y != end.y) {
            start.y += ys;
            if (p1 >= 0) {
                start.x += xs;
                p1 -= 2 * dy;
            }
            if (p2 >= 0) {
                start.z += zs;
                p2 -= 2 * dy;
            }
            p1 += 2 * dx;
            p2 += 2 * dz;
            points = (PointLidar*)realloc(points, (num + 1) * sizeof(PointLidar));
            PointLidar coord;
            coord.x = start.x;
            coord.y = start.y;
            coord.z = start.z;
            points[num] = coord;
            num++;
        } // while
    } // else if

    else {
        int p1 = 2 * dy - dz;
        int p2 = 2 * dx - dz;
        while (start.z != end.z) {
            start.z += zs;
            if (p1 >= 0) {
                start.y += ys;
                p1 -= 2 * dz;
            }
            if (p2 >= 0) {
                start.x += xs;
                p2 -= 2 * dz;
            }
            p1 += 2 * dy;
            p2 += 2 * dx;
            points = (PointLidar*)realloc(points, (num + 1) * sizeof(PointLidar));
            PointLidar coord;
            coord.x = start.x;
            coord.y = start.y;
            coord.z = start.z;
            points[num] = coord;
            num++;
        } // while
    } // else
  return points;

}

/**
 * @brief Bresenham line drawing function NOTE: https://en.wikipedia.org/wiki/Bresenham%27s_line_algorithm
 * 
 * @param start Start point (lidar frame)
 * @param end End point (scan frame)
 * @param num_of_points number of points in the resulting line
 * @return PointLidar Returned points generated
 */
PointLidar* bresenham(PointLidar start, PointLidar end, int *num_of_points) {
    // Setup initial conditions
    int x1 = start.x;
    int y1 = start.y;
    int x2 = end.x;
    int y2 = end.y;
    int dx = x2 - x1;
    int dy = y2 - y1;
    int is_steep = abs(dy) > abs(dx);
    PointLidar* points = (PointLidar *)malloc(0 * sizeof(PointLidar));
    if (points == NULL) {
            return NULL;
    }


    if (is_steep) {
        // Rotate line
        int temp = x1;
        x1 = y1;
        y1 = temp;
        temp = x2;
        x2 = y2;
        y2 = temp;
    }

    // Swap start and end points if necessary and store swap state
    int swapped = 0;
    if (x1 > x2) {
        int temp = x1;
        x1 = x2;
        x2 = temp;
        temp = y1;
        y1 = y2;
        y2 = temp;
        swapped = 1;
    }

    dx = x2 - x1;
    dy = y2 - y1;
    int error = dx / 2;
    int y_step = (y1 < y2) ? 1 : -1;

    // Iterate over the bounding box generating points between start and end
    int y = y1;
    for (int x = x1; x <= x2; x++) {
        PointLidar coord;
        coord.x = is_steep ? y : x;
        coord.y = is_steep ? x : y;

        // Append the point to the array
        PointLidar* temp = (PointLidar*)realloc(points, (*num_of_points + 1) * sizeof(PointLidar));
        if (temp == NULL) {
            // Reallocation failed, clean up and return NULL
            free(points);
            return NULL;
        }
        points = temp;
        points[*num_of_points] = coord;
        (*num_of_points)++;

        error -= abs(dy);
        if (error < 0) {
            y += y_step;
            error += dx;
        }
    }

    if (swapped) {
        // Reverse the list if the coordinates were swapped
        int i, j;
        for (i = 0, j = *num_of_points - 1; i < j; i++, j--) {
            PointLidar temp = (points)[i];
            (points)[i] = (points)[j];
            (points)[j] = temp;
        }
    }
    return points;
}


/**
 * @brief Casting the pointcloud to 3D grid of fixed size
 * 
 * @param cloud point cloud array (PointCloud)
 * @param grid grid (VoxelGrid) to be filled
 * @param center_cell (Cell) for the current position of the lidar in grid
 */
void generate_ray_casting_voxel_grid(PointCloud cloud, VoxelGrid* grid, Cell center_cell) {
    for (int k = 0; k < cloud.size; k++) {
        if (isnan(cloud.cloud[k].x) && cloud.cloud[k].x < grid->min_x || isnan(cloud.cloud[k].y) && cloud.cloud[k].y < grid->min_y) {
            // Handle NaN values (you can skip or log these points)
            continue;
        }
        int ix = abs((int)round((cloud.cloud[k].x - grid->min_x) / grid->resolution));
        int iy = abs((int)round((cloud.cloud[k].y - grid->min_y) / grid->resolution));
        int iz = abs((int)round((cloud.cloud[k].z - grid->min_z) / grid->resolution));
        if ((abs(ix) < (grid->width - 1)) && abs(iy) < (grid->height - 1) && abs(iz) < (grid->depth - 1)) {
            
            PointLidar LidarPoint;
            PointLidar Lidar;
            PointLidar * points = NULL;
            //Point ** points = (Point **)malloc(cloud_size * sizeof(Point));
            LidarPoint.x = ix;
            LidarPoint.y = iy;
            LidarPoint.z = iz;
            Lidar.x = center_cell.x;
            Lidar.y = center_cell.y;
            Lidar.z = center_cell.z;
            int num_of_points = 0;
            points = bresenham3D(Lidar, LidarPoint, &num_of_points);
            for (int i = 0; i < num_of_points; i++) {
                grid->grid[points[i].x][points[i].y][points[i].z] -= PROB_FREE; // freespace bewteen
                if (grid->grid[points[i].x][points[i].y][points[i].z] > FREE) {grid->grid[points[i].x][points[i].y][points[i].z] = FREE;}
                //printf("cell probability is: %lf \n", occupancy_map[points[i]->x][points[i]->y]);
                
            }
            float occ = occupied_with_intensity(cloud.cloud[k].intensity);
            grid->grid[ix][iy][iz] -= occ; // Occupied area
            if (grid->grid[ix][iy][iz] < OCC) {grid->grid[ix][iy][iz] = OCC;}
            free(points);
            points = NULL;
        }
            
    }
}

/**
 * @brief Casting the pointcloud to 2D grid of fixed size
 * 
 * @param arr point cloud array (LidarData)
 * @param cloud_size Size of the cloud
 * @param xw width of the map
 * @param yw height of the map
 * @param center_x x Center point of the map or the x pose of the lidar in the map
 * @param center_y y Center point of the map or the y pose of the lidar in the map
 * @param min_x Top left x-coordinate of the map or the min value
 * @param min_y Top left y-coordinate of the map or the min value
 * @param occupancy_map Earlier map to fill with new scan entries
 * @param xy_resolution Desired resolution of the map
 */
void generate_ray_casting_grid_map_old(LidarData* arr,  int cloud_size, int xw, int yw, int center_x, int center_y, int min_x, int min_y, float** occupancy_map, float xy_resolution) {
    for (int k = 0; k < cloud_size; k++) {
        if (isnan(arr[k].x) && arr[k].x < min_x || isnan(arr[k].y) && arr[k].y < min_y) {
            // Handle NaN values (you can skip or log these points)
            continue;
        }
        int ix = abs((int)round((arr[k].x - min_x) / xy_resolution));
        int iy = abs((int)round((arr[k].y - min_y) / xy_resolution));
        if ((abs(ix) < (xw - 1)) && abs(iy) < (yw - 1)) {
            printf("ix is %i \n", ix);
            printf("iy is %i \n", iy);
            PointLidar LidarPoint;
            PointLidar Lidar;
            PointLidar * points = NULL;
            //Point ** points = (Point **)malloc(cloud_size * sizeof(Point));
            LidarPoint.x = ix;
            LidarPoint.y = iy;
            Lidar.x = center_x;
            Lidar.y = center_y;
            int num_of_points = 0;
            points = bresenham(Lidar, LidarPoint, &num_of_points);
            for (int i = 0; i < num_of_points; i++) {
                occupancy_map[points[i].x][points[i].y] -= PROB_FREE; // freespace bewteen
                if (occupancy_map[points[i].x][points[i].y] > FREE) {occupancy_map[points[i].x][points[i].y] = FREE;}
                //printf("cell probability is: %lf \n", occupancy_map[points[i]->x][points[i]->y]);
                
            }
            float occ = occupied_with_intensity(arr[k].intensity);
            occupancy_map[ix][iy] -= occ; // Occupied area
            //occupancy_map[ix][iy - 1] -= occ; // Extend the occupied area
            //occupancy_map[ix - 1][iy] -= occ; // Extend the occupied area
            //occupancy_map[ix - 1][iy - 1] -= occ; // Extend the occupied area
            if (occupancy_map[ix][iy] < OCC) {occupancy_map[ix][iy] = OCC;}
            //if (occupancy_map[ix][iy - 1] < OCC) {occupancy_map[ix][iy - 1] = OCC;}
            //if (occupancy_map[ix - 1][iy] < OCC) {occupancy_map[ix - 1][iy] = OCC;}
            //if (occupancy_map[ix - 1][iy - 1] < OCC) {occupancy_map[ix - 1][iy - 1] = OCC;}

            //for (int i = 0; i < num_of_points; i++) {
            //    free(points[i]);
            //}
            free(points);
            points = NULL;
        }
            
    }
}


/**
 * @brief Casting the pointcloud to 2D grid of fixed size
 * 
 * @param cloud point cloud array (PointCloud)
 * @param grid grid (OccypancyGrid) to be filled
 * @param center_cell (Cell) for the current position of the lidar in grid
 */
void generate_ray_casting_grid_map(PointCloud cloud, OccypancyGrid* grid, Cell center_cell) {
    for (int k = 0; k < cloud.size; k++) {
        if (isnan(cloud.cloud[k].x) || isnan(cloud.cloud[k].y)) {
            // Handle NaN values (you can skip or log these points)
            continue;
        }
        
        if (abs(cloud.cloud[k].x) > 10000 || abs(cloud.cloud[k].y) > 10000) {
            continue; // skip corrupted values
        }
        int ix = abs((int)round((cloud.cloud[k].x - grid->min_x) / grid->resolution));
        int iy = abs((int)round((cloud.cloud[k].y - grid->min_y) / grid->resolution));
        if ((abs(ix) < (grid->width - 1)) && abs(iy) < (grid->height - 1)) {
            //printf("point x is %f, and y is %f \n", cloud.cloud[k].x, cloud.cloud[k].y);
            //printf("point ix is %i, and iy is %i \n", ix, iy);
            PointLidar LidarPoint;
            PointLidar Lidar;
            PointLidar * points = NULL;
            LidarPoint.x = ix;
            LidarPoint.y = iy;
            Lidar.x = center_cell.x;
            Lidar.y = center_cell.y;
            int num_of_points = 0;
            points = bresenham(Lidar, LidarPoint, &num_of_points);
            for (int i = 0; i < num_of_points; i++) {
                grid->grid[points[i].x][points[i].y] -= PROB_FREE; // freespace bewteen
                if (grid->grid[points[i].x][points[i].y] > FREE) {grid->grid[points[i].x][points[i].y] = FREE;}
                
            }
            float occ = occupied_with_intensity(cloud.cloud[k].intensity);
            grid->grid[ix][iy] -= occ; // Occupied area
            grid->grid[ix][iy - 1] -= occ; // Extend the occupied area
            grid->grid[ix - 1][iy] -= occ; // Extend the occupied area
            grid->grid[ix - 1][iy - 1] -= occ; // Extend the occupied area
            if (grid->grid[ix][iy] < OCC) {grid->grid[ix][iy] = OCC;}
            if (grid->grid[ix][iy - 1] < OCC) {grid->grid[ix][iy - 1] = OCC;}
            if (grid->grid[ix - 1][iy] < OCC) {grid->grid[ix - 1][iy] = OCC;}
            if (grid->grid[ix - 1][iy - 1] < OCC) {grid->grid[ix - 1][iy - 1] = OCC;}

            //for (int i = 0; i < num_of_points; i++) {
            //    free(points[i]);
            //}
            free(points);
            points = NULL;
        }
            
    }
}