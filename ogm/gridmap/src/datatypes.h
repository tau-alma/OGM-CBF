/**
 * @file datatypes.h
 * @author Teemu Mökkönen (teemu.mokkonen@tuni.com)
 * @brief This file contains the datatypes for the ogm mapping for pointclouds and pose
 * @version 0.1
 * @date 2024-02-01
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#ifndef DATATYPES_H
#define DATATYPES_H

/**
 * @brief datatype for containing one pointcloud point
 * 
 */
typedef struct {
    float timestamp;
    float x;
    float y;
    float z;
    float intensity;
} LidarData;

/**
 * @brief datatype for containing pointcloud
 * 
 */
typedef struct {
    LidarData * cloud;
    int size;
} PointCloud;

typedef struct {
    float stamp;
    LidarData * cloud;
    int size;
} PointCloudStamped;


/**
 * @brief datatype for containing point in space
 * 
 */
typedef struct {
    int x;
    int y;
    int z;
} PointLidar;

typedef struct {
    float x;
    float y;
    float heading;
} Pose;

typedef struct {
    int x;
    int y;
    int z;
} Cell;

/**
 * @brief datatype for containing the occypancy grid
 * 
 */
typedef struct {
    int width;
    int height;
    float ** grid;
    float resolution;
    float min_x;
    float min_y;
} OccypancyGrid;

/**
 * @brief datatype for containing the voxelgrid
 * 
 */
typedef struct {
    int width;
    int height;
    int depth;
    float *** grid;
    float resolution;
    float min_x;
    float min_y;
    float min_z;
} VoxelGrid;

#endif