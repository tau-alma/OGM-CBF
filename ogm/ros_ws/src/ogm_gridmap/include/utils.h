
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include "gridmap/datatypes.h"
using namespace cv;

struct Quaternion {
    double w, x, y, z;
};

struct Euler {
    double roll, pitch, yaw;
};

struct BoundingBox {
    int x_min;
    int y_min;
    int x_max;
    int y_max;
};

float rosTimeToSeconds(const builtin_interfaces::msg::Time& rosTime) {
        return rosTime.sec + static_cast<double>(rosTime.nanosec) / 1e9;
    }

void saveLidarDataToFile(const PointCloud& cloud, const std::string& filename) {
    // Open the output file in append mode
    std::ofstream outputFile(filename, std::ios::app); // Open file in append mode
    if (!outputFile.is_open()) {
        std::cerr << "Error: Unable to open file '" << filename << "' for writing." << std::endl;
        return; // Exit the function
    }

    // Write data to the file
    for (int i = 0; i < cloud.size; i++) {
        outputFile << "lidar x is " << cloud.cloud[i].x << " and y is " << cloud.cloud[i].y << std::endl;
    }

    // Close the file
    outputFile.close();

    std::cout << "Data appended to file '" << filename << "'" << std::endl;
}

bool withinRange(double value1, double value2, double deviation)
{
  return fabs(value1 - value2) <= deviation;
}


Euler quaternionToEuler(const Quaternion& q) {
    Euler euler;

    // roll (x-axis rotation)
    double sinr_cosp = +2.0 * (q.w * q.x + q.y * q.z);
    double cosr_cosp = +1.0 - 2.0 * (q.x * q.x + q.y * q.y);
    euler.roll = std::atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = +2.0 * (q.w * q.y - q.z * q.x);
    if (std::abs(sinp) >= 1)
        euler.pitch = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        euler.pitch = std::asin(sinp);

    // yaw (z-axis rotation)
    double siny_cosp = +2.0 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = +1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    euler.yaw = std::atan2(siny_cosp, cosy_cosp)  * (180.0 / M_PI);

    return euler;
}


void showImg(Mat img) {
    imshow("Map after inflation", img);
    waitKey(1);
}

void showImg(Mat img, std::string title) {
    imshow(title, img);
    waitKey(1);
}



Mat distanceTransform_fun(Mat img) {

    Mat distanceTransformMat;
    threshold(img, img, 127, 255, THRESH_BINARY);
    distanceTransform(img, distanceTransformMat, DIST_L2, 3);
    normalize(distanceTransformMat, distanceTransformMat, 0, 1.0, NORM_MINMAX);
    imshow("distance transform Gridmap Visualization", distanceTransformMat);
    waitKey(1);
    return distanceTransformMat;

}

Mat GridToImg(float** gridmap, int rows, int cols, bool show=false) {
    // Create a grayscale image to visualize the grid map
    Mat img(rows, cols, CV_8UC1);

    // Find the maximum log probability value in the gridmap

    // Populate the image with scaled values from the gridmap
    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
            // Scale the logarithmic probability value to fit in the range [0, 255]
            float prob = (1.0 - 1/exp(1 + gridmap[i][j]));
            int color = prob * 255;
            // Set the pixel value in the image
            img.at<uchar>(i, j) = static_cast<uchar>(color);
        }
    }
    if (show) {
        showImg(img);
        img = 255 - img;
    }
    return img;
}

void visualizeGridMap(float** gridmap, int rows, int cols) {
    // Create a grayscale image to visualize the grid map
    Mat img(rows, cols, CV_8UC1);

    // Find the maximum log probability value in the gridmap

    // Populate the image with scaled values from the gridmap
    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
            // Scale the logarithmic probability value to fit in the range [0, 255]
            float prob = (1.0 - 1/exp(1 + gridmap[i][j]));
            int color = prob * 255;
            // Set the pixel value in the image
            img.at<uchar>(i, j) = static_cast<uchar>(color);
        }
    }
    // Display the image
    imshow("Log Probability Gridmap Visualization", img);
    waitKey(1);
}

void gridToWorld(int center_x, int center_y, int row, int col, float resolution,  float &wx, float &wy) {
    wx = (row - center_x) * resolution; // move to center of the grid cell and move to world
    wy = (col - center_y) * resolution; // move to center of the grid cell and move to world
    
}

sensor_msgs::msg::Image::ConstPtr cvMatToROSImage(const cv::Mat& cv_image) {
    // Create cv_bridge object
    cv_bridge::CvImage cv_bridge_image;

    // Convert cv::Mat to sensor_msgs::Image
    cv_bridge_image.encoding = sensor_msgs::image_encodings::MONO8;
    cv_bridge_image.image = cv_image;

    // Convert to ROS2 sensor_msgs::ImagePtr
    return cv_bridge_image.toImageMsg();
}

float logToProb(float log_val) {
    return (1.0 - 1/exp(1 + log_val));
}


int logToProbColor(float log_val) {
    return (1.0 - 1/exp(1 + log_val)) * 255;
}

Mat inflateObstacles(OccypancyGrid map, float inflationSize) {
    OccypancyGrid new_map;
    new_map.height = map.height;
    new_map.width = map.width;
    new_map.resolution = map.resolution;
    new_map.min_x = map.min_x;
    new_map.min_y = map.min_y;
    allocateGrid(&new_map);
    for (int i = 0; i < new_map.width; ++i) {
        for (int j = 0; j < new_map.height; ++j) {
            new_map.grid[i][j] = 10; // populate the map to be free
        }
    }
    
    int inflationPixels = static_cast<int>(inflationSize / map.resolution);
    inflationPixels = inflationPixels / 2;
    int num_of_cells_inf = 0;
    for (int x = 0; x < map.width; x++) {
        for (int y = 0; y < map.height; y++) {
            // Check if the current cell is an obstacle
            if (logToProbColor(map.grid[x][y]) < 127) {  // Assuming obstacle cells are marked as 0
                num_of_cells_inf++;
                for (int i = -inflationPixels; i <= inflationPixels; i++) {
                    for (int j = -inflationPixels; j <= inflationPixels; j++) {
                        // Ensure the inflated cell is within the map boundaries
                        if (x + i >= 0 && x + i < map.width &&
                            y + j >= 0 && y + j < map.height) {
                            //std::cout << "inflating to " << x + i << " and " << y + j << std::endl;
                            new_map.grid[x + i][y + j] = -1;  // Inflated cell contains same prob as neighbor
                        }
                        
                    }
                    //std::cout << "\n" << std::endl;
                }
            }
        }
    }

    return GridToImg(new_map.grid, new_map.width, new_map.height);

}   

void copy_matrix(float source[4][4], float destination[4][4]) {
    int i, j;
    for (i = 0; i < 4; i++) {
        for (j = 0; j < 4; j++) {
            destination[i][j] = source[i][j];
        }
    }
}

float calculateHeading(const float stateMatrix[4][4]) {
    float heading = 0.0f;
    float dx = stateMatrix[0][0];
    float dy = stateMatrix[0][1];
    
    const double PI = 3.14159265358979323846;
    if (dx != 0 || dy != 0) {
        heading = std::atan2(dy, dx) + PI; // rotate the axis aor
    }
    

    return fmod((heading + PI), (2 * PI)) - PI;
}


float matrix2yaw(const float stateMatrix[4][4]) {
    float heading = 0.0f;
    float dx = stateMatrix[0][0];
    float dy = stateMatrix[0][1];
    
    const double PI = 3.14159265358979323846;
    if (dx != 0 || dy != 0) {
        heading = std::atan2(dy, dx); // rotate the axis aor
    }
    

    return fmod((heading + PI), (2 * PI)) - PI;
}

BoundingBox findBoundingBox(const Point& center, const Point& p1, const Point& p2, const Point& p3, const Point& p4) {
    BoundingBox bbox;
    bbox.x_min = std::min({center.x, p1.x, p2.x, p3.x, p4.x});
    bbox.y_min = std::min({center.y, p1.y, p2.y, p3.y, p4.y});
    bbox.x_max = std::max({center.x, p1.x, p2.x, p3.x, p4.x});
    bbox.y_max = std::max({center.y, p1.y, p2.y, p3.y, p4.y});
    return bbox;
}

Point rotatePoint(const Point& point, float angle) {
    Point rotated_point;
    rotated_point.x = round(point.x * cos(angle) - point.y * sin(angle));
    rotated_point.y = round(point.x * sin(angle) + point.y * cos(angle));
    return rotated_point;
}

bool cropMap(const OccypancyGrid map, OccypancyGrid* cropped_map, float cur[4][4], Cell current_cell, Cell center_cell) {
    // Calculate the heading
    float heading = calculateHeading(cur);

    // Rotate the points around the center
    Point center = {current_cell.x, current_cell.y};
    Point rotated_p1 = {- cropped_map->width / 2, - cropped_map->height / 2}; // top left corect
    Point rotated_p2 = {+ cropped_map->width / 2, - cropped_map->height / 2};        // Example point 2
    Point rotated_p3 = {- cropped_map->width / 2, + cropped_map->height / 2};
    Point rotated_p4 = {+ cropped_map->width / 2, + cropped_map->height / 2};
    rotated_p1 = rotatePoint(rotated_p1, heading);
    rotated_p2 = rotatePoint(rotated_p2, heading);
    rotated_p3 = rotatePoint(rotated_p3, heading);
    rotated_p4 = rotatePoint(rotated_p4, heading);


    rotated_p1 = {rotated_p1.x + center.x, rotated_p1.y + center.y}; // top left corect
    rotated_p2 = {rotated_p2.x + center.x, rotated_p2.y + center.y}; // Example point 2
    rotated_p3 = {rotated_p3.x + center.x, rotated_p3.y + center.y};
    rotated_p4 = {rotated_p4.x + center.x, rotated_p4.y + center.y};
    // Find the bounding box around rotated points
    BoundingBox bbox = findBoundingBox(center, rotated_p1, rotated_p2, rotated_p3, rotated_p4);


    Point box_min = {rotated_p1.x, rotated_p1.y};
    if (bbox.x_min < 0 || bbox.x_max >= map.width || bbox.y_min < 0 || bbox.y_max >= map.height) {
        std::cout << "Invalid crop region." << std::endl;
        return false;
    }


    std::cout << "heading " << heading << std::endl;
    std::cout << "center x " << center.x << std::endl;
    std::cout << "center y " << center.y << std::endl;
    std::cout << "min x " << bbox.x_min << std::endl;
    std::cout << "max x " << bbox.x_max << std::endl;
    std::cout << "min y " << bbox.y_min << std::endl;
    std::cout << "max y " << bbox.y_max << std::endl; 
    std::cout << "selected box" << box_min.x << std::endl;
    std::cout << "selected box" << box_min.y << std::endl;

    // Crop the map
    for (int x = 0; x < cropped_map->width; x++) {
        for (int y = 0; y < cropped_map->height; y++) {
            if (box_min.x > center_cell.x) {
                cropped_map->grid[x][y] = map.grid[box_min.x + x][box_min.y + y];
            }
        }
    }

    return true;
}
