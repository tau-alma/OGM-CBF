#include "gridmap/gridmap.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <functional>
#include <fstream>
#include "utils.h"
#include "gridmap/utils/matrix.h"
#include <cmath>
#include <gridmap/datatypes.h>

void calc_grid_map_config(const std::vector<LidarData>& arr, float xy_resolution, int* result) {
    float min_x = arr[0].x;
    float max_x = arr[0].x;
    float min_y = arr[0].y;
    float max_y = arr[0].y;

    for (const auto& data : arr) {
        min_x = std::min(min_x, data.x);
        max_x = std::max(max_x, data.x);
        min_y = std::min(min_y, data.y);
        max_y = std::max(max_y, data.y);
    }

    min_x = std::round(min_x - EXTEND_AREA / 2.0);
    min_y = std::round(min_y - EXTEND_AREA / 2.0);
    max_x = std::round(max_x + EXTEND_AREA / 2.0);
    max_y = std::round(max_y + EXTEND_AREA / 2.0);
    int xw = static_cast<int>(std::round((max_x - min_x) / xy_resolution));
    int yw = static_cast<int>(std::round((max_y - min_y) / xy_resolution));

    // Print the grid map dimensions (optional)
    // std::cout << "The grid map is " << xw << "x" << yw << ".\n";

    result[0] = (int)min_x;
    result[1] = (int)min_y;
    result[2] = (int)max_x;
    result[3] = (int)max_y;
    result[4] = xw;
    result[5] = yw;

}


namespace GridMapBase{

class SensorBase : public rclcpp::Node {
public:

    SensorBase() : Node("gridmap_node")
    {
        // ROS2 coms
        map_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("map", 1);

        // parameters
        this->declare_parameter("inflation_radious", 0.5);
        this->declare_parameter("ego_centric_map", false);
        this->declare_parameter("ego_centric_map_size", 200);
        this->declare_parameter("graph_distance", 2.0); // distance to save the graph
        this->declare_parameter("window_size", 0);

        inflation_distance = this->get_parameter("inflation_radious").as_double();
        crop_map = this->get_parameter("ego_centric_map").as_bool();
        cropped_map.height = this->get_parameter("ego_centric_map_size").as_int();
        cropped_map.width = this->get_parameter("ego_centric_map_size").as_int();
        distance_threshold = this->get_parameter("graph_distance").as_double();
        window_size = this->get_parameter("window_size").as_int();
        cropped_map.resolution = map.resolution;
        if (crop_map) {
            allocateGrid(&cropped_map);
        }

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

    bool cropMap(const OccypancyGrid map, OccypancyGrid* cropped_map) {
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


    Mat cropMapCV2(const OccypancyGrid map, OccypancyGrid* cropped_map) {
        // Calculate the heading
        printMatrix(cur);
        float heading = calculateHeading(cur);
        std::cout << "heading " << heading << std::endl;
        Mat img = GridToImg(map.grid, map.width, map.height);
        //Point center = {current_cell.x, current_cell.y};
        Point rotated_p1 = {- cropped_map->width * 2 / 2, - cropped_map->height * 2 / 2}; // top left corect
        cv::Point2f center(static_cast<float>(cropped_map->width * 2 / 2), static_cast<float>(cropped_map->height * 2 / 2));
        cv::Mat rotationMatrix = cv::getRotationMatrix2D(center, heading * (180.0 / M_PI), 1.0);
        img = img(Range(rotated_p1.x + current_cell.x, rotated_p1.x + current_cell.x + cropped_map->width * 2),
                  Range(rotated_p1.y + current_cell.y, rotated_p1.y + current_cell.y + cropped_map->height * 2));
        cv::Mat warpedImage;
        cv::warpAffine(img, warpedImage, rotationMatrix, img.size());
        //showImg(warpedImage);
        rotated_p1 = {- cropped_map->width / 2, - cropped_map->height / 2};
        warpedImage = warpedImage(Range(100, 300),//Range(rotated_p1.x + (cropped_map->width / 2), rotated_p1.x + (cropped_map->width / 2) + cropped_map->width),
                                  Range(100, 300));//Range(rotated_p1.y + (cropped_map->height / 2), rotated_p1.y + (cropped_map->height / 2) + cropped_map->height)); // crop the rotated image
        showImg(warpedImage);
        return warpedImage;
    }



    void publish_map() {

        if (map.width > 0 && map.height > 0) {
            Mat img;
            if (crop_map) {
                current_cell.x = round((cur[0][3] - map.min_x) / map.resolution);  
                current_cell.y = round((cur[1][3] - map.min_y) / map.resolution); 
                img = cropMapCV2(map, &cropped_map);
                
            }     
            //if (inflation_distance != 0.0) {
            //    img = inflateObstacles(map, inflation_distance);
            //}
    //
            //else {
            //    img = GridToImg(map.grid, map.width, map.height);
            //}
            else {
                img = GridToImg(map.grid, map.width, map.height);
            }
       
            map_publisher_->publish(*cvMatToROSImage(img));
        }
    }

    void input(std::vector<LidarData> input_cloud) {
        cloud.size = input_cloud.size();
        cloud.cloud = &input_cloud[0];
    }

    void inputStamped(float stamp, std::vector<LidarData> input_cloud) {
        cloud_stamped.stamp = stamp;
        cloud_stamped.size = input_cloud.size();
        cloud_stamped.cloud = new LidarData[input_cloud.size()];
        for (size_t i = 0; i < input_cloud.size(); ++i) {
            cloud_stamped.cloud[i] = input_cloud[i]; 
        }
        input_populated = true;
    }


    void generate_scan() {
        //saveLidarDataToFile(cloud, "/home/localadmin/lidar_cloud.txt");
        generate_ray_casting_grid_map(cloud, &map, center_cell);
        //inflateObstacles(map, 0.5);
        //free(cloud.cloud);
    }


    void generate_map() {
        free_map(&map); // free pre the prev map
        std::vector<LidarData> lidarDataArray;
        for (const auto& poseScan : PosedScans) {
            //std::cout << "cloud extracted from posed scans" << std::endl;
            PointCloud pointCloud = std::get<1>(poseScan);
            int size = pointCloud.size;
            LidarData* cloud = pointCloud.cloud;

            // Iterate through each LidarData element, copy it, and store in lidarDataArray
            for (int i = 0; i < size; i++) {
                if (abs(cloud[i].x) > 10000 && abs(cloud[i].y) > 10000) {
                    continue; // skip corrupted values
                }
                if (std::hypotf(cloud[i].x, cloud[i].y < 50)) {
                    LidarData newLidarData = cloud[i]; // Copy LidarData element
                    lidarDataArray.push_back(newLidarData); // Store the copied LidarData
                }
            }
        }
        PointCloud temp;
        temp.cloud = &lidarDataArray[0];
        temp.size = lidarDataArray.size();
        //saveLidarDataToFile(temp, "/home/teemu/lidar_cloud.txt");
        
        int result[6];
        calc_grid_map_config(lidarDataArray, map.resolution, result);
        map.min_x = result[0];
        map.min_y = result[1];
        map.width = result[4];
        map.height = result[5];
        if (allocateGrid(&map)) { // allocate the new grid
            clear_map(&map); // init to unkown 
            for (const auto& poseScan : PosedScans) {
                center_cell.x = (int)round((std::get<0>(poseScan).x - map.min_x) / map.resolution);
                center_cell.y = (int)round((std::get<0>(poseScan).y - map.min_y) / map.resolution);
                PointCloud cloud = std::get<1>(poseScan);

                //saveLidarDataToFile(cloud, "/home/teemu/lidar_cloud.txt");
                generate_ray_casting_grid_map(cloud, &map, center_cell);  
            }  
        }
    }

    void save_state(float stamp, float x, float y, float yaw) {
        // odom from start, plus the latest scan to posed scan -> contains the transformed pose also
        float cur_state_matrix[4][4];
        state_matrix(x, y, yaw, cur_state_matrix);
        multiplication(init_matrix, cur_state_matrix, cur); // odometry from beginning
        

        if (input_populated) {
            if (std::hypot(prev[0][3] - cur[0][3], prev[1][3] - cur[1][3]) > distance_threshold) { // proceed to store map if the distance moved is over 0.5 m
                if (stamp == cloud_stamped.stamp) { // deviation allowed -> 0.5 NOTE: later on interpolate/extrapolate to match the cloud
                    PointCloud transformed_cloud; 
                    transformStampedCloud(cur, cloud_stamped, &transformed_cloud);
                    //if (window_size != 0 && PosedScans.size() > window_size) {
                    //    delete std::get<1>(PosedScans.at(0)).cloud; // free the memory from the cloud
                    //    PosedScans.erase(PosedScans.begin()); // erase the entry
                    //}
                    Pose position;
                    position.x = cur[0][3];
                    position.y = cur[1][3];
                    PosedScans.push_back(std::make_tuple(position, transformed_cloud));
                    copy_matrix(cur, prev);
                    printMatrix(cur);
                    //delete transformed_cloud.cloud; // free allocated memory from the input cloud
                }

                else {
                    std::cout << "input cloud not in synch with state" << std::endl;
                }
            }
        }

    }

    void init_state(float x, float y, float yaw) {
        float init_matrix_rot[4][4];
        float init_matrix_trans[4][4];
        float init_matrix_rot_inv[4][4];
        float init_matrix_trans_inv[4][4];
        state_matrix(0.0, 0.0, yaw, init_matrix_rot);
        state_matrix(x, y, 0.0, init_matrix_trans);
        rowOperations(init_matrix_rot, init_matrix_rot_inv);
        rowOperations(init_matrix_trans, init_matrix_trans_inv);
        multiplication(init_matrix_rot_inv, init_matrix_trans_inv, init_matrix);
        state_matrix(0.0, 0.0, 0.0, prev);
        state_init = true;
    }

    bool initialized_state() {
        return state_init;
    }


    int numberOfSubmaps() {
        return PosedScans.size();
    }

    void init_center_cell(Cell center) {
        center_cell = center;
    }

    void init_map() {
        allocateGrid(&map);
    }
    
    int get_window_size() {
        return window_size;
    }

    Cell center_cell;
    OccypancyGrid map;

private:

    void transformStampedCloud(const float transform[4][4], const PointCloudStamped src, PointCloud * transformed_cloud) {
        PointCloud target;
        PointCloud src_t;
        src_t.cloud = src.cloud;
        src_t.size = src.size;
        transformCloud(transform, src_t, &target);
        transformed_cloud->cloud = target.cloud;
        transformed_cloud->size = target.size;
    }

    void transformCloud(const float transform[4][4], const PointCloud src, PointCloud* transformed_cloud) {
        transformed_cloud->cloud = new LidarData[src.size];
        transformed_cloud->size = src.size;
        for (int i = 0; i < transformed_cloud->size; i++){
            transformed_cloud->cloud[i].x = transform[0][3] + transform[0][0] * cloud_stamped.cloud[i].x + (transform[0][1]*cloud_stamped.cloud[i].y);
            transformed_cloud->cloud[i].y = transform[1][3] + transform[1][0] * cloud_stamped.cloud[i].x + (transform[1][1]*cloud_stamped.cloud[i].y);
            transformed_cloud->cloud[i].z = cloud_stamped.cloud[i].z;
            transformed_cloud->cloud[i].intensity = cloud_stamped.cloud[i].intensity;
        } // cloud transform loop
    }

    // private struct and pose graph
    typedef std::tuple<Pose, PointCloud> PoseScanTuple;
    std::vector<PoseScanTuple> PosedScans;
    int window_size = 0;

    // state handling
    bool state_init = false;
    float init_matrix[4][4]; // inverse of the first state
    float prev[4][4];
    float cur[4][4];
    double distance_threshold;
    Cell current_cell;

    // input clouds
    bool input_populated = false;
    PointCloud cloud; // pointer to latest scan
    PointCloudStamped cloud_stamped; // pointer to latest scan

    // map pub
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr map_publisher_;
    double inflation_distance = 0.0;
    bool crop_map;
    OccypancyGrid cropped_map;

}; // class
} // namespace