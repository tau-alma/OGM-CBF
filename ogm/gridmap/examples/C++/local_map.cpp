#include <iostream>
#include <fstream>
#include <vector>
#include <cstring>
#include <chrono>
#include <fstream>
#include <sstream>
#include <cstdlib>

#include "gridmap.h"
#include "utils/io.h"

#ifdef ENABLE_VISUALIZATION
// Include visualization-related headers and code
    #include "utils/render.h"
#endif


int main(int argc, char* argv[]) {

    double xy_resolution = 0.5;
    double min_x = -25.0;
    double min_y = -25.0;
    double max_x = 25.0;
    double max_y = 25.0;

    if (argc < 2) {
        std::cout << "Usage: " << argv[0] << " needs to arguments <path_to_lidar_data> " << std::endl;
        return 1; // Return an error code
    }

    std::string pointcloud_path = argv[1];  // Replace with your CSV file's name
    std::ifstream file(pointcloud_path);

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

    std::vector<LidarData> filteredCloud;
    std::string line;
    std::string gt_line;
    int xw = (int)round((max_x - min_x) / xy_resolution);
    int yw = (int)round((max_y - min_y) / xy_resolution);

    int N = xw;
    int center_x = (int)round(xw * xy_resolution);
    int center_y = (int)round(yw * xy_resolution);
        

    // read and skip the first lines
    std::getline(file, line); 
    std::getline(file, line);
    
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


    while (std::getline(file, line)) {
        for (int i = 0; i < N; i++) {
            for (int j = 0; j < N; j++) {
                map.grid[i][j] = logf(0.5/0.5); // Initialize to 0.5 (unknown)
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
            if (pointCloud[i].z > -1.8 && 0.2 <= distance && distance <= 25){
                filteredCloud.push_back(pointCloud[i]);
            }
        }
        LidarData* filteredCloud_arr = &filteredCloud[0];
        int filteredPoints = filteredCloud.size();
        std::cout << filteredPoints << std::endl;



        printf("current stamp of the cloud is %lf \n", pointCloud[0].timestamp);
        //occupancy_map = allocateGrid(occupancy_map, xw, yw);

        PointCloud cloud;
        cloud.size = filteredPoints;
        cloud.cloud = filteredCloud_arr;

        generate_ray_casting_grid_map(cloud, &map, center_cell);
        #ifdef ENABLE_VISUALIZATION
        // init visualization windows for cloud and map
            LidarData* a = &filteredCloud[0];
            render_cloud(cloud_renderer, filteredCloud.size(), a);
            int scale = round((1000 * xy_resolution) / (xw * xy_resolution));
            //render_gridmap(gridmap_renderer, xw, yw, scale, occupancy_map, center_x, center_y);
            render_gridmap_with_lidar(gridmap_renderer, xw, yw, scale, map.grid, center_x, center_y, filteredCloud.size(), a);
        #endif

        pointCloud.clear();
        filteredCloud.clear();        
        
    }
    

    file.close();

    #ifdef ENABLE_VISUALIZATION
        SDL_DestroyRenderer(cloud_renderer);
        SDL_DestroyWindow(cloud_window);

        SDL_DestroyRenderer(gridmap_renderer);
        SDL_DestroyWindow(gridmap_window);
        SDL_Quit();
    #endif

    writeGridToCSV(map, "test.csv");

    return 0;
}
