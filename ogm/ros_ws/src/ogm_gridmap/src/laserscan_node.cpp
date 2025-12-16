#include "laserscan_node.h"

void Gridmap::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        // handle the message and send it to the gridmap library to generate the gridmap.
        if (msg->header.frame_id == "front_laser_link") {
        RCLCPP_DEBUG(this->get_logger(), "scan received");
        int scans = msg->ranges.size();
        std::vector<LidarData> scans_arr;
        float lidar_to_base[4][4];
        state_matrix(0.4253, 0.2345, 45, lidar_to_base); // from base to lidar on MIR
        //printMatrix(lidar_to_base);
        for (int i = 0; i < scans ; i++) {
          double angle = msg->angle_min + i * msg->angle_increment;
          LidarData point;
          if (msg->ranges.at(i) != NAN) {
            float x = (msg->ranges.at(i) * cos(angle));
            float y = (msg->ranges.at(i) * sin(angle));

            point.x = lidar_to_base[0][3] + lidar_to_base[0][0] * x + lidar_to_base[0][1]*y;
            point.y = lidar_to_base[1][3] + lidar_to_base[1][0] * x + lidar_to_base[1][1]*y;
            point.intensity = 1.0;//msg->intensities.at(i);
            //std::cout << "intensity is: " << point.intensity << std::endl; 
            float dist = hypot(point.x, point.y);
  
            if (0.2 < dist && dist < 4.5) {
                //std::cout << dist << std::endl;
                scans_arr.push_back(point);
            }
            
          }
          
        }
        

      clear_map(&map);
      input(scans_arr);
      generate_scan();
      publish_map();
      }
    }

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Gridmap>());
  rclcpp::shutdown();
  return 0;
}

