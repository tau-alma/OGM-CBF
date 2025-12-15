#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cmath>

#include <opencv2/opencv.hpp>
using namespace cv;


#include "rclcpp/rclcpp.hpp"
#include <rclcpp/qos.hpp>
#include "sensor_msgs/msg/laser_scan.hpp"

// inherit base for map generation
#include "sensor_base.h"

using namespace std::chrono_literals;

class Gridmap : public GridMapBase::SensorBase
{
  public:
    Gridmap()
    {
      int rows = 200;
      int cols = 200;
      float resolution = 0.05;
      //OccypancyGrid map_init;
      map.width = rows;
      map.height = cols;
      map.resolution = resolution;
      map.min_x = 5;
      map.min_y = 5;
      //Cell center_cell_init;
      center_cell.x = 100;
      center_cell.y = 100;
      init_map();
      init_center_cell(center_cell);


      auto sensor_qos = rclcpp::QoS(rclcpp::SensorDataQoS());
      laserScan_subscribtion_ = this->create_subscription<sensor_msgs::msg::LaserScan>("scan", sensor_qos, std::bind(&Gridmap::scan_callback, this, std::placeholders::_1));
    }

    private:
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laserScan_subscribtion_;

};