#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cmath>

#include <opencv2/opencv.hpp>
using namespace cv;


#include "rclcpp/rclcpp.hpp"
#include <rclcpp/qos.hpp>
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "nav_msgs/msg/odometry.hpp"

// inherit base for map generation
#include "sensor_base.h"


using namespace std::chrono_literals;

class Gridmap : public GridMapBase::SensorBase
{
  public:
    Gridmap()
    {
      int rows = 0;//= 400;
      int cols = 0;//= 400;
      float resolution = 0.2;
      //OncyGrid map_init;
      map.width = rows;
      map.height = cols;
      map.resolution = resolution;
      map.min_x = 10;
      map.min_y = 10;
      //Cell center_cell_init;
      center_cell.x = 200;
      center_cell.y = 200;
      //init_map();
      //init_center_cell(center_cell);
      //auto my_callback_group = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
      //rclcpp::SubscriptionOptions options;
      //options.callback_group = my_callback_group;

      auto sensor_qos = rclcpp::QoS(rclcpp::SensorDataQoS());
      pc2_subscribtion_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("/carla/ego_vehicle/lidar",
                                                                                  sensor_qos,
                                                                                  std::bind(&Gridmap::pointcloud_callback, this, std::placeholders::_1));

      state_subscribtion_ = this->create_subscription<nav_msgs::msg::Odometry>("/carla/ego_vehicle/odometry",
                                                                                sensor_qos,
                                                                                std::bind(&Gridmap::state_callback, this, std::placeholders::_1));
      timer_ = this->create_wall_timer(
          500ms, std::bind(&Gridmap::map_build_callback, this)); // build map every two seconds
      

    }

    private:
    bool scanning = false;
    int last_map_size = 0;
    void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void state_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void map_build_callback();
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pc2_subscribtion_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr state_subscribtion_;
    rclcpp::TimerBase::SharedPtr timer_;


};
