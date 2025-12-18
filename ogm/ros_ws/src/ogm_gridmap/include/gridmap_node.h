#ifndef __OGM_GRIDMAP_GRIDMAP_NODE__
#define __OGM_GRIDMAP_GRIDMAP_NODE__

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include "gridmap.h"

class GridmapNode  : public rclcpp::Node
{
  private:
    
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_grid;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_pcd;

    std::string map_frame;

    float cell_size;
    uint32_t height;
    uint32_t width;

    std::shared_ptr<Gridmap> gridmap;

    void publish_grid()
    {
      rclcpp::Time now = this->get_clock()->now();

      nav_msgs::msg::OccupancyGrid msg_out;
	    msg_out.header.stamp = now;
	    msg_out.header.frame_id = this->map_frame;

      msg_out.info.resolution = gridmap->get_cell_size();
      msg_out.info.width = gridmap->get_width();
      msg_out.info.height = gridmap->get_height();
      
      msg_out.info.origin.position.x = gridmap->get_origin_x();
      msg_out.info.origin.position.y = gridmap->get_origin_y();
      msg_out.info.origin.position.z = 0.;
      msg_out.info.origin.orientation.x = 0.;
      msg_out.info.origin.orientation.y = 0.;
      msg_out.info.origin.orientation.z = 0.;
      msg_out.info.origin.orientation.w = 1.;

      msg_out.data = gridmap->report_int8();

      pub_grid->publish(msg_out);

    }

    void callback_pcd(
        const sensor_msgs::msg::PointCloud2::SharedPtr msg_pcd
        )
    {

      pcl::PCLPointCloud2 pcdp;
      pcl_conversions::toPCL(*msg_pcd, pcdp);
      
      pcl::PointCloud<pcl::PointXYZI> pcd;
      pcl::fromPCLPointCloud2(pcdp, pcd);

      gridmap->update(pcd);

      publish_grid();
    }
  
  public:
    GridmapNode() : Node("gridmap")
    {
      map_frame = this->declare_parameter("map_frame", "map_frame");
      RCLCPP_INFO(this->get_logger(), "map_frame: %s", map_frame.c_str());

      cell_size = this->declare_parameter("cell_size", 1.0);
      RCLCPP_INFO(this->get_logger(), "cell_size: %f", cell_size);

      float _height = this->declare_parameter("height", 20.0);
      height = std::ceil(_height / cell_size);
      RCLCPP_INFO(this->get_logger(), "height: %f (%u)", _height, height);

      float _width = this->declare_parameter("width", 20.0);
      width = std::ceil(_width / cell_size);
      RCLCPP_INFO(this->get_logger(), "width: %f (%u)", _width, width);

      float s_target = this->declare_parameter("s_target", .95);
      RCLCPP_INFO(this->get_logger(), "s_target: %f", s_target);

      gridmap = std::make_shared<Gridmap>(Gridmap(height,width,cell_size, s_target));

      pub_grid = this->create_publisher<nav_msgs::msg::OccupancyGrid>("gridmap", 1);

      sub_pcd = this->create_subscription<sensor_msgs::msg::PointCloud2>(
		      "pcd",
		      rclcpp::QoS(rclcpp::SensorDataQoS()),
		      std::bind(&GridmapNode::callback_pcd, this, std::placeholders::_1)
		      );

    }
};


int init_gridmap_node(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<GridmapNode>());
  rclcpp::shutdown();
  return 0;
}

#endif
