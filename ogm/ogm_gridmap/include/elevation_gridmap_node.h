#ifndef __OGM_GRIDMAP_ELEVATION_GRIDMAP_NODE__
#define __OGM_GRIDMAP_ELEVATION_GRIDMAP_NODE__

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "pcl_ros/transforms.hpp"
#include <pcl_conversions/pcl_conversions.h>

#include <cmath>

#include "elevation_gridmap.h"

class ElevationGridmapNode  : public rclcpp::Node
{
  private:

    float vis_z;
    
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_grid;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_map;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_pc2;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom;

    float clearance_x, clearance_y;
    float clearance_thr;

    std::shared_ptr<ElevationGridmap> gridmap;

    void callback_pc2(const sensor_msgs::msg::PointCloud2::SharedPtr msgp)
    {

      pcl::PCLPointCloud2::Ptr pcdp (new pcl::PCLPointCloud2());
      pcl_conversions::toPCL(*msgp,*pcdp);
      pcl::PointCloud<pcl::PointXYZ> xyz;
      pcl::fromPCLPointCloud2(*pcdp, xyz);

      gridmap->update(xyz);

      // get pts
      pcl::PointCloud<pcl::PointXYZI> elevgrid_xyz = gridmap->report_3d();

      // pc2 elev grid msg
      pcl::PCLPointCloud2 elevgrid_pcdp;
      pcl::toPCLPointCloud2(elevgrid_xyz, elevgrid_pcdp);
      sensor_msgs::msg::PointCloud2 elevgrid_msgp;
      pcl_conversions::fromPCL(elevgrid_pcdp,elevgrid_msgp); 
      elevgrid_msgp.header = msgp->header;
      pub_grid->publish(elevgrid_msgp);

      // occgrid msg
      nav_msgs::msg::OccupancyGrid occgrid_msg;
      occgrid_msg.header = msgp->header;

      occgrid_msg.info.resolution = gridmap->get_cellsize();
      occgrid_msg.info.width = gridmap->get_width();
      occgrid_msg.info.height = gridmap->get_height();
      
      occgrid_msg.info.origin.position.x = gridmap->get_origin_x();
      occgrid_msg.info.origin.position.y = gridmap->get_origin_y();
      occgrid_msg.info.origin.position.z = vis_z;
      occgrid_msg.info.origin.orientation.x = 0.;
      occgrid_msg.info.origin.orientation.y = 0.;
      occgrid_msg.info.origin.orientation.z = 0.;
      occgrid_msg.info.origin.orientation.w = 1.;

      occgrid_msg.data = gridmap->report_2d_int8();

      pub_map->publish(occgrid_msg);
    }

    void callback_clearance(const nav_msgs::msg::Odometry::SharedPtr msgo)
    {
      gridmap->update_clearance(
          clearance_x = msgo->pose.pose.position.x,
          clearance_y = msgo->pose.pose.position.y,
          clearance_thr);

    }

  public:
    ElevationGridmapNode() : Node("gridmap")
    {
      float cellsize = this->declare_parameter("cellsize", 0.025);
      RCLCPP_INFO(this->get_logger(), "cellsize: %f", cellsize);

      float _height = this->declare_parameter("height", 20.0);
      uint32_t height = std::ceil(_height / cellsize) ;
      RCLCPP_INFO(this->get_logger(), "height: %f -> %d", _height, height);

      float _width = this->declare_parameter("width", 20.0);
      uint32_t width = std::ceil(_width / cellsize) ;
      RCLCPP_INFO(this->get_logger(), "width: %f -> %d", _width, width);

      float traversable_slope = this->declare_parameter("traversable_slope", 0.78);
      RCLCPP_INFO(this->get_logger(), "traversable_slope: %f", traversable_slope);

      vis_z = this->declare_parameter("vis_z", 0.0);
      RCLCPP_INFO(this->get_logger(), "vis_z: %f", vis_z);

      gridmap = std::make_shared<ElevationGridmap>(ElevationGridmap(
            cellsize,
            height, width,
            traversable_slope));


      clearance_thr = this->declare_parameter("clearance_thr", 0.5);
      RCLCPP_INFO(this->get_logger(), "clearance_thr: %f", clearance_thr);

      pub_grid = this->create_publisher<sensor_msgs::msg::PointCloud2>("elevation_gridmap", 1);
      pub_map = this->create_publisher<nav_msgs::msg::OccupancyGrid>("gridmap", 1);

      sub_pc2 = this->create_subscription<sensor_msgs::msg::PointCloud2>(
          "pcd",
		      rclcpp::QoS(rclcpp::SensorDataQoS()),
          std::bind(&ElevationGridmapNode::callback_pc2, this, std::placeholders::_1)
          );

      sub_odom = this->create_subscription<nav_msgs::msg::Odometry>(
          "clearance_odom",
		      rclcpp::QoS(rclcpp::SensorDataQoS()),
          std::bind(&ElevationGridmapNode::callback_clearance, this, std::placeholders::_1)
          );

    }
};


int init_elevation_gridmap_node(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<ElevationGridmapNode>());
  rclcpp::shutdown();
  return 0;
}

#endif
