#ifndef __OGM_GRIDMAP_ELEVATION_GRIDMAP_NODE__
#define __OGM_GRIDMAP_ELEVATION_GRIDMAP_NODE__

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "pcl_ros/transforms.hpp"
#include <pcl_conversions/pcl_conversions.h>

#include <cmath>
#include <chrono>   
#include <opencv2/opencv.hpp>   
#include <cv_bridge/cv_bridge.h>

#include "elevation_gridmap.h"

using namespace std::chrono_literals;

class ElevationGridmapNode  : public rclcpp::Node
{
  private:

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_elevgrid_real;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_elevgrid_vis;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_occgrid;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_occimg;
    rclcpp::TimerBase::SharedPtr timer_map;

    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr trigger_on;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr trigger_off;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_pc2;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom;

    float occgrid_vis_z;
    std::string map_frame;

    float clearance_x, clearance_y;
    float clearance_thr;
    
    bool do_update;
    bool do_pub_occgrid;
    bool do_pub_occimg;
    bool do_pub_elevgrid_real;
    bool do_pub_elevgrid_vis;

    std::shared_ptr<ElevationGridmap> gridmap;

    void publish_occgrid(rclcpp::Time& now)
    {
      nav_msgs::msg::OccupancyGrid occgrid_msg;
	    occgrid_msg.header.stamp = now;
	    occgrid_msg.header.frame_id = this->map_frame;

      occgrid_msg.info.resolution = gridmap->get_cellsize();
      occgrid_msg.info.width = gridmap->get_width();
      occgrid_msg.info.height = gridmap->get_height();
      
      occgrid_msg.info.origin.position.x = gridmap->get_origin_x();
      occgrid_msg.info.origin.position.y = gridmap->get_origin_y();
      occgrid_msg.info.origin.position.z = occgrid_vis_z;
      occgrid_msg.info.origin.orientation.x = 0.;
      occgrid_msg.info.origin.orientation.y = 0.;
      occgrid_msg.info.origin.orientation.z = 0.;
      occgrid_msg.info.origin.orientation.w = 1.;

      occgrid_msg.data = gridmap->report_2d_int8();

      pub_occgrid->publish(occgrid_msg);
    }

    sensor_msgs::msg::PointCloud2 get_elevgrid_msg(rclcpp::Time& now, int _ground_nbh_size)
    {
      pcl::PointCloud<pcl::PointXYZI> elevgrid_xyz = gridmap->report_3d(_ground_nbh_size);
      
      pcl::PCLPointCloud2 elevgrid_pcdp;
      pcl::toPCLPointCloud2(elevgrid_xyz, elevgrid_pcdp);
      sensor_msgs::msg::PointCloud2 elevgrid_msgp;
      pcl_conversions::fromPCL(elevgrid_pcdp,elevgrid_msgp); 
      
      elevgrid_msgp.header.stamp = now;
	    elevgrid_msgp.header.frame_id = this->map_frame;

      return elevgrid_msgp;
    }

    void publish_elevgrid_real(rclcpp::Time& now)
    {
      sensor_msgs::msg::PointCloud2 elevgrid_msgp = get_elevgrid_msg(now, 0);
      pub_elevgrid_real->publish(elevgrid_msgp);
    }

    void publish_elevgrid_vis(rclcpp::Time& now)
    {
      sensor_msgs::msg::PointCloud2 elevgrid_msgp = get_elevgrid_msg(now, 1);
      pub_elevgrid_vis->publish(elevgrid_msgp);
    }

    void publish_occimg(rclcpp::Time& now)
    {
      std::vector<uint8_t> data = gridmap->report_2d_uint8();
      
      cv::Mat map(
        gridmap->get_height(),
        gridmap->get_width(),
        CV_8U,
        data.data()
        );
      cv::Mat img;
      cv::flip(map, img, 0);

      cv_bridge::CvImage cv_bridge_image;
      cv_bridge_image.encoding = sensor_msgs::image_encodings::MONO8;
      cv_bridge_image.image = img;

      sensor_msgs::msg::Image msg_img = *(cv_bridge_image.toImageMsg());
	    msg_img.header.stamp = now;
	    msg_img.header.frame_id = this->map_frame;

      pub_occimg->publish(msg_img);
    }

    void tick_map()
    {
      rclcpp::Time ts = this->now();

      // pc2 elev grid msg
      if (do_pub_elevgrid_real) publish_elevgrid_real(ts);
      if (do_pub_elevgrid_vis) publish_elevgrid_vis(ts);

      // occgrid
      if (do_pub_occgrid) publish_occgrid(ts);

      // occimg
      if (do_pub_occimg) publish_occimg(ts);
    }

    bool callback_on(
		    const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
		    const std::shared_ptr<std_srvs::srv::Trigger::Response> res)
    {
	    do_update = true;
    }
    bool callback_off(
		    const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
		    const std::shared_ptr<std_srvs::srv::Trigger::Response> res)
    {
	    do_update = false;
    }

    void callback_pc2(const sensor_msgs::msg::PointCloud2::SharedPtr msgp)
    {

      if (do_update)
      {
        pcl::PCLPointCloud2::Ptr pcdp (new pcl::PCLPointCloud2());
        pcl_conversions::toPCL(*msgp,*pcdp);
        pcl::PointCloud<pcl::PointXYZ> xyz;
        pcl::fromPCLPointCloud2(*pcdp, xyz);

        gridmap->update(xyz);
      }
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

      do_update = true;

      map_frame = this->declare_parameter("map_frame", "map_frame");
      RCLCPP_INFO(this->get_logger(), "map_frame: %s", map_frame.c_str());

      do_pub_occgrid = this->declare_parameter("do_pub_occgrid", false);
      RCLCPP_INFO(this->get_logger(), "do_pub_occgrid: %x", do_pub_occgrid);

      occgrid_vis_z = this->declare_parameter("occgrid_vis_z", 0.0);
      RCLCPP_INFO(this->get_logger(), "occgrid_vis_z: %f", occgrid_vis_z);

      do_pub_occimg = this->declare_parameter("do_pub_occimg", false);
      RCLCPP_INFO(this->get_logger(), "do_pub_occimg: %x", do_pub_occimg);

      do_pub_elevgrid_real = this->declare_parameter("do_pub_elevgrid_real", false);
      RCLCPP_INFO(this->get_logger(), "do_pub_elevgrid_real: %x", do_pub_elevgrid_real);

      do_pub_elevgrid_vis = this->declare_parameter("do_pub_elevgrid_vis", false);
      RCLCPP_INFO(this->get_logger(), "do_pub_elevgrid_vis: %x", do_pub_elevgrid_vis);

      gridmap = std::make_shared<ElevationGridmap>(ElevationGridmap(
            cellsize,
            height, width,
            traversable_slope));


      clearance_thr = this->declare_parameter("clearance_thr", 0.5);
      RCLCPP_INFO(this->get_logger(), "clearance_thr: %f", clearance_thr);

      pub_occgrid = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
          "occupancy_gridmap",
		      rclcpp::QoS(rclcpp::SensorDataQoS())
		      );
      pub_elevgrid_real = this->create_publisher<sensor_msgs::msg::PointCloud2>(
          "elevation_gridmap",
		      rclcpp::QoS(rclcpp::SensorDataQoS())
		      );
      pub_elevgrid_vis = this->create_publisher<sensor_msgs::msg::PointCloud2>(
          "elevation_gridmap_vis",
		      rclcpp::QoS(rclcpp::SensorDataQoS())
		      );
      pub_occimg = this->create_publisher<sensor_msgs::msg::Image>(
		      "occupancy_img",
		      rclcpp::QoS(rclcpp::SensorDataQoS())
		      );

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

      timer_map = this->create_wall_timer(
		      100ms,
		      std::bind(&ElevationGridmapNode::tick_map, this)
		      );

      trigger_on  = this->create_service<std_srvs::srv::Trigger>(
			    "mapping_on",
			    std::bind(&ElevationGridmapNode::callback_on, this,
				    std::placeholders::_1, std::placeholders::_2)
			    );
      trigger_off  = this->create_service<std_srvs::srv::Trigger>(
			    "mapping_off",
			    std::bind(&ElevationGridmapNode::callback_off, this,
				    std::placeholders::_1, std::placeholders::_2)
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
