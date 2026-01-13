#ifndef __OGM_GRIDMAP_GRIDMAP_NODE__
#define __OGM_GRIDMAP_GRIDMAP_NODE__

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "std_srvs/srv/trigger.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <chrono>   
#include <opencv2/opencv.hpp>   
#include <cv_bridge/cv_bridge.h>

#include "gridmap.h"

using namespace std::chrono_literals;

class GridmapNode  : public rclcpp::Node
{
  private:
    
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_grid;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_img;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_pcd;
    rclcpp::TimerBase::SharedPtr timer_map;

    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr trigger_on;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr trigger_off;

    std::string map_frame;

    float cell_size;
    uint32_t height;
    uint32_t width;

    bool do_update;
    bool do_pub_grid;
    bool do_pub_img;

    std::shared_ptr<Gridmap> gridmap;

    void publish_grid(rclcpp::Time& now)
    {
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

    void publish_image(rclcpp::Time& now)
    {

      auto wall_start = std::chrono::high_resolution_clock::now();
      std::vector<uint8_t> data = gridmap->report_uint8();
      auto wall_fetch = std::chrono::high_resolution_clock::now();
      RCLCPP_DEBUG(this->get_logger(), "wall-img fetch: %lf ms",
		      std::chrono::duration<double, std::milli>(wall_fetch - wall_start));
      
      
      cv::Mat map(
        gridmap->get_height(),
        gridmap->get_width(),
        CV_8U,
        data.data()
        );
      cv::Mat img;
      cv::flip(map, img, 0);

      auto wall_build = std::chrono::high_resolution_clock::now();
      RCLCPP_DEBUG(this->get_logger(), "wall-img build: %lf ms",
		      std::chrono::duration<double, std::milli>(wall_build - wall_fetch));

      cv_bridge::CvImage cv_bridge_image;
      cv_bridge_image.encoding = sensor_msgs::image_encodings::MONO8;
      cv_bridge_image.image = img;
      auto wall_bridge = std::chrono::high_resolution_clock::now();
      RCLCPP_DEBUG(this->get_logger(), "wall-img bridge: %lf ms",
		      std::chrono::duration<double, std::milli>(wall_bridge - wall_build));

      sensor_msgs::msg::Image msg_img = *(cv_bridge_image.toImageMsg());
	    msg_img.header.stamp = now;
	    msg_img.header.frame_id = this->map_frame;
      auto wall_msg = std::chrono::high_resolution_clock::now();
      RCLCPP_DEBUG(this->get_logger(), "wall-img msg: %lf ms",
		      std::chrono::duration<double, std::milli>(wall_msg - wall_bridge));

      pub_img->publish(msg_img);
      auto wall_pub = std::chrono::high_resolution_clock::now();
      RCLCPP_DEBUG(this->get_logger(), "wall-img pub: %lf ms",
		      std::chrono::duration<double, std::milli>(wall_pub - wall_msg));

    }

    void callback_pcd(
        const sensor_msgs::msg::PointCloud2::SharedPtr msg_pcd
        )
    {
      auto wall_start = std::chrono::high_resolution_clock::now();
      rclcpp::Time ts(msg_pcd->header.stamp);
      rclcpp::Time now = this->now();
      RCLCPP_DEBUG(this->get_logger(), "--------------------");
      RCLCPP_DEBUG(this->get_logger(), "pcd callback, delay %lf ms", (now.seconds() - ts.seconds())*1e3);

      pcl::PCLPointCloud2 pcdp;
      pcl_conversions::toPCL(*msg_pcd, pcdp);
      pcl::PointCloud<pcl::PointXYZI> pcd;
      pcl::fromPCLPointCloud2(pcdp, pcd);
      
      auto wall_conv = std::chrono::high_resolution_clock::now();
      RCLCPP_DEBUG(this->get_logger(), "wall conversions: %lf ms",
		      std::chrono::duration<double, std::milli>(wall_conv - wall_start));


      if (do_update) gridmap->update(pcd);
      auto wall_update = std::chrono::high_resolution_clock::now();
      RCLCPP_DEBUG(this->get_logger(), "wall update: %lf ms",
		      std::chrono::duration<double, std::milli>(wall_update - wall_conv));
    }


    void tick_map()
    {
      rclcpp::Time ts = this->now();

      if (do_pub_grid) publish_grid(ts);
      if (do_pub_img) publish_image(ts);
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


  public:
    GridmapNode() : Node("gridmap")
    {
      do_update = true;

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

      do_pub_grid = this->declare_parameter("do_pub_grid", false);
      RCLCPP_INFO(this->get_logger(), "do_pub_grid: %x", do_pub_grid);

      do_pub_img = this->declare_parameter("do_pub_img", false);
      RCLCPP_INFO(this->get_logger(), "do_pub_img: %x", do_pub_img);

      gridmap = std::make_shared<Gridmap>(Gridmap(height,width,cell_size, s_target));

      pub_grid = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
		      "gridmap",
		      rclcpp::QoS(rclcpp::SensorDataQoS())
		      );
      pub_img = this->create_publisher<sensor_msgs::msg::Image>(
		      "imgmap",
		      rclcpp::QoS(rclcpp::SensorDataQoS())
		      );

      sub_pcd = this->create_subscription<sensor_msgs::msg::PointCloud2>(
		      "pcd",
		      rclcpp::QoS(rclcpp::SensorDataQoS()),
		      std::bind(&GridmapNode::callback_pcd, this, std::placeholders::_1)
		      );

      timer_map = this->create_wall_timer(
		      100ms,
		      std::bind(&GridmapNode::tick_map, this)
		      );

      trigger_on  = this->create_service<std_srvs::srv::Trigger>(
			    "mapping_on",
			    std::bind(&GridmapNode::callback_on, this,
				    std::placeholders::_1, std::placeholders::_2)
			    );
      trigger_off  = this->create_service<std_srvs::srv::Trigger>(
			    "mapping_off",
			    std::bind(&GridmapNode::callback_off, this,
				    std::placeholders::_1, std::placeholders::_2)
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
