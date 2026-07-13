#ifndef __OGM_GRIDMAP_ELEVATION_GRIDMAP_NODE__
#define __OGM_GRIDMAP_ELEVATION_GRIDMAP_NODE__

#include "rclcpp/rclcpp.hpp"
#include "rcutils/logging.h"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "logging_demo/srv/config_logger.hpp"

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
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_elevimg;
    rclcpp::TimerBase::SharedPtr timer_map;

    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr trigger_on;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr trigger_off;
    rclcpp::Service<logging_demo::srv::ConfigLogger>::SharedPtr logger_service;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_pc2;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom;

    float occgrid_vis_z;
    std::string map_frame;

    float clearance_thr_pos_low;
    float clearance_thr_pos_high;
    float clearance_thr_dir_low;
    float clearance_thr_dir_high;
    double max_clearance_age;
    double clearance_ts; 
    
    bool do_update;
    bool do_reset;
    bool do_clear_pos;
    bool invert_clear_pos;
    bool do_clear_dir;
    bool invert_clear_dir;

    bool do_pub_occgrid;
    bool do_pub_occimg;
    bool do_pub_elevimg;
    bool flip_occimg_values;
    bool discretize_elevimg;
    float elevimg_z_res; 
    bool do_pub_elevgrid_real;
    bool do_pub_elevgrid_vis;

    std::shared_ptr<ElevationGridmap> gridmap;

    void handle_logger_config_request(
        const std::shared_ptr<logging_demo::srv::ConfigLogger::Request> request,
        std::shared_ptr<logging_demo::srv::ConfigLogger::Response> response)
    {
      const char * severity_string = request->level.c_str();
      int severity;
      rcutils_ret_t ret = rcutils_logging_severity_level_from_string(
          severity_string,
          rcl_get_default_allocator(),
          &severity
          );
     
      if (RCUTILS_RET_OK != ret)
      {
        RCLCPP_ERROR(this->get_logger(), "Failed to parse severity: %s", severity_string);
      }

      if (RCUTILS_RET_OK == ret)
      {
        ret = rcutils_logging_set_logger_level(request->logger_name.c_str(), severity);
      }

      if (RCUTILS_RET_OK != ret)
      {
        RCLCPP_ERROR(this->get_logger(), "Failed to set severity");
        response->success = false;
      }
      else
      {
        RCLCPP_INFO(this->get_logger(), "Set %s severity to %s (%d)", request->logger_name.c_str(), severity_string, severity);
        response->success = true;
      }



    }

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
      std::vector<uint8_t> data = gridmap->report_2d_uint8(flip_occimg_values);
      
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

    void publish_elevimg(rclcpp::Time& now)
    {

      cv::Mat img;
      cv_bridge::CvImage cv_bridge_image;

      if (discretize_elevimg)
      {
        std::vector<int8_t> data = gridmap->report_3d_int8(elevimg_z_res, occgrid_vis_z);
        
        cv::Mat map(
          gridmap->get_height(),
          gridmap->get_width(),
          CV_8S,
          data.data()
          );
        cv::flip(map, img, 0);
        cv_bridge_image.encoding = sensor_msgs::image_encodings::MONO8;
      }
      else
      {
        std::vector<float> data = gridmap->report_3d_float(occgrid_vis_z);

        cv::Mat map(
          gridmap->get_height(),
          gridmap->get_width(),
          CV_32FC1,
          data.data()
          );
        cv::flip(map, img, 0);
        cv_bridge_image.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
      }

      cv_bridge_image.image = img;
      sensor_msgs::msg::Image msg_img = *(cv_bridge_image.toImageMsg());
	    msg_img.header.stamp = now;
	    msg_img.header.frame_id = this->map_frame;

      pub_elevimg->publish(msg_img);
    }

    void tick_map()
    {
      RCLCPP_DEBUG(this->get_logger(), "--------------------");
      RCLCPP_DEBUG(this->get_logger(), ">> map tick ");
      
      auto wall_start = std::chrono::high_resolution_clock::now();
      
      rclcpp::Time ts = this->now();

      // pc2 elev grid msg
      if (do_pub_elevgrid_real) publish_elevgrid_real(ts);

      auto wall_real = std::chrono::high_resolution_clock::now();
      RCLCPP_DEBUG(this->get_logger(), "wall-tick real: %lf ms",
		      std::chrono::duration<double, std::milli>(wall_real - wall_start).count());

      if (do_pub_elevgrid_vis) publish_elevgrid_vis(ts);

      auto wall_vis = std::chrono::high_resolution_clock::now();
      RCLCPP_DEBUG(this->get_logger(), "wall-tick vis: %lf ms",
		      std::chrono::duration<double, std::milli>(wall_vis - wall_real).count());
      
      // occgrid
      if (do_pub_occgrid) publish_occgrid(ts);

      auto wall_grid = std::chrono::high_resolution_clock::now();
      RCLCPP_DEBUG(this->get_logger(), "wall-tick grid: %lf ms",
		      std::chrono::duration<double, std::milli>(wall_grid - wall_vis).count());

      // occimg
      if (do_pub_occimg) publish_occimg(ts);
      
      // elevimg
      if (do_pub_elevimg) publish_elevimg(ts);
      
      auto wall_img = std::chrono::high_resolution_clock::now();
      RCLCPP_DEBUG(this->get_logger(), "wall-tick img: %lf ms",
		      std::chrono::duration<double, std::milli>(wall_img - wall_grid).count());

      RCLCPP_DEBUG(this->get_logger(), "<< map tick total : %lf ms",
		      std::chrono::duration<double, std::milli>(wall_img - wall_start).count());
      RCLCPP_DEBUG(this->get_logger(), "--------------------");
    }

    void callback_on(
		    const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
		    const std::shared_ptr<std_srvs::srv::Trigger::Response> res)
    {
      if (req != nullptr) // handle unused warning
      {
	      do_update = true;
        res->success = true;
      }
    }
    void callback_off(
		    const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
		    const std::shared_ptr<std_srvs::srv::Trigger::Response> res)
    {
      if (req != nullptr) // handle unused warning
      {
	      do_update = false;
        res->success = true;
      }
    }

    void callback_pc2(const sensor_msgs::msg::PointCloud2::SharedPtr msgp)
    {
      RCLCPP_DEBUG(this->get_logger(), "--------------------");
      RCLCPP_DEBUG(this->get_logger(), ">> pcd callback");

      auto wall_start = std::chrono::high_resolution_clock::now();


      if (do_reset)
      {
        gridmap->reset();
      }


      auto wall_reset = std::chrono::high_resolution_clock::now();
      RCLCPP_DEBUG(this->get_logger(), "wall-callback reset: %lf ms",
		      std::chrono::duration<double, std::milli>(wall_reset - wall_start).count());

      rclcpp::Time pcd_stamp(msgp->header.stamp);
      double pcd_ts = pcd_stamp.seconds();
      double clearance_age = pcd_ts - clearance_ts;

      if (do_update && (max_clearance_age < 0. || clearance_age < max_clearance_age))
      {
        pcl::PCLPointCloud2::Ptr pcdp (new pcl::PCLPointCloud2());
        pcl_conversions::toPCL(*msgp,*pcdp);
        pcl::PointCloud<pcl::PointXYZ> xyz;
        pcl::fromPCLPointCloud2(*pcdp, xyz);

        gridmap->update(xyz);
      }

      auto wall_update = std::chrono::high_resolution_clock::now();
      RCLCPP_DEBUG(this->get_logger(), "wall-callback update: %lf ms",
		      std::chrono::duration<double, std::milli>(wall_update - wall_reset).count());
      
      RCLCPP_DEBUG(this->get_logger(), "<< pcd callback total : %lf ms",
		      std::chrono::duration<double, std::milli>(wall_update - wall_start).count());
      RCLCPP_DEBUG(this->get_logger(), "--------------------");
    }

    void callback_clearance(const nav_msgs::msg::Odometry::SharedPtr msgo)
    {

      Eigen::Quaternionf q(
          msgo->pose.pose.orientation.w,
          msgo->pose.pose.orientation.x,
          msgo->pose.pose.orientation.y,
          msgo->pose.pose.orientation.z);
      Eigen::Matrix3f R = q.toRotationMatrix();

      float magnitude = msgo->twist.twist.linear.x;

      ClearanceConfig cfg_pos(do_clear_pos, invert_clear_pos, clearance_thr_pos_low, clearance_thr_pos_high);
      ClearanceConfig cfg_dir(do_clear_dir, invert_clear_dir, clearance_thr_dir_low, clearance_thr_dir_high);
      Clearance clearance(
          cfg_pos, cfg_dir,
          msgo->pose.pose.position.x,
          msgo->pose.pose.position.y,
          msgo->pose.pose.position.z,
          R(0,0)*magnitude,
          R(1,0)*magnitude,
          R(2,0)*magnitude
          );

      gridmap->update_clearance(clearance);

      rclcpp::Time clearance_stamp(msgo->header.stamp);
      clearance_ts = clearance_stamp.seconds();
    }

  public:
    ElevationGridmapNode() : Node("gridmap")
    {

      RCLCPP_INFO(this->get_logger(), "logging into %s", this->get_logger().get_name());

      float cellsize = this->declare_parameter("cellsize", 0.025);
      RCLCPP_INFO(this->get_logger(), "cellsize: %f", cellsize);

      float _height = this->declare_parameter("height", 20.0);
      uint32_t height = std::ceil(_height / cellsize) ;
      RCLCPP_INFO(this->get_logger(), "height: %f -> %d", _height, height);

      float _width = this->declare_parameter("width", 20.0);
      uint32_t width = std::ceil(_width / cellsize) ;
      RCLCPP_INFO(this->get_logger(), "width: %f -> %d", _width, width);

      int pt_step = this->declare_parameter("pt_step", 1);
      RCLCPP_INFO(this->get_logger(), "pt_step: %d", pt_step);

      float traversable_slope = this->declare_parameter("traversable_slope", 0.78);
      RCLCPP_INFO(this->get_logger(), "traversable_slope: %f", traversable_slope);

      float traversability_r = this->declare_parameter("traversability_r", 0.5);
      RCLCPP_INFO(this->get_logger(), "traversability_r: %f", traversability_r);

      float traversable_z = this->declare_parameter("traversable_z", 1e6);
      RCLCPP_INFO(this->get_logger(), "traversable_z: %f", traversable_z);

      float crop_z_max = this->declare_parameter("crop_z_max", 1e6);
      RCLCPP_INFO(this->get_logger(), "crop_z_max: %f", crop_z_max);

      float cell_z_var = this->declare_parameter("cell_z_var", 1.0);
      RCLCPP_INFO(this->get_logger(), "cell_z_var: %f", cell_z_var);

      float cell_sensor_var = this->declare_parameter("cell_sensor_var", 0.1);
      RCLCPP_INFO(this->get_logger(), "cell_sensor_var: %f", cell_sensor_var);

      float cell_system_var = this->declare_parameter("cell_system_var", 0.01);
      RCLCPP_INFO(this->get_logger(), "cell_system_var: %f", cell_system_var);

      bool cell_update_var = this->declare_parameter("cell_update_var", false);
      RCLCPP_INFO(this->get_logger(), "cell_update_var: %x", cell_update_var);

      do_update = true;

      do_reset = this->declare_parameter("do_reset", false);
      RCLCPP_INFO(this->get_logger(), "do_reset: %x", do_reset);

      map_frame = this->declare_parameter("map_frame", "map_frame");
      RCLCPP_INFO(this->get_logger(), "map_frame: %s", map_frame.c_str());

      do_pub_occgrid = this->declare_parameter("do_pub_occgrid", false);
      RCLCPP_INFO(this->get_logger(), "do_pub_occgrid: %x", do_pub_occgrid);

      occgrid_vis_z = this->declare_parameter("occgrid_vis_z", 0.0);
      RCLCPP_INFO(this->get_logger(), "occgrid_vis_z: %f", occgrid_vis_z);

      do_pub_occimg = this->declare_parameter("do_pub_occimg", false);
      RCLCPP_INFO(this->get_logger(), "do_pub_occimg: %x", do_pub_occimg);

      do_pub_elevimg = this->declare_parameter("do_pub_elevimg", false);
      RCLCPP_INFO(this->get_logger(), "do_pub_elevimg: %x", do_pub_elevimg);

      flip_occimg_values = this->declare_parameter("flip_occimg_values", false);
      RCLCPP_INFO(this->get_logger(), "flip_occimg_values: %x", flip_occimg_values);

      discretize_elevimg = this->declare_parameter("discretize_elevimg", true);
      RCLCPP_INFO(this->get_logger(), "discretize_elevimg: %x", discretize_elevimg);

      elevimg_z_res = this->declare_parameter("elevimg_z_res", 0.02);
      RCLCPP_INFO(this->get_logger(), "elevimg_z_res: %f", elevimg_z_res);

      do_pub_elevgrid_real = this->declare_parameter("do_pub_elevgrid_real", false);
      RCLCPP_INFO(this->get_logger(), "do_pub_elevgrid_real: %x", do_pub_elevgrid_real);

      do_pub_elevgrid_vis = this->declare_parameter("do_pub_elevgrid_vis", false);
      RCLCPP_INFO(this->get_logger(), "do_pub_elevgrid_vis: %x", do_pub_elevgrid_vis);

      gridmap = std::make_shared<ElevationGridmap>(ElevationGridmap(
            cellsize,
            height, width,
            pt_step,
            traversable_slope,
            traversability_r,
            traversable_z,
            crop_z_max,
            cell_z_var,
            cell_sensor_var,
            cell_system_var,
            cell_update_var));


      clearance_thr_pos_low = this->declare_parameter("clearance_thr_pos_low", 0.5);
      RCLCPP_INFO(this->get_logger(), "clearance_thr_pos_low: %f", clearance_thr_pos_low);
      
      clearance_thr_pos_high = this->declare_parameter("clearance_thr_pos_high", 0.5);
      RCLCPP_INFO(this->get_logger(), "clearance_thr_pos_high: %f", clearance_thr_pos_high);
      
      clearance_thr_dir_low = this->declare_parameter("clearance_thr_dir_low", 0.5);
      RCLCPP_INFO(this->get_logger(), "clearance_thr_dir_low: %f", clearance_thr_dir_low);
      
      clearance_thr_dir_high = this->declare_parameter("clearance_thr_dir_high", 0.5);
      RCLCPP_INFO(this->get_logger(), "clearance_thr_dir_high: %f", clearance_thr_dir_high);
      
      max_clearance_age = this->declare_parameter("max_clearance_age", -1.0);
      RCLCPP_INFO(this->get_logger(), "max_clearance_age: %lf", max_clearance_age);

      clearance_ts = max_clearance_age < 0 ? 0 : -2*max_clearance_age;

      do_clear_pos = this->declare_parameter("do_clear_pos", false);
      RCLCPP_INFO(this->get_logger(), "do_clear_pos: %x", do_clear_pos);

      invert_clear_pos = this->declare_parameter("invert_clear_pos", false);
      RCLCPP_INFO(this->get_logger(), "invert_clear_pos: %x", invert_clear_pos);

      do_clear_dir = this->declare_parameter("do_clear_dir", false);
      RCLCPP_INFO(this->get_logger(), "do_clear_dir: %x", do_clear_dir);

      invert_clear_dir = this->declare_parameter("invert_clear_dir", false);
      RCLCPP_INFO(this->get_logger(), "invert_clear_dir: %x", invert_clear_dir);

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
      pub_elevimg = this->create_publisher<sensor_msgs::msg::Image>(
		      "elevation_img",
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

      logger_service = create_service<logging_demo::srv::ConfigLogger>(
          "config_logger",
          std::bind(
            &ElevationGridmapNode::handle_logger_config_request, this,
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
