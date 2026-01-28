#ifndef __OGM_GRIDMAP_LASERSCAN_TRANSFORMER__
#define __OGM_GRIDMAP_LASERSCAN_TRANSFORMER__

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

class Scan2PcdNode  : public rclcpp::Node
{
  private:
    
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_pcd;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_scan;

    std::string target_frame;

    void callback_scan(
        const sensor_msgs::msg::LaserScan::SharedPtr msg_scan
        )
    {
      if (msg_scan->header.frame_id == target_frame)
      {    
      pcl::PointCloud<pcl::PointXYZI> pcd;

      // push origin ref point
      pcl::PointXYZI ref_pt;
      ref_pt.x = 0.;
      ref_pt.y = 0.;
      ref_pt.z = 0.;
      ref_pt.intensity = 0.;
      pcd.push_back(ref_pt);

      for (int i = 0; i < msg_scan->ranges.size() ; i++)
      {
          double ang = msg_scan->angle_min + i * msg_scan->angle_increment;
          if (msg_scan->ranges.at(i) != NAN)
	  {
            pcl::PointXYZI pt;
            pt.x = (msg_scan->ranges.at(i) * cos(ang));
            pt.y = (msg_scan->ranges.at(i) * sin(ang));
            pt.z = 0.;
            pt.intensity = 1;
	    pcd.push_back(pt);
          }
      }

      pcl::PCLPointCloud2 pcdp;
      pcl::toPCLPointCloud2(pcd, pcdp);

      sensor_msgs::msg::PointCloud2 msg_out;
      pcl_conversions::fromPCL(pcdp, msg_out);

      msg_out.header = msg_scan->header;
      pub_pcd->publish(msg_out);
      }
    }
  
  public:
    Scan2PcdNode() : Node("scan_transformer")
    {
      target_frame = this->declare_parameter("target_frame", "target_frame");
      RCLCPP_INFO(this->get_logger(), "target_frame: %s", target_frame.c_str());

      pub_pcd = this->create_publisher<sensor_msgs::msg::PointCloud2>("pcd",
		      rclcpp::QoS(rclcpp::SensorDataQoS())
		      );

      sub_scan = this->create_subscription<sensor_msgs::msg::LaserScan>(
		      "scan",
		      rclcpp::QoS(rclcpp::SensorDataQoS()),
		      std::bind(&Scan2PcdNode::callback_scan, this, std::placeholders::_1)
		      );

    }
};


int init_scan2pcd_node(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<Scan2PcdNode>());
  rclcpp::shutdown();
  return 0;
}

#endif
