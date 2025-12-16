#ifndef __OGM_GRIDMAP_POSE2ODOM__
#define __OGM_GRIDMAP_POSE2ODOM__

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose.hpp"


class Pose2OdomNode  : public rclcpp::Node
{
  private:
    
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr sub_pose;

    std::string map_frame;
    std::string target_frame;

    void callback_pose(
        const geometry_msgs::msg::Pose::ConstSharedPtr msg_pose
        )
    {
            rclcpp::Time now = this->get_clock()->now();

	    nav_msgs::msg::Odometry msg_odom;
	    msg_odom.header.stamp = now;
	    msg_odom.header.frame_id = this->target_frame;
	    msg_odom.child_frame_id = this->target_frame;

	    msg_odom.pose.pose = *msg_pose;

	    this->pub_odom->publish(msg_odom);

    }
  
  public:
    Pose2OdomNode() : Node("pcd_transformer")
    {
      map_frame = this->declare_parameter("map_frame", "map_frame");
      RCLCPP_INFO(this->get_logger(), "map_frame: %s", map_frame.c_str());

      target_frame = this->declare_parameter("target_frame", "target_frame");
      RCLCPP_INFO(this->get_logger(), "target_frame: %s", target_frame.c_str());

      pub_odom = this->create_publisher<nav_msgs::msg::Odometry>("odom", 1);
 
      sub_pose = this->create_subscription<geometry_msgs::msg::Pose>(
		      "pose",
		      rclcpp::QoS(rclcpp::SensorDataQoS()),
		      std::bind(&Pose2OdomNode::callback_pose, this, std::placeholders::_1)
		      );

    }
};


int init_pose_2_odom_node(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<Pose2OdomNode>());
  rclcpp::shutdown();
  return 0;
}

#endif
