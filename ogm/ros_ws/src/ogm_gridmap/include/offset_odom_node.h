#ifndef __OGM_GRIDMAP_OFFSET_ODOM_NODE__
#define __OGM_GRIDMAP_OFFSET_ODOM_NODE__

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose.hpp"

#include "pcl_ros/transforms.hpp"

class OffsetOdomNode  : public rclcpp::Node
{
  private:
    
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr sub_pose;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom;

    bool from_pose;
    bool from_params;
    bool offset_to_link;

    std::string map_frame;
    std::string link_frame;

    Eigen::Matrix4f T_map2initlink = Eigen::Matrix4f::Identity();
    bool has_map2initlink = false;

    Eigen::Matrix4f T_map2odom = Eigen::Matrix4f::Identity();
    bool has_map2odom = false;

    Eigen::Matrix4f get_T_matrix(
      float x, float y, float z,
      float qx, float qy, float qz, float qw)
    {
      Eigen::Matrix4f T = Eigen::Matrix4f::Identity();

      Eigen::Vector3f t(x,y,z);

      Eigen::Quaternionf q;
      q.x() = qx;
      q.y() = qy;
      q.z() = qz;
      q.w() = qw;
      q.normalize();
      Eigen::Matrix3f R(q);

      T.block(0,0,3,3) = R;
      T.block(0,3,3,1) = t;
      T.block(3,0,1,4) << 0, 0, 0, 1;

      return T;
    }

    Eigen::Matrix4f get_T_matrix(const geometry_msgs::msg::Pose& pose)
    {
      Eigen::Matrix4f T = Eigen::Matrix4f::Identity();

      Eigen::Vector3f t(
          pose.position.x,
          pose.position.y,
          pose.position.z
          );

      Eigen::Quaternionf q;
      q.x() = pose.orientation.x;
      q.y() = pose.orientation.y;
      q.z() = pose.orientation.z;
      q.w() = pose.orientation.w;
      q.normalize();
      Eigen::Matrix3f R(q);

      T.block(0,0,3,3) = R;
      T.block(0,3,3,1) = t;
      T.block(3,0,1,4) << 0, 0, 0, 1;

      return T;
    }

    void callback_pose(
        const geometry_msgs::msg::Pose::ConstSharedPtr msg_pose
        )
    {
      if (from_pose && offset_to_link && !has_map2initlink)
      {
        T_map2initlink = get_T_matrix(*msg_pose);
        has_map2initlink = true;
        RCLCPP_INFO(this->get_logger(), "observed offset pose at %lf", this->now().seconds());
      }
      if (from_pose && !offset_to_link && !has_map2odom)
      {
        T_map2odom = get_T_matrix(*msg_pose);
        has_map2odom = true;
        RCLCPP_INFO(this->get_logger(), "observed offset pose at %lf", this->now().seconds());
      }
    }
  
    void callback_odom(
        const nav_msgs::msg::Odometry::ConstSharedPtr msg_odom
        )
    {

      rclcpp::Time odom_stamp(msg_odom->header.stamp);
      Eigen::Matrix4f T_odom2link = get_T_matrix(msg_odom->pose.pose);

      if (!has_map2odom && has_map2initlink)
      {
        RCLCPP_INFO(this->get_logger(), "building map2odom using odom at %lf", odom_stamp.seconds());
        T_map2odom =  T_odom2link.inverse() * T_map2initlink;
        has_map2odom = true;
      }

      if (has_map2odom)
      {
        //RCLCPP_INFO(this->get_logger(), "publish wrt odom at %lf", odom_stamp.seconds());
        Eigen::Matrix4f T_map2link = T_map2odom * T_odom2link;
        //Eigen::Matrix4f T_map2link = T_odom2link ;

        Eigen::Vector3f t = T_map2link.col(3).head<3>();
        Eigen::Matrix3f R = T_map2link.block<3,3>(0,0);
        Eigen::Quaternionf q(R);

	      nav_msgs::msg::Odometry msg_out;
        msg_out.header = msg_odom->header;
        msg_out.header.frame_id = map_frame;
        msg_out.child_frame_id = link_frame;

        msg_out.pose.pose.position.x = t(0);
        msg_out.pose.pose.position.y = t(1);
        msg_out.pose.pose.position.z = t(2);
        msg_out.pose.pose.orientation.x = q.x();
        msg_out.pose.pose.orientation.y = q.y();
        msg_out.pose.pose.orientation.z = q.z();
        msg_out.pose.pose.orientation.w = q.w();

	      this->pub_odom->publish(msg_out);
      }
    }

  public:
    OffsetOdomNode() : Node("pcd_transformer")
    {

      from_pose = this->declare_parameter("from_pose", false);
      RCLCPP_INFO(this->get_logger(), "from_pose: %x", from_pose);

      from_params = this->declare_parameter("from_params", false);
      RCLCPP_INFO(this->get_logger(), "from_params: %x", from_params);

      offset_to_link = this->declare_parameter("offset_to_link", false);
      RCLCPP_INFO(this->get_logger(), "offset_to_link: %x", offset_to_link);

      if (from_params)
      {
        float fixed_offset_x = this->declare_parameter("fixed_offset_x", 0.);
        RCLCPP_INFO(this->get_logger(), "fixed_offset_x: %f", fixed_offset_x);

        float fixed_offset_y = this->declare_parameter("fixed_offset_y", 0.);
        RCLCPP_INFO(this->get_logger(), "fixed_offset_y: %f", fixed_offset_y);

        float fixed_offset_z = this->declare_parameter("fixed_offset_z", 0.);
        RCLCPP_INFO(this->get_logger(), "fixed_offset_z: %f", fixed_offset_z);

        float fixed_offset_qx = this->declare_parameter("fixed_offset_qx", 0.);
        RCLCPP_INFO(this->get_logger(), "fixed_offset_qx: %f", fixed_offset_qx);

        float fixed_offset_qy = this->declare_parameter("fixed_offset_qy", 0.);
        RCLCPP_INFO(this->get_logger(), "fixed_offset_qy: %f", fixed_offset_qy);

        float fixed_offset_qz = this->declare_parameter("fixed_offset_qz", 0.);
        RCLCPP_INFO(this->get_logger(), "fixed_offset_qz: %f", fixed_offset_qz);
        
        float fixed_offset_qw = this->declare_parameter("fixed_offset_qw", 1.);
        RCLCPP_INFO(this->get_logger(), "fixed_offset_qw %f", fixed_offset_qw);

        Eigen::Matrix4f T = get_T_matrix(
            fixed_offset_x, fixed_offset_y, fixed_offset_z,
            fixed_offset_qx, fixed_offset_qy, fixed_offset_qz, fixed_offset_qw);
        
        if (offset_to_link)
        {
            T_map2initlink = T;
            has_map2initlink = true;
        }
        else
        {
            T_map2odom = T;
            has_map2odom = true;
        }

      }

      map_frame = this->declare_parameter("map_frame", "map_frame");
      RCLCPP_INFO(this->get_logger(), "map_frame: %s", map_frame.c_str());

      link_frame = this->declare_parameter("link_frame", "link_frame");
      RCLCPP_INFO(this->get_logger(), "link_frame: %s", link_frame.c_str());

      pub_odom = this->create_publisher<nav_msgs::msg::Odometry>("odom_out", 1);
 
      sub_pose = this->create_subscription<geometry_msgs::msg::Pose>(
		      "offset_pose",
		      rclcpp::QoS(rclcpp::SensorDataQoS()),
		      std::bind(&OffsetOdomNode::callback_pose, this, std::placeholders::_1)
		      );

      sub_odom = this->create_subscription<nav_msgs::msg::Odometry>(
		      "odom_in",
		      rclcpp::QoS(rclcpp::SensorDataQoS()),
		      std::bind(&OffsetOdomNode::callback_odom, this, std::placeholders::_1)
		      );

    }
};


int init_offset_odom_node(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<OffsetOdomNode>());
  rclcpp::shutdown();
  return 0;
}

#endif
