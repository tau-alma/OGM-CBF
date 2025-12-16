#ifndef __OGM_GRIDMAP_LASERSCAN_TRANSFORMER__
#define __OGM_GRIDMAP_LASERSCAN_TRANSFORMER__

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"

//#include "tf2_ros/transform_listener.h"
//#include "tf2_ros/buffer.h"

#include "message_filters/subscriber.hpp"
#include "message_filters/time_synchronizer.hpp"
#include "message_filters/sync_policies/approximate_time.hpp"

class TransformerNode  : public rclcpp::Node
{
  private:
    
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_scan;

    message_filters::Subscriber<sensor_msgs::msg::LaserScan> sub_scan;
    message_filters::Subscriber<nav_msgs::msg::Odometry> sub_odom;
    std::shared_ptr<message_filters::Synchronizer<
      message_filters::sync_policies::ApproximateTime<
      sensor_msgs::msg::LaserScan,
      nav_msgs::msg::Odometry>>> sync;


    //std::shared_ptr<tf2_ros::TransformListener> tf_listener{nullptr};
    //std::unique_ptr<tf2_ros::Buffer> tf_buffer;

    std::string target_frame;
    std::string link_frame;
    std::string sensor_frame;

    //Eigen::Matrix4f T_link2sensor = Eigen::Matrix4f::Identity();

    /*
    Eigen::Matrix4f get_T_matrix(std::shared_ptr<geometry_msgs::msg::TransformStamped> transform)
    {
      Eigen::Matrix4f T = Eigen::Matrix4f::Identity();

      if (transform != nullptr)
      {
		    Eigen::Vector3f t(
            transform->transform.translation.x,
            transform->transform.translation.y,
            transform->transform.translation.z
            );

		    Eigen::Quaternionf q;
        q.x() = transform->transform.rotation.x;
        q.y() = transform->transform.rotation.y;
        q.z() = transform->transform.rotation.z;
        q.w() = transform->transform.rotation.w;
        q.normalize();
        Eigen::Matrix3f R(q);

        T.block(0,0,3,3) = R;
        T.block(0,3,3,1) = t;
        T.block(3,0,1,4) << 0, 0, 0, 1;
      }

      return T;

    }*/
    /*
    Eigen::Matrix4f get_T_matrix(nav_msgs::msg::Odometry::ConstSharedPtr odom)
    {
      Eigen::Matrix4f T = Eigen::Matrix4f::Identity();

      Eigen::Vector3f t(
          odom->pose.pose.position.x,
          odom->pose.pose.position.y,
          odom->pose.pose.position.z
          );

      Eigen::Quaternionf q;
      q.x() = odom->pose.pose.orientation.x;
      q.y() = odom->pose.pose.orientation.y;
      q.z() = odom->pose.pose.orientation.z;
      q.w() = odom->pose.pose.orientation.w;
      q.normalize();
      Eigen::Matrix3f R(q);

      T.block(0,0,3,3) = R;
      T.block(0,3,3,1) = t;
      T.block(3,0,1,4) << 0, 0, 0, 1;

      return T;
    }

    void get_link2sensor()
    {
      std::shared_ptr<geometry_msgs::msg::TransformStamped> transform(nullptr);

      if (link_frame != sensor_frame)
      {
          RCLCPP_INFO(this->get_logger(), "link (%s) and base (%s) frames differ, assuming static transform", link_frame.c_str(), sensor_frame.c_str());

          tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
          tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

          while (transform == nullptr)
          {
            try
            {
              transform = std::make_shared<geometry_msgs::msg::TransformStamped>(
                  tf_buffer->lookupTransform(sensor_frame, link_frame, tf2::TimePointZero));
            }
            catch (const tf2::TransformException & ex)
            {
              RCLCPP_INFO(this->get_logger(), "failed to get link to sensor transform, retrying");
              sleep(1);
            }
          }

          RCLCPP_INFO(this->get_logger(), "acquired link to sensor transform");

      }
      else
      {
          RCLCPP_INFO(this->get_logger(), "link (%s) and base (%s) frames do not differ", link_frame.c_str(), sensor_frame.c_str());
      }

      T_link2sensor = get_T_matrix(transform);
    }*/


    void callback_scan_registration(
        const sensor_msgs::msg::LaserScan::ConstSharedPtr msg_scan,
        const nav_msgs::msg::Odometry::ConstSharedPtr msg_odom
        )
    {
      rclcpp::Time scan_stamp(msg_scan->header.stamp);
      rclcpp::Time odom_stamp(msg_odom->header.stamp);
      RCLCPP_DEBUG(this->get_logger(), "sync %lf %lf", scan_stamp.seconds(), odom_stamp.seconds());

  
      sensor_msgs::msg::LaserScan msg_out;

      //msg_out = *msg_scan;
      msg_out.header.frame_id = target_frame;
      pub_scan->publish(msg_out);
    }
  
  public:
    TransformerNode() : Node("scan_transformer")
    {
      target_frame = this->declare_parameter("target_frame", "target_frame");
      RCLCPP_INFO(this->get_logger(), "target_frame: %s", target_frame.c_str());

      link_frame = this->declare_parameter("link_frame", "link_frame");
      RCLCPP_INFO(this->get_logger(), "link_frame: %s", link_frame.c_str());

      sensor_frame = this->declare_parameter("sensor_frame", "base_sensor");
      RCLCPP_INFO(this->get_logger(), "sensor_frame: %s", sensor_frame.c_str());

      //get_link2sensor();

      pub_scan = this->create_publisher<sensor_msgs::msg::LaserScan>("scan_transformed", 1);

      rclcpp::QoS qos = rclcpp::QoS(10);
      sub_scan.subscribe(this, "scan", qos.get_rmw_qos_profile());
      sub_odom.subscribe(this, "pose", qos.get_rmw_qos_profile());

      uint32_t queue_size = 100;
      sync = std::make_shared<message_filters::Synchronizer<
        message_filters::sync_policies::ApproximateTime<
        sensor_msgs::msg::LaserScan,
        nav_msgs::msg::Odometry>>>(
            message_filters::sync_policies::ApproximateTime<
              sensor_msgs::msg::LaserScan,
              nav_msgs::msg::Odometry>(queue_size),
            sub_scan,
            sub_odom);
      sync->registerCallback(std::bind(
          &TransformerNode::callback_scan_registration,
          this,
          std::placeholders::_1,
          std::placeholders::_2));

    }
};


int init_laserscan_transformer_node(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<TransformerNode>());
  rclcpp::shutdown();
  return 0;
}

#endif
