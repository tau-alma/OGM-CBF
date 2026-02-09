#ifndef __OGM_GRIDMAP_LASERSCAN_TRANSFORMER__
#define __OGM_GRIDMAP_LASERSCAN_TRANSFORMER__

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

#include "pcl_ros/transforms.hpp"

#include "manual_sync.h"

class TransformerNode  : public rclcpp::Node
{
  private:
    
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_pcd;
    
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_pcd;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom;
    std::shared_ptr<ManualSync<
      nav_msgs::msg::Odometry::SharedPtr,
      sensor_msgs::msg::PointCloud2::SharedPtr>> manual_sync;

    std::shared_ptr<tf2_ros::TransformListener> tf_listener{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer;

    std::string odom_frame;
    std::string link_frame;
    std::string target_frame;

    Eigen::Matrix4f T_link2sensor = Eigen::Matrix4f::Identity();

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

    }

    Eigen::Matrix4f get_T_matrix(std::string from_frame, std::string to_frame)
    {
        std::shared_ptr<geometry_msgs::msg::TransformStamped> transform(nullptr);

        if (from_frame != to_frame)
        {
          RCLCPP_INFO(this->get_logger(), "from (%s) and to (%s) frames differ, assuming static transform", from_frame.c_str(), to_frame.c_str());

          tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
          tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
    
       	while (transform == nullptr)
        {
          try
          {
            transform = std::make_shared<geometry_msgs::msg::TransformStamped>(
            tf_buffer->lookupTransform(from_frame, to_frame, tf2::TimePointZero));
          }
          catch (const tf2::TransformException & ex)
          {
            RCLCPP_INFO(this->get_logger(), "failed to get transform, retrying");
            sleep(1);
          }
        }
        RCLCPP_INFO(this->get_logger(), "acquired link to sensor transform");
        RCLCPP_INFO(this->get_logger(),
            "t=(%f, %f %f)",
            transform->transform.translation.x,
            transform->transform.translation.y,
            transform->transform.translation.z
            );
        RCLCPP_INFO(this->get_logger(),
            "q=(%f, %f, %f, %f)",
            transform->transform.rotation.x,
            transform->transform.rotation.y,
            transform->transform.rotation.z,
            transform->transform.rotation.w
            );
      }
      else
      {
        RCLCPP_INFO(this->get_logger(), "from (%s) and to (%s) frames do not differ", from_frame.c_str(), to_frame.c_str());
      }

      return get_T_matrix(transform);
    }

    void process(double stamp)
    {
      //RCLCPP_INFO(this->get_logger(), "processing at %lf", stamp);
      auto pair = manual_sync->pop(stamp);
      //RCLCPP_INFO(this->get_logger(), "popped");

      if (pair.first && pair.second)
      {
        const nav_msgs::msg::Odometry::SharedPtr msg_odom = *pair.first;
        const sensor_msgs::msg::PointCloud2::SharedPtr msg_pcd = *pair.second;
       
        rclcpp::Time odom_stamp(msg_odom->header.stamp);
        rclcpp::Time pcd_stamp(msg_pcd->header.stamp);

        /*RCLCPP_INFO(this->get_logger(), "sync odom-pcd at %lf--%lf (%lf)",
            odom_stamp.seconds(),
            pcd_stamp.seconds(),
            odom_stamp.seconds() - pcd_stamp.seconds()
	    );*/
        
	Eigen::Matrix4f T_odom2link = get_T_matrix(msg_odom);
 
	Eigen::Matrix4f T_map2sensor =  T_odom2link * T_link2sensor;
	           
	sensor_msgs::msg::PointCloud2 msg_out;
	pcl_ros::transformPointCloud (T_map2sensor, *msg_pcd, msg_out);
			     
	msg_out.header.frame_id = odom_frame;
	pub_pcd->publish(msg_out);

      }
    }

    void callback_odom(
        const nav_msgs::msg::Odometry::SharedPtr msg_odom
        )
    {
      rclcpp::Time odom_stamp(msg_odom->header.stamp);
      //RCLCPP_INFO(this->get_logger(), "odom at %lf", odom_stamp.seconds());
      manual_sync->push_t1(msg_odom, odom_stamp.seconds());
      
      auto sz = manual_sync->size();
      auto r1 = manual_sync->range_t1();
      auto r2 = manual_sync->range_t2();
      //RCLCPP_INFO(this->get_logger(), "pushed %ld:(%lf--%lf) %ld:(%lf--%lf)", sz.first, r1.first, r1.second, sz.second, r2.first, r2.second);
      
      process(odom_stamp.seconds());  
      sz = manual_sync->size();
      r1 = manual_sync->range_t1();
      r2 = manual_sync->range_t2();
      //RCLCPP_INFO(this->get_logger(), "processed %ld:(%lf--%lf) %ld:(%lf--%lf)", sz.first, r1.first, r1.second, sz.second, r2.first, r2.second);
    }

    void callback_pcd(
        const sensor_msgs::msg::PointCloud2::SharedPtr msg_pcd
        )
    {
      rclcpp::Time pcd_stamp(msg_pcd->header.stamp);
      //RCLCPP_INFO(this->get_logger(), "pcd at %lf", pcd_stamp.seconds());
      manual_sync->push_t2(msg_pcd, pcd_stamp.seconds());
      auto sz = manual_sync->size();
      auto r1 = manual_sync->range_t1();
      auto r2 = manual_sync->range_t2();
      //RCLCPP_INFO(this->get_logger(), "pushed %ld:(%lf--%lf) %ld:(%lf--%lf)", sz.first, r1.first, r1.second, sz.second, r2.first, r2.second);
      
      process(pcd_stamp.seconds());  
      sz = manual_sync->size();
      r1 = manual_sync->range_t1();
      r2 = manual_sync->range_t2();
      //RCLCPP_INFO(this->get_logger(), "processed %ld:(%lf--%lf) %ld:(%lf--%lf)", sz.first, r1.first, r1.second, sz.second, r2.first, r2.second);
    }
  
  public:
    TransformerNode() : Node("pcd_transformer")
    {
      odom_frame = this->declare_parameter("odom_frame", "odom_frame");
      RCLCPP_INFO(this->get_logger(), "odom_frame: %s", odom_frame.c_str());

      link_frame = this->declare_parameter("link_frame", "link_frame");
      RCLCPP_INFO(this->get_logger(), "link_frame: %s", link_frame.c_str());

      target_frame = this->declare_parameter("target_frame", "target_frame");
      RCLCPP_INFO(this->get_logger(), "target_frame: %s", target_frame.c_str());

      float sync_slack = this->declare_parameter("sync_slack", 0.1);
      RCLCPP_INFO(this->get_logger(), "sync_slack: %f", sync_slack);

      float sync_window = this->declare_parameter("sync_window", 1.0);
      RCLCPP_INFO(this->get_logger(), "sync_window: %f", sync_window);


      manual_sync = std::make_shared<ManualSync<
          nav_msgs::msg::Odometry::SharedPtr,
          sensor_msgs::msg::PointCloud2::SharedPtr>>(
              ManualSync<
          nav_msgs::msg::Odometry::SharedPtr,
          sensor_msgs::msg::PointCloud2::SharedPtr>
              (sync_slack, sync_window));

      T_link2sensor = get_T_matrix(link_frame, target_frame);

      pub_pcd = this->create_publisher<sensor_msgs::msg::PointCloud2>(
		      "pcd_out",
                      rclcpp::QoS(rclcpp::SensorDataQoS())
		      );
 
      sub_pcd = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "pcd_in",
        rclcpp::QoS(rclcpp::SensorDataQoS()),
        std::bind(&TransformerNode::callback_pcd, this, std::placeholders::_1)
        );
      sub_odom = this->create_subscription<nav_msgs::msg::Odometry>(
        "odom",
        rclcpp::QoS(rclcpp::SensorDataQoS()),
        std::bind(&TransformerNode::callback_odom, this, std::placeholders::_1)
        );
    }
};


int init_pcd_transformer_node(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<TransformerNode>());
  rclcpp::shutdown();
  return 0;
}

#endif
