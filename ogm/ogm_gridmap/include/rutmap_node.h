#ifndef __OGM_GRIDMAP_RUTMAP_NODE__
#define __OGM_GRIDMAP_RUTMAP_NODE__

#include "rclcpp/rclcpp.hpp"

#include <chrono>   
#include <opencv2/opencv.hpp>   
#include <cv_bridge/cv_bridge.h>

using namespace std::chrono_literals;

class RutMapNode  : public rclcpp::Node
{
  private:
    
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_img_rut;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_img_vis;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_img_front;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_img_rear;
    rclcpp::TimerBase::SharedPtr timer_map;

    sensor_msgs::msg::Image::SharedPtr _front_elevimg;
    sensor_msgs::msg::Image::SharedPtr _rear_elevimg;

    bool do_pub_vis;
    bool do_pub_exact;
    float vis_scale;

    void pub_exact(cv::Mat& img_front, cv::Mat& img_rear, std::string encoding)
    {
      cv::Mat img_rut = (img_front - img_rear);

      cv_bridge::CvImage cv_bridge_image;
      cv_bridge_image.encoding = encoding;
      cv_bridge_image.image = img_rut;

      sensor_msgs::msg::Image msg_img = *(cv_bridge_image.toImageMsg());
      msg_img.header = _front_elevimg->header;

      pub_img_rut->publish(msg_img);
    }

    void pub_vis(cv::Mat& img_front, cv::Mat& img_rear, std::string encoding)
    {
      cv::Mat img_rut = (img_front - img_rear)*vis_scale;

      cv_bridge::CvImage cv_bridge_image;
      cv_bridge_image.encoding = encoding;
      cv_bridge_image.image = img_rut;

      sensor_msgs::msg::Image msg_img = *(cv_bridge_image.toImageMsg());
      msg_img.header = _front_elevimg->header;

      pub_img_vis->publish(msg_img);
    }

    void process_img_pair(std::string encoding)
    {
      cv_bridge::CvImagePtr front_image_ptr = cv_bridge::toCvCopy(_front_elevimg, encoding);
      cv_bridge::CvImagePtr rear_image_ptr  = cv_bridge::toCvCopy(_rear_elevimg, encoding);

      cv::Mat img_front = front_image_ptr->image;
      cv::Mat img_rear = rear_image_ptr->image;

      if (do_pub_exact) pub_exact(img_front, img_rear, encoding);
      if (do_pub_vis) pub_vis(img_front, img_rear, encoding);
    }

    void callback_img_front(
        const sensor_msgs::msg::Image::SharedPtr msgimg
        )
    {
      _front_elevimg = msgimg;
    }

    void callback_img_rear(
        const sensor_msgs::msg::Image::SharedPtr msgimg
        )
    {
      _rear_elevimg = msgimg;
    }


    void tick_map()
    {
      if (!_rear_elevimg || !_front_elevimg);
      else if (_front_elevimg->encoding != _rear_elevimg->encoding) 
        RCLCPP_WARN(this->get_logger(), "different encoding: %s %s",
            _front_elevimg->encoding.c_str(), _rear_elevimg->encoding.c_str());
      else process_img_pair(_front_elevimg->encoding);
    }

  public:
    RutMapNode() : Node("rut_map")
    {

      do_pub_exact = this->declare_parameter("do_pub_exact", false);
      RCLCPP_INFO(this->get_logger(), "do_pub_exact: %x", do_pub_exact);

      do_pub_vis = this->declare_parameter("do_pub_vis", false);
      RCLCPP_INFO(this->get_logger(), "do_pub_vis: %x", do_pub_vis);

      vis_scale = this->declare_parameter("vis_scale", 1.0);
      RCLCPP_INFO(this->get_logger(), "vis_scale: %f", vis_scale);

      pub_img_rut = this->create_publisher<sensor_msgs::msg::Image>(
		      "elevation_img_rut",
		      rclcpp::QoS(rclcpp::SensorDataQoS())
		      );

      pub_img_vis = this->create_publisher<sensor_msgs::msg::Image>(
		      "elevation_img_rut_scaled",
		      rclcpp::QoS(rclcpp::SensorDataQoS())
		      );

      sub_img_front = this->create_subscription<sensor_msgs::msg::Image>(
		      "elevation_img_front",
		      rclcpp::QoS(rclcpp::SensorDataQoS()),
		      std::bind(&RutMapNode::callback_img_front, this, std::placeholders::_1)
		      );

      sub_img_rear = this->create_subscription<sensor_msgs::msg::Image>(
		      "elevation_img_rear",
		      rclcpp::QoS(rclcpp::SensorDataQoS()),
		      std::bind(&RutMapNode::callback_img_rear, this, std::placeholders::_1)
		      );

      timer_map = this->create_wall_timer(
		      100ms,
		      std::bind(&RutMapNode::tick_map, this)
		      );

    }
};


int init_rutmap_node(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<RutMapNode>());
  rclcpp::shutdown();
  return 0;
}

#endif
