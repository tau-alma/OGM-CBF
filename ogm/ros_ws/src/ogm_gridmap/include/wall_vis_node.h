#ifndef __OGM_GRIDMAP_WALL_VIS_NODE__
#define __OGM_GRIDMAP_WALL_VIS_NODE__

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <image_transport/image_transport.hpp>
#include "std_msgs/msg/header.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "cv_bridge/cv_bridge.h"
#include <opencv2/opencv.hpp>

class WallVisNode  : public rclcpp::Node
{
  private:
    
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_grid;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_img;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_pcd;

    float image_resolution;

    float wall_height;
    float clearance_height;
    
    float thr_free;
    float thr_obst;

    int dilation_size;

    bool show_dst;
    float max_dst;

    // FF6F3C
    const static uint8_t FREE_R = 255;
    const static uint8_t FREE_G = (6 << 4) | (15);
    const static uint8_t FREE_B = (3 << 4) | (13);
    // 
    const static uint8_t WALL_R = 0;
    const static uint8_t WALL_G = 0;
    const static uint8_t WALL_B = 0;
    // 
    //const static uint8_t DILATE_R = 255;
    //const static uint8_t DILATE_G = 216;
    //const static uint8_t DILATE_B = 0;
    const static uint8_t DILATE_R = 127;
    const static uint8_t DILATE_G = 127;
    const static uint8_t DILATE_B = 127;

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

    void callback_img(const sensor_msgs::msg::Image::SharedPtr msg_img)
    {
      cv_bridge::CvImagePtr cv_ptr;
      cv_ptr = cv_bridge::toCvCopy(msg_img, "mono8");

      //cv::imshow("Input", cv_ptr->image);
      //cv::waitKey(100); 

      uint32_t h = msg_img->height;
      uint32_t w = msg_img->width;
      float res = image_resolution;

      std::vector<int8_t> prob8;
      for (int i = 0; i < h; ++i)
      {
        for (int j = 0; j < w; ++j)
        {
          prob8.push_back(cv_ptr->image.at<uint8_t>(h-i-1, j)/2);
        }
      }

      process(prob8, h, w, res, msg_img->header);
    }

    void callback_grid(
        const nav_msgs::msg::OccupancyGrid::SharedPtr msg_grid
        )
    {
      /*Eigen::Matrix4f get_T_matrix(
          msg_grid->info.origin.position.x,
          msg_grid->info.origin.position.y,
          msg_grid->info.origin.position.z,
          msg_grid->info.origin.orientation.x,
          msg_grid->info.origin.orientation.y,
          msg_grid->info.origin.orientation.z,
          msg_grid->info.origin.orientation.w
          );*/

      uint32_t h = msg_grid->info.height;
      uint32_t w = msg_grid->info.width;
      float res = msg_grid->info.resolution;
      
      std::vector<int8_t> prob8;
      for (int i = 0; i < h*w; ++i)
        prob8.push_back(msg_grid->data[i]);

      process(prob8, h, w, res, msg_grid->header);
    }

      void process(
          std::vector<int8_t>& prob8,
          uint32_t h, uint32_t w, float res,
          std_msgs::msg::Header& header
          )
      {

      std::vector<uint8_t> raw;
      for (int i = 0; i < h*w; ++i)
      {
        if (prob8[i] > thr_obst*127) raw.push_back(255);
        else raw.push_back(0);
      }

      cv::Mat raw_obst_image(
        h,
        w,
        CV_8U,
        raw.data()
        );

      cv::Mat kernel = cv::getStructuringElement(
          cv::MORPH_ELLIPSE,
          cv::Size( 2*dilation_size + 1, 2*dilation_size+1 ),
          cv::Point( dilation_size, dilation_size )
          );

      cv::Mat inflated_obst_image;
      cv::dilate(raw_obst_image, inflated_obst_image, kernel);
     
      //cv::imshow("Inlated obstacles", inflated_obst_image);
      //cv::waitKey(100); 


      cv::Mat distance_image;
      cv::distanceTransform(255-inflated_obst_image, distance_image, cv::DIST_L2, 3, CV_32F);
      distance_image = distance_image  * res;

      cv::threshold(distance_image, distance_image, max_dst, max_dst, cv::THRESH_TRUNC);
      cv::normalize(distance_image, distance_image, 0, 255.0, cv::NORM_MINMAX);


      cv::Mat distance_image_8uc1;
      distance_image.convertTo(distance_image_8uc1, CV_8UC1);

      cv::Mat distance_cmap_image;
      cv::applyColorMap(255 - distance_image_8uc1, distance_cmap_image, cv::COLORMAP_JET);

      //cv::imshow("Distance", distance_cmap_image);
      //cv::waitKey(100); 

      std::vector<Eigen::Vector4f> pts;
      std::vector<Eigen::Vector<uint8_t,3>> colors;
      for (int i = 0; i < h*w; ++i)
      {
        int _x = (i % w);
        int _y = (i / w);
        float x = _x*res;
        float y = _y*res;
        float p_occ = float(prob8.at(i)) / 127;
        
        if (p_occ > thr_obst)
        {
          for (float z = 0; z <= wall_height; z += res)
          {
            pts.push_back(Eigen::Vector4f(x, y, z, 1.));
            Eigen::Vector<uint8_t,3> rgb;
            rgb(0) = WallVisNode::WALL_R;
            rgb(1) = WallVisNode::WALL_G;
            rgb(2) = WallVisNode::WALL_B;
            colors.push_back(rgb);
          }
        }
        else if (inflated_obst_image.at<uint8_t>(_y, _x) > thr_obst*255)
        {
          for (float z = 0; z <= clearance_height ; z += res)
          {
            pts.push_back(Eigen::Vector4f(x, y, z, .5));
            Eigen::Vector<uint8_t,3> rgb;
            rgb(0) = WallVisNode::DILATE_R;
            rgb(1) = WallVisNode::DILATE_G;
            rgb(2) = WallVisNode::DILATE_B;
            colors.push_back(rgb);
          }
        }
        else if (p_occ < thr_free)
        {
          pts.push_back(Eigen::Vector4f(x, y, 0., 0.));
          Eigen::Vector<uint8_t,3> rgb;
          //rgb << WallVisNode::WALL_R,  WallVisNode::WALL_G,  WallVisNode::WALL_B;
          if (show_dst) 
          {
            cv::Vec3b pixel = distance_cmap_image.at<cv::Vec3b>(_y, _x); 
            rgb(0) = pixel(2);
            rgb(1) = pixel(1);
            rgb(2) = pixel(0);
          }
          else
          {
            rgb(0) = WallVisNode::FREE_R;
            rgb(1) = WallVisNode::FREE_G;
            rgb(2) = WallVisNode::FREE_B;
          }
          colors.push_back(rgb);
        }
      }

      sensor_msgs::msg::PointCloud2 pc2msg;

      //generate x field for rviz
      sensor_msgs::msg::PointField pf_x;
      pf_x.name = "x";
      pf_x.datatype = 7;
      pf_x.offset = 0;
      pf_x.count = 1;
      //generate y field for rviz
      sensor_msgs::msg::PointField pf_y;
      pf_y.name = "y";
      pf_y.datatype = 7;
      pf_y.offset = 4;
      pf_y.count = 1;
      //generate z field for rviz
      sensor_msgs::msg::PointField pf_z;
      pf_z.name = "z";
      pf_z.datatype = 7;
      pf_z.offset = 8;
      pf_z.count = 1;
      //generate rgb field for rviz
      sensor_msgs::msg::PointField pf_rgb;
      pf_rgb.name = "rgb";
      pf_rgb.datatype = 6;
      pf_rgb.offset = 12;
      pf_rgb.count = 1;

      pc2msg.fields.push_back(pf_x); 
      pc2msg.fields.push_back(pf_y); 
      pc2msg.fields.push_back(pf_z); 
      pc2msg.fields.push_back(pf_rgb); 

      
      for (int i = 0; i < pts.size(); ++i)
      {
        float x_val = pts.at(i)(0);
        for (uint8_t * byteptr = (uint8_t *) &x_val; byteptr < ((uint8_t *) &x_val) + 4; byteptr++) pc2msg.data.push_back(*byteptr);
        float y_val = pts.at(i)(1);
        for (uint8_t * byteptr = (uint8_t *) &y_val; byteptr < ((uint8_t *) &y_val) + 4; byteptr++) pc2msg.data.push_back(*byteptr);
        float z_val = pts.at(i)(2);
        for (uint8_t * byteptr = (uint8_t *) &z_val; byteptr < ((uint8_t *) &z_val) + 4; byteptr++) pc2msg.data.push_back(*byteptr);
        uint32_t rgb_val = (colors.at(i)(0) << 16) | (colors.at(i)(1) << 8) | (colors.at(i)(2));
        /*if (pts.at(i)(3) > .99) rgb_val = (WallVisNode::WALL_R << 16) | (WallVisNode::WALL_G << 8) | (WallVisNode::WALL_B);
        else if (pts.at(i)(3) > .49) rgb_val = (WallVisNode::DILATE_R << 16) | (WallVisNode::DILATE_G << 8) | (WallVisNode::DILATE_B);
        else rgb_val = (WallVisNode::FREE_R << 16) | (WallVisNode::FREE_G << 8) | (WallVisNode::FREE_B);*/
        for (uint8_t * byteptr = (uint8_t *) &rgb_val; byteptr < ((uint8_t *) &rgb_val) + 4; byteptr++) pc2msg.data.push_back(*byteptr);
      }


      pc2msg.point_step = 3*4+1*4;
      pc2msg.height = 1;
      pc2msg.width = pts.size();
      pc2msg.row_step = pc2msg.width*pc2msg.point_step;

      pc2msg.header = header;

      pub_pcd->publish(pc2msg);
    }

  public:
    WallVisNode() : Node("gridmap")
    {

      image_resolution = this->declare_parameter("image_resolution", 0.1);
      RCLCPP_INFO(this->get_logger(), "image_resolution: %f", image_resolution);

      wall_height = this->declare_parameter("wall_height", 1.0);
      RCLCPP_INFO(this->get_logger(), "wall_height: %f", wall_height);

      clearance_height = this->declare_parameter("clearance_height", 0.0);
      RCLCPP_INFO(this->get_logger(), "clearance_height: %f", clearance_height);

      thr_free = this->declare_parameter("thr_free", 0.33);
      RCLCPP_INFO(this->get_logger(), "thr_free: %f", thr_free);

      thr_obst = this->declare_parameter("thr_obst", 0.66);
      RCLCPP_INFO(this->get_logger(), "thr_obst: %f", thr_obst);

      dilation_size = this->declare_parameter("dilation_size", 7);
      RCLCPP_INFO(this->get_logger(), "dilation_size: %d", dilation_size);

      show_dst = this->declare_parameter("show_dst", false);
      RCLCPP_INFO(this->get_logger(), "show_dst: %x", show_dst);
      
      max_dst = this->declare_parameter("max_dst", 5.0);
      RCLCPP_INFO(this->get_logger(), "max_dst: %f", max_dst);

      pub_pcd = this->create_publisher<sensor_msgs::msg::PointCloud2>(
		      "pcd",
		      rclcpp::QoS(rclcpp::SensorDataQoS())
		      );
      
      sub_grid = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
		      "gridmap",
		      rclcpp::QoS(rclcpp::SensorDataQoS()),
		      std::bind(&WallVisNode::callback_grid, this, std::placeholders::_1)
		      );

      sub_img = this->create_subscription<sensor_msgs::msg::Image>(
		      "imgmap",
		      rclcpp::QoS(rclcpp::SensorDataQoS()),
		      std::bind(&WallVisNode::callback_img, this, std::placeholders::_1)
		      );

    }
};


int init_wall_vis_node(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<WallVisNode>());
  rclcpp::shutdown();
  return 0;
}

#endif
