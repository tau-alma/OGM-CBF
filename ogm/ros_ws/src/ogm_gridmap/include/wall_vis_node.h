#ifndef __OGM_GRIDMAP_WALL_VIS_NODE__
#define __OGM_GRIDMAP_WALL_VIS_NODE__

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <opencv2/opencv.hpp>

class WallVisNode  : public rclcpp::Node
{
  private:
    
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_grid;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_pcd;

    float wall_height;
    float thr_free;
    float thr_obst;

    int dilation_size;

    // FF6F3C
    const static uint8_t FREE_R = 255;
    const static uint8_t FREE_G = (6 << 4) | (15);
    const static uint8_t FREE_B = (3 << 4) | (13);
    // 
    const static uint8_t WALL_R = 0;
    const static uint8_t WALL_G = 0;
    const static uint8_t WALL_B = 0;
    // 
    const static uint8_t DILATE_R = 255;
    const static uint8_t DILATE_G = 216;
    const static uint8_t DILATE_B = 0;

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
      

      std::vector<uint8_t> raw;
      for (int i = 0; i < h*w; ++i)
      {
        if (msg_grid->data[i] > thr_obst*127) raw.push_back(255);
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

      std::vector<Eigen::Vector4f> pts;
      for (int i = 0; i < h*w; ++i)
      {
        int _x = (i % w);
        int _y = (i / w);
        float x = _x*res;
        float y = _y*res;
        float p_occ = float(msg_grid->data[i]) / 127;
        
        if (p_occ > thr_obst)
        {
          for (float z = 0; z <= wall_height; z += res)
            pts.push_back(Eigen::Vector4f(x, y, z, 1.));
        }
        else if (inflated_obst_image.at<uint8_t>(_y, _x) > thr_obst*255)
        {
          for (float z = 0; z <= wall_height ; z += res)
            pts.push_back(Eigen::Vector4f(x, y, z, .5));
        }
        else if (p_occ < thr_free)
        {
          pts.push_back(Eigen::Vector4f(x, y, 0., 0.));
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
        uint32_t rgb_val;
        if (pts.at(i)(3) > .99) rgb_val = (WallVisNode::WALL_R << 16) | (WallVisNode::WALL_G << 8) | (WallVisNode::WALL_B);
        else if (pts.at(i)(3) > .49) rgb_val = (WallVisNode::DILATE_R << 16) | (WallVisNode::DILATE_G << 8) | (WallVisNode::DILATE_B);
        else rgb_val = (WallVisNode::FREE_R << 16) | (WallVisNode::FREE_G << 8) | (WallVisNode::FREE_B);
        for (uint8_t * byteptr = (uint8_t *) &rgb_val; byteptr < ((uint8_t *) &rgb_val) + 4; byteptr++) pc2msg.data.push_back(*byteptr);
      }


      pc2msg.point_step = 3*4+1*4;
      pc2msg.height = 1;
      pc2msg.width = pts.size();
      pc2msg.row_step = pc2msg.width*pc2msg.point_step;

      pc2msg.header = msg_grid->header;

      pub_pcd->publish(pc2msg);
    }

  public:
    WallVisNode() : Node("gridmap")
    {

      wall_height = this->declare_parameter("wall_height", 1.0);
      RCLCPP_INFO(this->get_logger(), "wall_height: %f", wall_height);

      thr_free = this->declare_parameter("thr_free", 0.33);
      RCLCPP_INFO(this->get_logger(), "thr_free: %f", thr_free);

      thr_obst = this->declare_parameter("thr_obst", 0.66);
      RCLCPP_INFO(this->get_logger(), "thr_obst: %f", thr_obst);

      dilation_size = this->declare_parameter("dilation_size", 7);
      RCLCPP_INFO(this->get_logger(), "dilation_size: %d", dilation_size);

      pub_pcd = this->create_publisher<sensor_msgs::msg::PointCloud2>(
		      "pcd",
		      rclcpp::QoS(rclcpp::SensorDataQoS())
		      );
      
      sub_grid = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
		      "gridmap",
		      rclcpp::QoS(rclcpp::SensorDataQoS()),
		      std::bind(&WallVisNode::callback_grid, this, std::placeholders::_1)
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
