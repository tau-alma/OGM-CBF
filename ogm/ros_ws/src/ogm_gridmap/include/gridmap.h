#ifndef __OGM_GRIDMAP_GRIDMAP__
#define __OGM_GRIDMAP_GRIDMAP__

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "sensor_model.h"

class Gridmap
{
  private:
    uint32_t h;
    uint32_t w;
    float origin_x;
    float origin_y;
    float cell_size;

    float s_target;

    Eigen::MatrixXf map;

    std::pair<int, int> coord2sub(float x, float y)
    {
      int i = std::floor((y - origin_y) / cell_size);
      int j = std::floor((x - origin_x) / cell_size);
      return std::make_pair(i,j);
    }

    bool is_in_map(int i, int j)
    {
      if (i >= 0 && i < h && j >= 0 && j < w)
        return true;
      return false;
    }

    void update_occ(float x, float y)
    {
      int i, j;
      std::tie(i,j) = coord2sub(x, y);
      if (is_in_map(i, j))
      {
        float p_z_occ = (1 + s_target) / 2;
        float p_z_free = (1 - s_target) / 2;
        float p_m = map(i,j);
        map(i,j) = p_z_occ*p_m / (p_z_occ*p_m + p_z_free*p_m);
      }
    }

    void update_free(float x, float y)
    {
      int i, j;
      std::tie(i,j) = coord2sub(x, y);
      if (is_in_map(i, j))
      {
        float p_z_occ = (1 + s_target) / 2;
        float p_z_free = (1 - s_target) / 2;
        float p_m = map(i,j);
        map(i,j) = p_z_free*p_m / (p_z_occ*p_m + p_z_free*p_m);
      }
    }

    std::vector<std::pair<float,float>> bresenham(
        float sensor_x, float sensor_y,
        float obs_x, float obs_y)
    {
      float mx = std::abs(obs_x - sensor_x);
      float my = std::abs(obs_y - sensor_y);

      int sx = (obs_x - sensor_x > 0) ? 1 : -1;
      int sy = (obs_y - sensor_y > 0) ? 1 : -1;

      float x = sensor_x;
      float y = sensor_y;

      std::vector<std::pair<float,float>> pts;

      if (mx > my)
      {
        float e = mx / 2;
        while (x*sx <= obs_x*sx)
        {
          pts.push_back(std::make_pair(x,y));
          e -= my;
          if (e < 0)
          {
            y += sy * cell_size;
            e += mx;
          }
          x += sx * cell_size;
        }
      }
      else
      {
        float e = my / 2;
        while (y*sy <= obs_y*sy)
        {
          pts.push_back(std::make_pair(x,y));
          e -= mx;
          if (e < 0)
          {
            x += sx * cell_size;
            e += my;
          }
          y += sy * cell_size;
        }
      }

      return pts;
    }

    void update(
        SensorModel& sensor_model,
        float sensor_x, float sensor_y,
        float obs_x, float obs_y)
    {

      float dst_sensor_obs = std::sqrt((obs_x - sensor_x)*(obs_x - sensor_x) + (obs_y - sensor_y)*(obs_y - sensor_y));

      std::vector<std::pair<float,float>> pts = bresenham(sensor_x, sensor_y, obs_x, obs_y);

      for (std::pair<float,float> pt : pts)
      {
        float x,y;
        std::tie(x,y) = pt;

        float dst_sensor = std::sqrt((x - sensor_x)*(x - sensor_x) + (y - sensor_y)*(y - sensor_y));
        float dst_obs = std::sqrt((x - obs_x)*(x - obs_x) + (y - obs_y)*(y - obs_y));

        if ( dst_sensor < sensor_model.low_crop_dist 
            || dst_sensor > sensor_model.high_crop_dist 
            || dst_sensor_obs < sensor_model.min_dist
            || dst_sensor_obs > sensor_model.max_dist)
        {}
        else if ( dst_obs < sensor_model.hit_dist )
        {
          update_occ(x,y);
        }
        else
        {
          update_free(x,y);
        }
      }
    }


  public:
    Gridmap(int _h, int _w, float _cell_size, float _s_target)
    {
      h = _h;
      w = _w;
      origin_x = 0.;
      origin_y = 0.;
      cell_size = _cell_size;
      s_target = _s_target;

      map = Eigen::MatrixXf::Constant(h,w,0.5);
    }

    void update(
        SensorModel& sensor_model,
        pcl::PointCloud<pcl::PointXYZI>& pcd
        )
    {
      pcl::PointXYZI ref_pt;
      for (pcl::PointXYZI p : pcd)
      {
        if (p.intensity == 0.)
        {
          ref_pt = p;
        }
        else
        {
          update(
              sensor_model,
              ref_pt.x, ref_pt.y,
              p.x, p.y);
        }
      }
    }

    float get_origin_x()
    {
      return origin_x;
    }

    float get_origin_y()
    {
      return origin_y;
    }

    float get_cell_size()
    {
      return cell_size;
    }

    uint32_t get_height()
    {
      return h;
    }

    uint32_t get_width()
    {
      return w;
    }

    std::vector<int8_t> report_int8()
    {
      std::vector<int8_t> ret;
      for (int i = 0; i < h; ++i)
      {
        for (int j = 0; j < w; ++j)
        {
          int8_t val = std::floor(map(i,j)*127);
          ret.push_back(val);
        }
      }

      return ret;
    }
   
    std::vector<uint8_t> report_uint8()
    {
      std::vector<uint8_t> ret;
      for (int i = 0; i < h; ++i)
      {
        for (int j = 0; j < w; ++j)
        {
          uint8_t val = std::floor(map(i,j)*255);
          ret.push_back(val);
        }
      }

      return ret;
    }
};

#endif
