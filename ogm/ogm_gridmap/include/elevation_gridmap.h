#ifndef __OGM_GRIDMAP_ELEVATION_GRIDMAP__
#define __OGM_GRIDMAP_ELEVATION_GRIDMAP__

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <cmath>

class Cell
{

  public:
    static const uint8_t UNKNOWN = 0;
    static const uint8_t OBSERVED = 1;

    uint8_t type;
    float z;

    Cell()
    {
      type = UNKNOWN;
      z = 0.;
    }

    void update(float _z)
    {
      type = OBSERVED;
      z = _z;
    }
};


class KalmanCell : public Cell
{
  public : 
    float z_var;
    float sensor_var;
    float system_var;
    bool update_var;
    float h;

    KalmanCell() : Cell()
    {
      z_var = -1.0;
      sensor_var = -1.0;
      system_var = -1.0;
      h = -1.0;
    }

    void check_kalman_init(float _z_var, float _sensor_var, float _system_var, bool _update_var)
    {
      if (h < 0)
      {
        z_var = _z_var;
        sensor_var = _sensor_var;
        system_var = _system_var;
        update_var = _update_var;
        h = 1.0;
      }
    }

    void update(float _z)
    {
      if (type == Cell::UNKNOWN)
      {
        type = Cell::OBSERVED;
        z = _z;
      }
      else
      {
        float pred_cov = z_var + system_var;
        float gain = z_var * h / (sensor_var + pred_cov * h);
        z = z + gain * (_z - z * h);
        z_var = (1 - gain * h) * z_var  ;
      }
    }
};


class KalmanCellOccupancy : public KalmanCell
{
  public : 
    static constexpr float UNKNOWN_P = 0.5;
    static constexpr float UNSET_P = -1.0;
    
    float p_obs;

    KalmanCellOccupancy() : KalmanCell()
    {
      p_obs = UNKNOWN_P;
    }


    void update_occupancy(float _p_obs)
    {
      p_obs = _p_obs;
    }


};


class ElevationGridmap
{
  private:

    uint32_t height;
    uint32_t width;
    float cellsize;
    float origin_x;
    float origin_y;

    float traversable_slope;
    float traversability_r;
    int traversability_nbh;
    float traversable_z;
    
    float crop_z_max;

    Eigen::Matrix<KalmanCellOccupancy, Eigen::Dynamic, Eigen::Dynamic> gridmap;

    float clearance_x, clearance_y;
    float clearance_thr;
    
    float cell_z_var;
    float cell_sensor_var;
    float cell_system_var;
    bool cell_update_var;


    std::pair<int, int> coord2sub(float _x, float _y)
    {
      int i = std::round((_x - origin_x) / cellsize); 
      int j = std::round((_y - origin_y) / cellsize);
      return std::make_pair(i, j);
    }

    std::pair<float, float> sub2coord(int _i, int _j)
    {
      float x = _i*cellsize + origin_x;
      float y = _j*cellsize + origin_y;
      return std::make_pair(x, y);
    }

    bool is_in_bounds(int i, int j)
    {
      bool in_bounds = true;
      in_bounds &= 0 < i; 
      in_bounds &= i < (int64_t)width; 
      in_bounds &= 0 < j; 
      in_bounds &= j < (int64_t)height;
      return in_bounds; 
    }

    std::vector<std::pair<int, int>> nbh(int _i, int _j, int d, int d_step)
    {
      std::vector<std::pair<int, int>> r;
      for (int i = _i-d; i <= _i+d; i += d_step)
      {
         for (int j = _j-d; j <= _j+d; j += d_step)
         {
            if (is_in_bounds(i, j) && gridmap(i, j).type != KalmanCellOccupancy::UNKNOWN)
            {
              r.push_back(std::make_pair(i, j));
            } 
         }
      }
      return r;
    }
    
    bool is_in_clearance(pcl::PointXYZ& p)
    {
      float d = std::sqrt((clearance_x - p.x)*(clearance_x - p.x) + (clearance_y - p.y)*(clearance_y - p.y));
      return d < clearance_thr;
    }

  public:  

    void reset()
    {
      gridmap = Eigen::Matrix<KalmanCellOccupancy, Eigen::Dynamic, Eigen::Dynamic>::Constant(width, height, KalmanCellOccupancy());
    }

    void update_clearance(float _clearance_x, float _clearance_y, float _clearance_thr)
    {
      clearance_x = _clearance_x;
      clearance_y = _clearance_y;
      clearance_thr = _clearance_thr;
    }

    void update(pcl::PointCloud<pcl::PointXYZ>& xyz)
    {
      std::list<std::pair<int, int>> p_update_candidates;

      for (pcl::PointXYZ pt : xyz.points)
      {
        float z = pt.z;
        int i, j;
        std::tie(i, j) = coord2sub(pt.x, pt.y);
        //RCLCPP_INFO(this->get_logger(), "%d %d %f", i, j, pt.z);
        if (is_in_bounds(i, j) 
	          && !is_in_clearance(pt)
	          && (z < crop_z_max))
        {
          gridmap(i, j).check_kalman_init(cell_z_var, cell_sensor_var, cell_system_var, cell_update_var);
          gridmap(i ,j).update(z);
          for (std::pair<int,int> c : nbh(i,j,traversability_nbh,traversability_nbh))
          {
            gridmap(c.first, c.second).update_occupancy(KalmanCellOccupancy::UNSET_P);
            p_update_candidates.push_back(c);
          }
        } 
      }

      for (std::pair<int,int> cand : p_update_candidates)
      {
        if (gridmap(cand.first, cand.second).type != KalmanCellOccupancy::UNKNOWN
            && gridmap(cand.first, cand.second).p_obs == KalmanCellOccupancy::UNSET_P)
        {
          float x, y, z, p_obs;
          z = gridmap(cand.first, cand.second).z;
          std::tie(x, y) = sub2coord(cand.first, cand.second);
          //
          float ground = z;
          for (std::pair<int,int> c : nbh(cand.first,cand.second,traversability_nbh,traversability_nbh))
          {
            if (gridmap(c.first, c.second).z < ground)
            {
              ground = gridmap(c.first, c.second).z;
            }
          }
          //
          if ( gridmap(cand.first,cand.second).z > traversable_z ) p_obs = 1.;
          else if ( atan2f(z - ground, cellsize*traversability_nbh) > traversable_slope ) p_obs = 1.;
          else p_obs = 0;
          gridmap(cand.first, cand.second).update_occupancy(p_obs);
        }
      }
    }


    pcl::PointCloud<pcl::PointXYZI> report_3d(int _ground_nbh_size)
    {
      pcl::PointCloud<pcl::PointXYZI> xyz;

      for (uint32_t i = 0; i < width; ++i)
      {
        for (uint32_t j = 0; j < height; ++j)
        {
          if (gridmap(i, j).type != KalmanCellOccupancy::UNKNOWN)
          {
            float x, y, z, p_obs, vis_ground;
            z = gridmap(i, j).z;
            p_obs = gridmap(i, j).p_obs;
            std::tie(x, y) = sub2coord(i, j);
            //
            vis_ground = z - cellsize / 2;
            for (std::pair<int,int> c : nbh(i,j,_ground_nbh_size,1))
              if (gridmap(c.first, c.second).z < vis_ground)
                vis_ground = gridmap(c.first, c.second).z;
            //
            // 
            for (float z_vis = z; z_vis > vis_ground; z_vis -= cellsize)
            {
                //pcl::PointXYZI pt_vis(x, y, z_vis, p_obs);
                pcl::PointXYZI pt_vis;
		pt_vis.x = x;
		pt_vis.y = y;
		pt_vis.z = z_vis;
		pt_vis.intensity = p_obs;
                xyz.push_back(pt_vis);
            }
          }
        }
      }

      return xyz;
    }

    std::vector<int8_t> report_2d_int8()
    {
      std::vector<int8_t> ret;

      for (uint32_t j = 0; j < height; ++j)
      {
        for (uint32_t i = 0; i < width; ++i)
        {
          int8_t val = std::floor(gridmap(i,j).p_obs*127);
          ret.push_back(val);
        }
      }
      return ret;
    }

    std::vector<uint8_t> report_2d_uint8()
    {
      std::vector<uint8_t> ret;

      for (uint32_t j = 0; j < height; ++j)
      {
        for (uint32_t i = 0; i < width; ++i)
        {
          uint8_t val = std::floor(gridmap(i,j).p_obs*255);
          ret.push_back(val);
        }
      }
      return ret;
    }

    float get_cellsize()
    {
      return cellsize;
    }

    float get_width()
    {
      return width;
    }

    float get_height()
    {
      return height;
    }

    float get_origin_x()
    {
      return origin_x;
    }

    float get_origin_y()
    {
      return origin_y;
    }

    ElevationGridmap(
        float _cellsize,
        uint32_t _height, uint32_t _width,
        float _traversable_slope,
        float _traversability_r,
        float _traversable_z,
        float _crop_z_max,
        float _cell_z_var,
        float _cell_sensor_var,
        float _cell_system_var,
        bool _cell_update_var
        )
    {
      cellsize = _cellsize;
      traversable_slope = _traversable_slope;
      traversability_r = _traversability_r;
      traversability_nbh = traversability_r / cellsize;
      traversable_z = _traversable_z;
      crop_z_max = _crop_z_max;

      height = _height;
      width = _width;

      origin_x = 0.;
      origin_y = 0.;

      clearance_x = 0.;
      clearance_y = 0.;
      clearance_thr = 0.;
      
      cell_z_var = _cell_z_var;
      cell_sensor_var = _cell_sensor_var;
      cell_system_var = _cell_system_var;
      cell_update_var = _cell_update_var;

      reset();
    }

};

#endif

