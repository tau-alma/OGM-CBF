#ifndef __OGM_GRIDMAP_PT_CLEARANCE__
#define __OGM_GRIDMAP_PT_CLEARANCE__

#include <pcl/point_types.h>

class ClearanceConfig
{
  public:
    bool do_clear = false;
    bool invert = false;
    float low = 0.;
    float high = 0.;
    float x;
    float y;
    float z;

    ClearanceConfig()
    {}

    ClearanceConfig(bool _do_clear, bool _invert, float _low, float _high)
    {
      do_clear = _do_clear;
      invert = _invert;
      low = _low;
      high = _high;
    }
};


class Clearance
{
  private:

    bool is_init = false;

    ClearanceConfig clearance_pos;
    ClearanceConfig clearance_dir;

    float x,y,z,dx,dy,dz;

    bool in_pos(pcl::PointXYZ& p)
    {
      float dst_sign = clearance_pos.high < 0 ? -1 : 1;
      float dst_abs = std::sqrt(
          (x - p.x)*(x - p.x) + 
          (y - p.y)*(y - p.y) + 
          (z - p.z)*(z - p.z));

      return dst_abs*dst_sign >= clearance_pos.low*dst_sign && dst_abs*dst_sign < clearance_pos.high*dst_sign;
    }

    bool in_dir(pcl::PointXYZ& p)
    {
      float dst_sign = clearance_dir.high < 0 ? -1 : 1;
      float dst_vec = 
        (p.x - x)*dx +
        (p.y - y)*dy +
        (p.z - z)*dz;

      return dst_vec*dst_sign >= clearance_dir.low*dst_sign && dst_vec*dst_sign < clearance_dir.high*dst_sign;
    }

  public:
    bool is_in_clearance(pcl::PointXYZ& p)
    {
      bool ret = false;
      if (is_init)
      {
        bool ret_pos, ret_dir;
        if (clearance_pos.do_clear && clearance_pos.invert) ret_pos = !in_pos(p); 
        else if (clearance_pos.do_clear) ret_pos = in_pos(p); 
        else ret_pos = false;
        if (clearance_dir.do_clear && clearance_dir.invert) ret_dir = !in_dir(p); 
        else if (clearance_dir.do_clear) ret_dir = in_dir(p); 
        else ret_dir = false;
        ret = ret_pos || ret_dir;
      }
      return ret;
    }
    
    Clearance()
    {
      is_init = false;
    }

    Clearance(ClearanceConfig& _clearance_pos, ClearanceConfig& _clearance_dir,
        float _x, float _y, float _z, float _dx, float _dy, float _dz)
    {
      clearance_pos = _clearance_pos;
      clearance_dir = _clearance_dir;
      x = _x;
      y = _y;
      z = _z;
      dx = _dx;
      dy = _dy;
      dz = _dz;
      is_init = true;
    }
};


#endif
