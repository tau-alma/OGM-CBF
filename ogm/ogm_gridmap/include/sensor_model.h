#ifndef __OGM_GRIDMAP_SENSOR_MODEL__
#define __OGM_GRIDMAP_SENSOR_MODEL__

class SensorModel
{
  public:

    float hit_dist;
    
    float min_dist;
    float low_crop_dist;
    
    float max_dist;
    float high_crop_dist;

    SensorModel(
        float _hit_dist,
        float _min_dist, float _low_crop_dist,
        float _max_dist, float _high_crop_dist)
    {
      hit_dist = _hit_dist;

      min_dist = _min_dist;
      low_crop_dist = _low_crop_dist;
      
      max_dist = _max_dist;
      high_crop_dist = _high_crop_dist;
    }
};

#endif
