#ifndef __OGM_GRIDMAP_SENSOR_MODEL__
#define __OGM_GRIDMAP_SENSOR_MODEL__

class SensorModel
{
  public:

    float hit_dist;
    
    float min_dist;
    float max_dist;

    bool partial_trace;

    SensorModel(
        float _hit_dist,
        float _min_dist, float _max_dist,
        bool _partial_trace)
    {
      hit_dist = _hit_dist;
      min_dist = _min_dist;
      max_dist = _max_dist;
      partial_trace = _partial_trace;
    }
};

#endif
