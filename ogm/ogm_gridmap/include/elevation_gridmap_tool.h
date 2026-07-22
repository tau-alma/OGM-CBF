#ifndef __OGM_GRIDMAP_ELEVATION_GRIDMAP_TOOL__
#define __OGM_GRIDMAP_ELEVATION_GRIDMAP_TOOL__

#include <cmath>
#include <chrono>   
#include <opencv2/opencv.hpp>   
#include <cv_bridge/cv_bridge.h>
#include <string>
#include <ryml.hpp>
#include <ryml_std.hpp>
#include<iostream>
#include<fstream>
#include<sstream>
#include <dirent.h>

#include "elevation_gridmap.h"

using namespace std::chrono_literals;

class Entry
{
  public:
    double ts;

    bool carries_pcd_ref;
    std::string pcd_ref;

    Entry(double _ts)
    {
      ts = _ts;
      carries_pcd_ref = false;
    }

    void set_pcd_ref(std::string _pcd_ref)
    {
      pcd_ref = _pcd_ref;
      carries_pcd_ref = true;
    }
};
struct EntryComp
{
    bool operator() (Entry const& e1, Entry const& e2)
    {
        return e1.ts < e2.ts;
    }
};

class ElevationGridmapTool 
{
  private:

    float occgrid_vis_z;
    std::string map_frame;

    // cellmap config parameters
    float cellsize;
    uint32_t height;
    uint32_t width;
    int pt_step;
    float traversable_slope;
    float traversability_r;
    float traversable_z;
    float crop_z_max;
    float cell_z_var;
    float cell_sensor_var;
    float cell_system_var;
    bool cell_update_var;

    // clearance parameters
    float clearance_thr_pos_low;
    float clearance_thr_pos_high;
    float clearance_thr_dir_low;
    float clearance_thr_dir_high;
    double max_clearance_age;
    double clearance_ts; 
    
    bool do_update;
    bool do_reset;
    bool do_clear_pos;
    bool invert_clear_pos;
    bool do_clear_dir;
    bool invert_clear_dir;

    std::priority_queue<Entry, std::vector<Entry>, EntryComp> pq;

    std::shared_ptr<ElevationGridmap> gridmap;

  public:

    void load_from_pcd_folder(std::string fldr_pth)
    {
      // fldr_pth is a path to a folder with a set of *pcd files
      // iterate over the files pcd files in fldr_pth
      std::string dir = fldr_pth;
      if (dir.back() != '/') {
        dir += '/';
      }
      
      DIR* d = opendir(dir.c_str());
      if (!d) {
        return;
      }
      
      struct dirent* entry;
      while ((entry = readdir(d)) != nullptr)
      {
        std::string filename = entry->d_name;
        if (filename == "." || filename == "..") continue;
        
        // Check if file has .pcd extension
        if ((filename.length() >= 4) && (filename.substr(filename.size() - 4) == ".pcd"))
        {
          std::string file_path = dir + filename;

          double ts = std::stold(filename.substr(0, filename.size() - 4)) / 1e9;
          printf("  Processing pcd file: %s at time %lf\n", file_path.c_str(), ts);

          Entry e(ts);
          e.set_pcd_ref(file_path);

          pq.push(e);

          
          // TODO: Load pcd into 32FPointPCLCloudPtr and call pointcloud_to_elev_gridmap(file_path);
        }
      }
      closedir(d);
    }


    void process_queue()
    {
      printf("Processing queue %lu\n",pq.size());
      while (!pq.empty())
      {
        Entry e = pq.top();
        pq.pop();
        if (e.carries_pcd_ref) printf("[%lf] pcd %s\n", e.ts, e.pcd_ref.c_str());
      }
    }

    ElevationGridmapTool(std::string config_pth) 
    {
      if (!config_pth.empty())
      {
        std::ifstream file(config_pth.c_str());
        std::string contents;
        if (file)
        {
          std::ostringstream ss;
          ss<<file.rdbuf();
          contents = ss.str();
        }

        ryml::Tree tree = ryml::parse_in_arena(ryml::to_csubstr(contents));
        ryml::ConstNodeRef root = tree.crootref();

        // clearance parameters
        clearance_thr_pos_low = 0.5f;
        if (root.has_child("clearance_thr_pos_low")) root["clearance_thr_pos_low"].load(&clearance_thr_pos_low);
        printf("  clearance_thr_pos_low: %f\n", clearance_thr_pos_low);

        clearance_thr_pos_high = 0.5f;
        if (root.has_child("clearance_thr_pos_high")) root["clearance_thr_pos_high"].load(&clearance_thr_pos_high);
        printf("  clearance_thr_pos_high: %f\n", clearance_thr_pos_high);

        clearance_thr_dir_low = 0.5f;
        if (root.has_child("clearance_thr_dir_low")) root["clearance_thr_dir_low"].load(&clearance_thr_dir_low);
        printf("  clearance_thr_dir_low: %f\n", clearance_thr_dir_low);

        clearance_thr_dir_high = 0.5f;
        if (root.has_child("clearance_thr_dir_high")) root["clearance_thr_dir_high"].load(&clearance_thr_dir_high);
        printf("  clearance_thr_dir_high: %f\n", clearance_thr_dir_high);

        max_clearance_age = -1.0;
        if (root.has_child("max_clearance_age")) root["max_clearance_age"].load(&max_clearance_age);
        printf("  max_clearance_age: %f\n", max_clearance_age);

        do_reset = false;
        if (root.has_child("do_reset")) root["do_reset"].load(&do_reset);
        printf("  do_reset: %d\n", do_reset);

        do_clear_pos = false;
        if (root.has_child("do_clear_pos")) root["do_clear_pos"].load(&do_clear_pos);
        printf("  do_clear_pos: %d\n", do_clear_pos);

        invert_clear_pos = false;
        if (root.has_child("invert_clear_pos")) root["invert_clear_pos"].load(&invert_clear_pos);
        printf("  invert_clear_pos: %d\n", invert_clear_pos);

        do_clear_dir = false;
        if (root.has_child("do_clear_dir")) root["do_clear_dir"].load(&do_clear_dir);
        printf("  do_clear_dir: %d\n", do_clear_dir);

        invert_clear_dir = false;
        if (root.has_child("invert_clear_dir")) root["invert_clear_dir"].load(&invert_clear_dir);
        printf("  invert_clear_dir: %d\n", invert_clear_dir);

        // cellmap config parameters
        cellsize = 0.025f;
        if (root.has_child("cellsize")) root["cellsize"].load(&cellsize);
        printf("  cellsize: %f\n", cellsize);

        // Height and width are read as floats from config, but the class stores them as uint32_t (cell count)
        float height_float = 20.0f;
        float width_float = 20.0f;
        if (root.has_child("height")) root["height"].load(&height_float);
        if (root.has_child("width")) root["width"].load(&width_float);
        
        // Convert float dimensions to number of cells (uint32_t)
        height = uint32_t(height_float / cellsize);
        width = uint32_t(width_float / cellsize);
        
        printf("  height_float: %f -> height_cells: %u\n", height_float, height);
        printf("  width_float: %f -> width_cells: %u\n", width_float, width);

        pt_step = 1;
        if (root.has_child("pt_step")) root["pt_step"].load(&pt_step);
        printf("  pt_step: %d\n", pt_step);

        traversable_slope = 0.78f;
        if (root.has_child("traversable_slope")) root["traversable_slope"].load(&traversable_slope);
        printf("  traversable_slope: %f\n", traversable_slope);

        traversability_r = 0.5f;
        if (root.has_child("traversability_r")) root["traversability_r"].load(&traversability_r);
        printf("  traversability_r: %f\n", traversability_r);

        traversable_z = 1e6f;
        if (root.has_child("traversable_z")) root["traversable_z"].load(&traversable_z);
        printf("  traversable_z: %f\n", traversable_z);

        crop_z_max = 1e6f;
        if (root.has_child("crop_z_max")) root["crop_z_max"].load(&crop_z_max);
        printf("  crop_z_max: %f\n", crop_z_max);

        cell_z_var = 1.0f;
        if (root.has_child("cell_z_var")) root["cell_z_var"].load(&cell_z_var);
        printf("  cell_z_var: %f\n", cell_z_var);

        cell_sensor_var = 0.1f;
        if (root.has_child("cell_sensor_var")) root["cell_sensor_var"].load(&cell_sensor_var);
        printf("  cell_sensor_var: %f\n", cell_sensor_var);

        cell_system_var = 0.01f;
        if (root.has_child("cell_system_var")) root["cell_system_var"].load(&cell_system_var);
        printf("  cell_system_var: %f\n", cell_system_var);

        cell_update_var = false;
        if (root.has_child("cell_update_var")) root["cell_update_var"].load(&cell_update_var);
        printf("  cell_update_var: %d\n", cell_update_var);

        printf("clearance params:\n");
        printf("  do_reset: %d\n", do_reset);
        printf("  do_clear_pos: %d\n", do_clear_pos);
        printf("  invert_clear_pos: %d\n", invert_clear_pos);
        printf("  do_clear_dir: %d\n", do_clear_dir);
        printf("  invert_clear_dir: %d\n", invert_clear_dir);
        printf("  max_clearance_age: %f\n", max_clearance_age);
        printf("  occgrid_vis_z: %f\n", occgrid_vis_z);
        printf("  map_frame: %s\n", map_frame.c_str());
        printf("  clearance_thr_pos_low: %f\n", clearance_thr_pos_low);
        printf("  clearance_thr_pos_high: %f\n", clearance_thr_pos_high);
        printf("  clearance_thr_dir_low: %f\n", clearance_thr_dir_low);
        printf("  clearance_thr_dir_high: %f\n", clearance_thr_dir_high);

        // init gridmap
        gridmap = std::make_shared<ElevationGridmap>(ElevationGridmap(
            cellsize,
            height, width,
            pt_step,
            traversable_slope,
            traversability_r,
            traversable_z,
            crop_z_max,
            cell_z_var,
            cell_sensor_var,
            cell_system_var,
            cell_update_var)
            );

      }
      else
      {
        printf("Warning: Config file %s is not found or invalid\n", config_pth.c_str());
      }
    }
};


int init_elevation_gridmap_tool(int argc, char** argv)
{
  std::string config_path = argv[1];
  printf("Config path: %s\n", config_path.c_str());
  ElevationGridmapTool tool(config_path); 

  for (int ic = 2; ic < argc; ++ic)
  {
    std::string fldr_path = argv[ic];
    printf("Processing data folder '%s'\n",fldr_path.c_str());
    tool.load_from_pcd_folder(fldr_path);
    printf("done'\n");
  }

  tool.process_queue();

  return 0;
}

#endif
