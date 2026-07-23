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
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include "elevation_gridmap.h"

using namespace std::chrono_literals;

class Entry
{
  public:
    double ts;

    bool carries_pcd_ref;
    std::string pcd_ref;

    bool carries_clearance;
    std::shared_ptr<Clearance> clearance_;

    Entry(double _ts)
    {
      ts = _ts;
      carries_pcd_ref = false;
      carries_clearance = false;
    }

    void set_pcd_ref(std::string _pcd_ref)
    {
      pcd_ref = _pcd_ref;
      carries_pcd_ref = true;
    }
    
    void set_clearance(Clearance& _clearance)
    {
      clearance_ = std::make_shared<Clearance>(_clearance);
      carries_clearance = true;
    }
};
struct EntryComp
{
    bool operator() (Entry const& e1, Entry const& e2)
    {
        return e1.ts > e2.ts;
    }
};

class ElevationGridmapTool 
{
  private:

    std::string output_path;

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

    bool do_save_occimg;
    bool do_save_elevimg;
    bool flip_occimg_values;
    bool discretize_elevimg;
    float elevimg_z_res; 

    std::priority_queue<Entry, std::vector<Entry>, EntryComp> pq;

    std::shared_ptr<ElevationGridmap> gridmap;

  public:

    void load_from_odom_file(std::string fl_pth)
    {
      FILE* f = fopen(fl_pth.c_str(), "r");
      char line[1024];

      bool has_tum = false;
      bool has_cmd_mag = false;
      while (fgets(line, sizeof(line), f))
      {
        double ts;
        float x,y,z,qx,qy,qz,qw;
        float magnitude;
        int retval = 0;
        if (line[0] == '#')
        {
          std::string h(line);
          h = h.substr(1,h.size()-2) + ",";
          printf("Header '%s'\n",h.c_str());
          
          has_tum =
            h.find("x,") != std::string::npos && h.find("y,") != std::string::npos && h.find("z,") != std::string::npos
            && h.find("qx,") != std::string::npos && h.find("qx,") != std::string::npos && h.find("qx,") != std::string::npos && h.find("qz,") != std::string::npos;
          has_cmd_mag = h.find("cmd_magnitude,") != std::string::npos;

          printf("TUM %x, CMD Magnitude %x\n",has_tum,has_cmd_mag);

        }
        else if ((retval = sscanf(line, "%lf,%f,%f,%f,%f,%f,%f,%f,%f", &ts, &x, &y, &z, &qx, &qy, &qz, &qw, &magnitude)) >= 8)
        {
          Eigen::Quaternionf q(qw,qx,qy,qz);
          Eigen::Matrix3f R = q.toRotationMatrix();

          if (!has_cmd_mag || retval < 9) magnitude = 1;

          ClearanceConfig cfg_pos(do_clear_pos, invert_clear_pos, clearance_thr_pos_low, clearance_thr_pos_high);
          ClearanceConfig cfg_dir(do_clear_dir, invert_clear_dir, clearance_thr_dir_low, clearance_thr_dir_high);
          Clearance clearance(
              cfg_pos, cfg_dir,
              x,
              y,
              z,
              R(0,0)*magnitude,
              R(1,0)*magnitude,
              R(2,0)*magnitude
              );

          //printf("Processing pos %.2f %.2f %.2f with mag %.2f\n",x,y,z,magnitude);
          Entry e(ts);
          e.set_clearance(clearance);
          pq.push(e);
        }
        else printf("Skip invalid '%s'\n",line);
      }

      if (f) fclose(f);
    }

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
      
      struct dirent* dentry;
      while ((dentry = readdir(d)) != nullptr)
      {
        std::string filename = dentry->d_name;
        if (filename == "." || filename == "..") continue;
        
        // Check if file has .pcd extension
        if ((filename.length() >= 4) && (filename.substr(filename.size() - 4) == ".pcd"))
        {
          std::string file_path = dir + filename;

          double ts = std::stold(filename.substr(0, filename.size() - 4)) / 1e9;
          //printf("  Processing pcd file: %s at time %lf\n", file_path.c_str(), ts);

          Entry e(ts);
          e.set_pcd_ref(file_path);
          pq.push(e);
        }
      }
      closedir(d);
    }

    void save_occimg(double ts)
    {
      std::vector<uint8_t> data = gridmap->report_2d_uint8(flip_occimg_values);
      
      cv::Mat map(
        gridmap->get_height(),
        gridmap->get_width(),
        CV_8U,
        data.data()
        );
      cv::Mat img;
      cv::flip(map, img, 0);

      long unsigned uts = std::round(ts*1e9);
      char buf[128];
      snprintf(buf, sizeof(buf), "%lu.png", uts);
      std::string img_file = output_path + "/occ_" + std::string(buf);
      cv::imwrite(img_file, img);
    }

    void save_elevimg(double ts)
    {

      cv::Mat img;

      if (discretize_elevimg)
      {
        std::vector<int8_t> data = gridmap->report_3d_int8(elevimg_z_res, occgrid_vis_z);
        
        cv::Mat map(
          gridmap->get_height(),
          gridmap->get_width(),
          CV_8S,
          data.data()
          );
        cv::flip(map, img, 0);
      }
      else
      {
        std::vector<float> data = gridmap->report_3d_float(occgrid_vis_z);

        cv::Mat map(
          gridmap->get_height(),
          gridmap->get_width(),
          CV_32FC1,
          data.data()
          );
        cv::flip(map, img, 0);
      }

      long unsigned uts = std::round(ts*1e9);
      char buf[128];
      snprintf(buf, sizeof(buf), "%lu.png", uts);
      std::string img_file = output_path + "/elev_" + std::string(buf);
      cv::imwrite(img_file, img);
    }

    void process_queue()
    {
      printf("Processing queue %lu\n",pq.size());
      while (!pq.empty())
      {
        Entry e = pq.top();
        pq.pop();

        if (e.carries_clearance)
        {
          //printf("[%lf] clearance \n", e.ts);

          Clearance c = *(e.clearance_);
          gridmap->update_clearance(c);
          clearance_ts = e.ts;
        }

        if (e.carries_pcd_ref) 
        {
          double clearance_age = e.ts - clearance_ts;
          //printf("[%lf] pcd ref to %s\n", e.ts, e.pcd_ref.c_str());
          if (do_update && (max_clearance_age < 0. || clearance_age < max_clearance_age))
          {
            pcl::PointCloud<pcl::PointXYZ> xyz;
            pcl::io::loadPCDFile<pcl::PointXYZ>(e.pcd_ref, xyz);
            gridmap->update(xyz);
          }

          if (do_save_occimg) save_occimg(e.ts);
          if (do_save_elevimg) save_elevimg(e.ts);
        }
      }
    }

    ElevationGridmapTool(std::string _config_pth, std::string _output_path) 
    {

      output_path = _output_path;
      std::string mkdir_cmd = "mkdir -p \"" + output_path + "\"";
      system(mkdir_cmd.c_str());

      if (!_config_pth.empty())
      {
        std::ifstream file(_config_pth.c_str());
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
        printf("  max_clearance_age: %lf\n", max_clearance_age);
        clearance_ts = max_clearance_age < 0 ? 0 : -2*max_clearance_age;

        do_update = true;

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

        //output params
        do_save_occimg = false;
        if (root.has_child("do_save_occimg")) root["do_save_occimg"].load(&do_save_occimg);
        printf("  do_save_occimg: %d\n", do_save_occimg);

        do_save_elevimg = false;
        if (root.has_child("do_save_elevimg")) root["do_save_elevimg"].load(&do_save_elevimg);
        printf("  do_save_elevimg: %d\n", do_save_elevimg);

        flip_occimg_values = false;
        if (root.has_child("flip_occimg_values")) root["flip_occimg_values"].load(&flip_occimg_values);
        printf("  flip_occimg_values: %d\n", flip_occimg_values);

        discretize_elevimg = false;
        if (root.has_child("discretize_elevimg")) root["discretize_elevimg"].load(&discretize_elevimg);
        printf("  discretize_elevimg: %d\n", discretize_elevimg);

        elevimg_z_res = 0.02f;
        if (root.has_child("elevimg_z_res")) root["elevimg_z_res"].load(&elevimg_z_res);
        printf("  elevimg_z_res: %f\n", elevimg_z_res);

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
        printf("Warning: Config file %s is not found or invalid\n", _config_pth.c_str());
      }
    }
};


int init_elevation_gridmap_tool(int argc, char** argv)
{
  std::string output_path = argv[1];
  printf("Output path: %s\n", output_path.c_str());
  std::string config_path = argv[2];
  printf("Config path: %s\n", config_path.c_str());
  ElevationGridmapTool tool(config_path, output_path); 

  for (int ic = 3; ic < argc-1; ic += 2)
  {
    std::string pose_path = argv[ic];
    printf("Processing pose file '%s'\n", pose_path.c_str());
    tool.load_from_odom_file(pose_path);

    std::string fldr_path = argv[ic+1];
    printf("Processing data folder '%s'\n",fldr_path.c_str());
    tool.load_from_pcd_folder(fldr_path);
    printf("done'\n");
  }

  tool.process_queue();

  return 0;
}

#endif
