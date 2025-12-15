"""Running pcd data to 2D occupancy map"""
import os
import argparse
import numpy as np
import csv
import open3d as o3d
import matplotlib.pyplot as plt
from gridmap import GridMap

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Generate 2D occupancy grid from LiDAR and ground truth data")
    parser.add_argument('lidar_data', type=str, help='Path to the lidar data csv file')
    parser.add_argument('ground_truth', type=str, help='Path to the ground truth csv file')
    parser.add_argument('--xy_res', type=float, default=0.5, help='XY resolution of the grid map (default: 0.5)')
    parser.add_argument('--grid_size', type=int, default=50, help='Size of the grid map (default: 50)')
    parser.add_argument('--z_limit', type=float, default=-2.8, help='Z limit for point cloud filtering (default: -2.8)')
    
    args = parser.parse_args()


    with open(args.lidar_data, 'r') as pcdfile:
      x = csv.reader(pcdfile)
      with open(args.ground_truth, 'r') as gtfile:
          gt = csv.reader(gtfile)

        #loading point cloud (pcd) and gt from the csv files  
          for cloud, gt in zip(x, gt):
            if x.line_num == 1:
              continue
            pcd = np.reshape(cloud[1:], (100000,4)).astype(np.float64) 
            pose = gt[1:7]
          
            xyz = pcd[:, :3]
            pcd_c = o3d.geometry.PointCloud()
            pcd_c.points = o3d.utility.Vector3dVector(xyz)
            o3d.visualization.draw_geometries([pcd_c])

            xyz = xyz[xyz[:,2]> args.z_limit]
            xy_resolution = args.xy_res
            ox = xyz[:,0]
            oy = xyz[:,1]

            map = GridMap(grid_size=args.grid_size)
            occupancy_map, min_x, max_x, min_y, max_y, xy_resolution = map.generate_ray_casting_grid_map(xyz, True)
            xy_res = np.array(occupancy_map).shape

          #Plotting the map [0 represents unknown; occupied > 0, free < 0] (Using matplotlib for now, It can be replaced with better option as it's slower in speed)
            plt.figure(1, figsize=(10, 7))
            plt.subplot(122)
            plt.imshow(occupancy_map, cmap="PiYG_r")
          # cmap = "binary" "PiYG_r" "PiYG_r" "bone" "bone_r" "RdYlGn_r"
            plt.clim(-0.4, 1.4)
            plt.gca().set_xticks(np.arange(-.5, xy_res[1], 1), minor=True)
            plt.gca().set_yticks(np.arange(-.5, xy_res[0], 1), minor=True)
            plt.grid(True, which="minor", color="w", linewidth=0.6, alpha=0.5)
            plt.colorbar()
            plt.subplot(121)
            plt.plot([oy, np.zeros(np.size(oy))], [ox, np.zeros(np.size(oy))], "ro-")
            plt.axis("equal")
            plt.plot(0.0, 0.0, "ob")
            plt.gca().set_aspect("equal", "box")
            bottom, top = plt.ylim()  # return the current y-lim
            plt.ylim((top, bottom))  # rescale y axis, to match the grid orientation
            plt.grid(True)
            plt.show()  

