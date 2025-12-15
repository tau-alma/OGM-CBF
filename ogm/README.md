# Tristan-OGM
2D Occupancy Grid Mapping from 3D Lidar Data. It has cell resolution of 0.5 meter and currently it is using full range point cloud data.

## Running the code
clone the repo ```git clone https://github.com/tau-alma/tristan-ogm.git```


```cd tristan-ogm```

* Pass the lidar csv file path and ground truth csv file path with the script to run, other arguments are optional as they are already been defined in code as default values

```python3 pcd_to_ogm.py path_to_lidar_data.csv path_to_ground_truth.csv```

### Dependency 
* Python3 (3.8.*)
* numpy (1.23.1)
* matplotlib (3.1.2)
* csv (1.0)
* argparse (1.1)

## Algorithm
Occupancy grid maps are used to represent the environments in form of occupied and un-occupied grid cells (Hans Moravec, A.E. Elfes: High resolution maps from wide angle sonar, Proc. IEEE Int. Conf. Robotics Autom. (1985)).A basic 2D grid map has been obtained here from the 3D lidar point cloud. We have extracted the (x,y) values from the point cloud data (x, y, z, I) by rejecting the ground plain and choosing only the points which belongs to the objects and obstacles higher than certain Z dimension value. The output map is represented as a numpy array, and numbers greater than 0 means the cell is occupied (marked with red on the next image), numbers less than 0 shows free cells. The grid has the ability to represent unknown (unobserved) areas, which are equal to 0. This is a basic implementation which provides occupancy for each point cloud scan in sensor frame which can be easily transformed in ego frame by using relative pose of lidar with respect to vehicle. It can be further enhanced as 3D cost maps or more robust incremental occupancy maps, we will be opening the issues in this repo as feature enhancement.

## Dataset
Here is [Link to the dataset](https://tuni-my.sharepoint.com/:f:/g/personal/golnaz_raja_tuni_fi/EuAtyotJjzZJlbt7BZnjed8BP46INH_Gzma1Y_GAILKq-A?e=8MohxM) <br>
Password: tristan
