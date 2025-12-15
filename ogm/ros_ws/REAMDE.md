# Tristan OGM C library ROS2 bridge

This ROS2 workspace contains the needed ROS2 RCLPY API for laserscan and pointcloud2 topics to be able to run
the ogm grid algorithm on the ROS2 ecosystem.

# Build

## library for ogm mapping

You need to install the gridmap algorihm package in the root of this git repo.

```
cd gridmap
```

and 

```
mkdir build && cd build
```

and to build the project

```
cmake .. -DLARGE_DATASET=off && make -j8
```

finally install

```
sudo make install
```

## ROS2

Install and prepare [rosdep](https://wiki.ros.org/rosdep):

and install dependecies:

```
rosdep install --from-paths src --ignore-src -r -y
```

and build:

```
colcon build
```

