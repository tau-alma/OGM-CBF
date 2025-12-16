# Tristan OGM C/CPP

This folder contains the implementation of the tristan occupancy grid implementation

# Building

In this folder:

```
mkdir build && cd build
cmake .. 
make -j8
```
or with the compile flags


```
mkdir build && cd build
cmake -DENABLE_VISUALIZATION=OFF -DLARGE_DATASET=OFF ..
make -j8
```


Compile flags include following:

| FLAG      | Description | DEFAULT |
| ----------- | ----------- | ----|
| ENABLE_VISUALIZATION | Option to enable or disable the SDL2 visualization of the cloud and map | ON|
| LARGE_DATASET  | Option to use small or large dataset for the C implementation| ON|

# Versions

In this version there are following versions:


## Occypancy grid scans

Pure C implementation utilizing dataset as a headers to the project during the compile time, dataset is compiled to the project before running the project according to the compiler flags
* ```./bin/simple_example```
    
C++/C implementation of the local instance of the map with the given lidar dataset in the csv file
* ```./bin/local_map <path/to/the/dataset/>lidardata.csv```

C++/C implementation of the global instance of the map building with the given lidar dataset and motion estimation in the csv file
* ```./bin/global_map <path/to/the/dataset/>lidardata.csv> <path/to/the/dataset/ground_truth.csv>```

## Occypancy Voxel scan 

Pure C implementation utilizing dataset as a headers to the project during the compile time, dataset is compiled to the project before running the project according to the compiler flags
* ```./bin/simple_example_voxel```

C++/C implementation of the local instance of the voxel map with the given lidar dataset in the csv file
* ```./bin/voxel_map <path/to/the/dataset/>lidardata.csv```