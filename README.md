# Graph SLAM

# Environment
- Ubuntu16.04
- ROS(Kinetic)
- g2o
- PCL(1.8)

# Setup

## Graph SLAM
Download Graph SLAM package

```bash
(catkin workspace)
$ git clone https://github.com/Sadaku1993/graph_slam
```

## g2o
Download g2o package

### Requirements
* cmake
* Eigen3
* suitesparse
* Qt5
* libQGLViewer

On Ubuntu these dependencies are resolved by installing the following packages.

- cmake
- Eigen3
- libsuitesparse-dev
- qtdeclarative5-dev
- qt5-qmake
- libqglviewer-dev

### Install

```bash
( graph_slam package )
$ ./setup.bash
$ cd Thirdparty
$ git clone https://github.com/Sadaku1993/g2o
$ cd g2o
$ mkdir build
$ cmake .. -DBUILD_CSPARSE=ON
$ make
$ sudo make install
```

## Velodyne
Download Velodyne package

```bash
( catkin workspace )
$ git clone https://github.com/ros-drivers/velodyne
```

## Normal Estimation
Download Normal Estimation Package

```bash
( catkin workspace )
$ git clone https://github.com/amslabtech/perfect_velodyne
```

## Download Bag Data
[This bagdata](https://drive.google.com/open?id=1VEy_iJZKEGcNDDKsK-YrqgKxwy6qsric) is recorded at Meiji University (Autonomous Mobile Systems Lab).

Move this bagdata to bagfiles dirs.

```bash
graph_slam/
  data/
    bagfiles/
      Infant-Dkan.bag
```

# How to Use

1. Save Data
2. Remove Obstacle
3. GICP
4. Graph Optimization and Loop Closing

## Save Data
save odometry and pointcloud data.

```bach
$ roslaunch graph_slam data_save.launch
```

odometry is saved at 
```bash
graph_slam
    /data
        /csv/odometry.csv
```

pointcloud is saved at 
```bash
/graph_slam
    /data
        /pcd
            /0.pcd
            /1.pcd
             :
             :
             :
```


## Remove Obstacle (By Clustering)

in the pointcloud data, obstacles (like human and so on) are included.
in order to removed them, use clustering(PCL)

```bash
$ roslaunch graph_slam remove_cluster.launch
```

pointcloud is saved at
```bash
/graph_slam
    /data
        /remove
            /0.pcd
            /1.pcd
             :
             :
             :
```

## Gicp

```bash
$roslaunch graph_slam node_edge.launch
```

node edge data is saved at 

```bash
/graph_slam
  /data
    /csv
      gicp.csv
```

## Loop Closing

```bash
$roslaunch graph_slam slam3d.launch
```

update node edge data is saved at

```bash
/graph_slam
  /data
    /csv
      /0.csv
      /1.csv
       :
       :
       :
```


## View Created 3D Map

prease serect csv file name and detect pcd file name

```bash
$roslaunch graph_slam integrator.launch

  CSV file list
    odometry.csv
    gicp.csv
    0.csv
    1.csv
    :
    :
  
  Please select csv file: 2.csv
  Please enter map file: 2.pcd
    :
    :
```
