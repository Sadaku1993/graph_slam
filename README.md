# Graph SLAM

# How to Use

## Download Bag Data
[This bagdata](https://drive.google.com/open?id=1VEy_iJZKEGcNDDKsK-YrqgKxwy6qsric) is recorded at Meiji University (Autonomous Mobile Systems Lab).

## Data Save

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


## Remove Obstacle (Using Clustering)

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
