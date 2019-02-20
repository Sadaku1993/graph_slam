# Graph SLAM

# How to Use


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
$ rosrun graph_slam rm_cluster
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
$rosrun graph_slam slam3d
```
