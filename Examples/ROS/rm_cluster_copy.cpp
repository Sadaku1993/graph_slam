
/*

remove clustered pointcloud

author:
    Yudai Sadakuni

*/

#include <iostream>

#include <ros/ros.h>
#include <ros/package.h> 

#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/io/pcd_io.h>

#include <sys/stat.h>
#include <sys/types.h>

#include <graph_slam/function.h>
#include <graph_slam/normal_estimation.h>
#include <graph_slam/cluster.h>

#include <Util.h>

#define MIN(x, y) ((x) < (y) ? (x) : (y))
#define MAX(x, y) ((x) > (y) ? (x) : (y))


class GRAPH_SLAM::Util; 

template<typename T_p>
class RmCluster{
    private:
        ros::NodeHandle nh;
        std::string package_path;
        std::string cloud_path;
        std::string save_path;
        // min max algorithm parameter
        double cell_size;
        int grid_dimentions;
        double threshold;
        // clustering parameter
        double leaf_size;
        double tolerance;
        int min_size, max_size;
        // normal estimation parameter
        double search_radius;

        GRAPH_SLAM::Util* mpUtil;

    public:
        RmCluster();

        Function<T_p> Fc;
        NormalEstimation<T_p> Ne;
        Clustering<T_p> Ci;

        void min_max(typename pcl::PointCloud<T_p>::Ptr& cloud, 
                     typename pcl::PointCloud<T_p>::Ptr& ground_cloud, 
                     typename pcl::PointCloud<T_p>::Ptr& obstacle_cloud);
        void remove_cluster(typename pcl::PointCloud<T_p>::Ptr& input_cloud, 
                            typename pcl::PointCloud<T_p>::Ptr& remove_cloud);           
        void main();
};

template<typename T_p>
RmCluster<T_p>::RmCluster()
    : nh("~")
{
    package_path = ros::package::getPath("mapping");
    nh.param<std::string>("cloud_path", cloud_path, "/data/cloud");
    nh.param<std::string>("save_path", save_path, "/data/remove");
   
    // min max algorithm parameter
    nh.param<double>("cell_size", cell_size, 0.25);
    nh.param<int>("grid_dimentions", grid_dimentions, 500);
    nh.param<double>("threshold", threshold, 0.1);
    // clustering parameter
    nh.param<double>("leaf_size", leaf_size, 0.07);
    nh.param<double>("torelance", tolerance, 0.15);
    nh.param<int>("min_size", min_size, 100);
    nh.param<int>("max_size", max_size, 1500);
    // normal estimation parameter
    nh.param<double>("search_radius", search_radius, 0.80);

    cloud_path.insert(0, package_path);
    save_path.insert(0, package_path);
}

// min max algorithm
template<typename T_p>
void RmCluster<T_p>::min_max(typename pcl::PointCloud<T_p>::Ptr& cloud, 
                             typename pcl::PointCloud<T_p>::Ptr& ground_cloud, 
                             typename pcl::PointCloud<T_p>::Ptr& obstacle_cloud)
{
    float min[grid_dimentions][grid_dimentions];
    float max[grid_dimentions][grid_dimentions];
    bool init[grid_dimentions][grid_dimentions];

    memset(&min,  0, grid_dimentions*grid_dimentions);
    memset(&max,  0, grid_dimentions*grid_dimentions); 
    memset(&init, 0, grid_dimentions*grid_dimentions);

// #pragma omp parallel for
    for(size_t i=0; i<cloud->points.size(); i++)
    {
        int x = (grid_dimentions/2) + cloud->points[i].x/cell_size;
        int y = (grid_dimentions/2) + cloud->points[i].y/cell_size;

        if(0<=x && x<grid_dimentions && 0<=y && y<grid_dimentions)
        {
            if(!init[x][y]){
                min[x][y] = cloud->points[i].z;
                max[x][y] = cloud->points[i].z;
                init[x][y] = true;
            }
            else{
                min[x][y] = MIN(min[x][y], cloud->points[i].z);
                max[x][y] = MAX(max[x][y], cloud->points[i].z);
            }
        }
    }

    for(size_t i=0;i<cloud->points.size();i++)
    {
        int x = (grid_dimentions/2) + cloud->points[i].x/cell_size;
        int y = (grid_dimentions/2) + cloud->points[i].y/cell_size;

        if(0<=x && x<grid_dimentions && 0<=y && y<grid_dimentions){
            if(init[x][y] && max[x][y]-min[x][y]<threshold)
                ground_cloud->points.push_back(cloud->points[i]);
            else
                obstacle_cloud->points.push_back(cloud->points[i]);
        }else
            obstacle_cloud->points.push_back(cloud->points[i]);
    }
}

// Remove Cluster
template<typename T_p>
void RmCluster<T_p>::remove_cluster(typename pcl::PointCloud<T_p>::Ptr& cloud, 
                                    typename pcl::PointCloud<T_p>::Ptr& cloud_removed)
{
    //Downsample//
    pcl::VoxelGrid<T_p> vg;
    typename pcl::PointCloud<T_p>::Ptr ds_cloud(new pcl::PointCloud<T_p>);
    vg.setInputCloud (cloud);  
    vg.setLeafSize (leaf_size, leaf_size, leaf_size);
    vg.filter (*ds_cloud);

    //downsampled point's z =>0
    std::vector<float> tmp_z;
    tmp_z.resize(ds_cloud->points.size());
	for(int i=0;i<(int)ds_cloud->points.size();i++){
        tmp_z[i]=ds_cloud->points[i].z;
		ds_cloud->points[i].z  = 0.0;
    }

    //Clustering
    typename pcl::search::KdTree<T_p>::Ptr tree (new pcl::search::KdTree<T_p>);
    tree->setInputCloud (ds_cloud);
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<T_p> ec;
    ec.setClusterTolerance (tolerance);
    ec.setMinClusterSize (min_size);
    ec.setMaxClusterSize (max_size);
    ec.setSearchMethod (tree);
    ec.setInputCloud(ds_cloud);
    ec.extract (cluster_indices);
    
    //reset z value
	for(int i=0;i<(int)ds_cloud->points.size();i++)
        ds_cloud->points[i].z=tmp_z[i];

    // remove cluster
    std::vector<int> id_rm;
    std::vector<int> id_cluster;

    // check cluster size
    for(const auto& it : cluster_indices){
        typename pcl::PointCloud<T_p>::Ptr cluster_cloud(new pcl::PointCloud<T_p>);
        for(const auto& pit : it.indices){
            cluster_cloud->points.push_back(ds_cloud->points[pit]);
        }
        Cluster cluster;
        Ci.getClusterInfo(cluster_cloud, cluster);
        
        // pick up human size cluster
        if(0.3<cluster.width && cluster.width<1.0 && 
           0.3<cluster.depth && cluster.depth<1.0 && 
           1.0<cluster.height && cluster.height<2.0)
        {
            for(const auto& pit : it.indices){
                id_cluster.push_back(pit);
            }
        }
    }

    // for(const auto& it : cluster_indices){
    //     for(const auto& pit : it.indices){
    //         id_cluster.push_back(pit);
    //     }
    // }

    std::sort(id_cluster.begin(), id_cluster.end());

    int npoints = ds_cloud->points.size();
    
    std::vector<int>::iterator it = id_cluster.begin();

    for(int i=0;i<npoints; ++i){
        if(it == id_cluster.end() || i != *it){
            cloud_removed->points.push_back(ds_cloud->points[i]);
        }
        else{
            ++it;
        }
    }
}

// main Function
template<typename T_p>
void RmCluster<T_p>::main()
{
    int file_size = Fc.file_count_boost(cloud_path.c_str());
    std::cout<<"file size:"<<file_size<<std::endl;

    for(int i=0;i<file_size;i++){
        typename pcl::PointCloud<T_p>::Ptr cloud(new pcl::PointCloud<T_p>);
        std::string cloud_name = cloud_path+"/"+std::to_string(i)+".pcd";
        std::cout<<"step : "<<cloud_name<<std::endl;
        
        // loadPointCloud
        bool cloud_flag = Fc.loadCloud(cloud, cloud_name);
        if(!cloud_flag) break;

        // Min-Max Algorithm
        typename pcl::PointCloud<T_p>::Ptr ground_cloud(new pcl::PointCloud<T_p>);
        typename pcl::PointCloud<T_p>::Ptr obstacle_cloud(new pcl::PointCloud<T_p>);
        min_max(cloud, ground_cloud, obstacle_cloud);

        // Remove Cluster
        typename pcl::PointCloud<T_p>::Ptr remove_cloud(new pcl::PointCloud<T_p>);
        remove_cluster(obstacle_cloud, remove_cloud);

        // mergePointCloud
        typename pcl::PointCloud<T_p>::Ptr merge_cloud(new pcl::PointCloud<T_p>);
        *merge_cloud += *ground_cloud;
        *merge_cloud += *remove_cloud;

        // Normal Estimation
        typename pcl::PointCloud<T_p>::Ptr normal_cloud(new pcl::PointCloud<T_p>);
        Ne.normal_estimation(merge_cloud, normal_cloud);

        // savePointCloud
        std::string save_name = save_path+"/"+std::to_string(i)+".pcd";
        Fc.saveCloud(merge_cloud, save_name);
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "remove_cluster");

    RmCluster <pcl::PointXYZINormal> rc;

    rc.main();

    return 0;
}
