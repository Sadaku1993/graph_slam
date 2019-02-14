#include <ros/ros.h>
#include <ros/package.h>

#include <File.h>
#include <PCL.h>

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


    public:
        RmCluster();
        GRAPH_SLAM::File File;
        GRAPH_SLAM::PCL<T_p> PCL;

        void main();
};

template<typename T_p>
RmCluster<T_p>::RmCluster()
    : nh("~")
{
    // path
    package_path = ros::package::getPath("graph_slam");
    nh.param<std::string>("cloud_path", cloud_path, "/data/pcd/");
    nh.param<std::string>("save_path", save_path, "/data/remove/");
    cloud_path.insert(0, package_path);
    save_path.insert(0, package_path);
    // min max algorithm parameter
    nh.param<double>("cell_size", cell_size, 0.25);
    nh.param<int>("grid_dimentions", grid_dimentions, 500);
    nh.param<double>("threshold", threshold, 0.1);
    // clustering parameter
    nh.param<double>("leaf_size", leaf_size, 0.07);
    nh.param<double>("tolerance", tolerance, 0.15);
    nh.param<int>("min_size", min_size, 100);
    nh.param<int>("max_size", max_size, 1500);
    // normal estimation parameter
    nh.param<double>("search_radius", search_radius, 0.80);
}

template<typename T_p>
void RmCluster<T_p>::main()
{
    std::cout<<"Remove Cluster"<<std::endl;

    int file_size = File.file_count_boost(cloud_path.c_str());
    std::cout<<"file size : "<<file_size<<std::endl;

    for(int i=0;i<file_size;i++){
        std::cout<<"File:"<<i+1<<"/"<<file_size<<" ";
        // load PointCloud
        typename pcl::PointCloud<T_p>::Ptr cloud(new pcl::PointCloud<T_p>);
        std::string cloud_name = cloud_path + std::to_string(i)+".pcd";
        File.loadCloud<T_p>(cloud, cloud_name);
        std::cout<<cloud_name<<std::endl;

        // Min-Max
        typename pcl::PointCloud<T_p>::Ptr ground_cloud(new pcl::PointCloud<T_p>);
        typename pcl::PointCloud<T_p>::Ptr obstacle_cloud(new pcl::PointCloud<T_p>);
        PCL.min_max(cloud, ground_cloud, obstacle_cloud,
                    grid_dimentions, threshold, cell_size);

        // Remove Cluster
        typename pcl::PointCloud<T_p>::Ptr remove_cloud(new pcl::PointCloud<T_p>);
        PCL.remove_cluster(obstacle_cloud, remove_cloud,
                           leaf_size, min_size, max_size, tolerance);

        // mergePointCloud
        typename pcl::PointCloud<T_p>::Ptr merge_cloud(new pcl::PointCloud<T_p>);
        *merge_cloud += *ground_cloud;
        *merge_cloud += *remove_cloud;

        // normal estimation
        // typename pcl::PointCloud<pcl::PointXYZINormal>::Ptr normal_cloud(new pcl::PointCloud<pcl::PointXYZINormal>);
        // PCL.NormalEstimation(merge_cloud, normal_cloud, search_radius);

        // savePointCloud
        std::string save_name = save_path + std::to_string(i) + ".pcd";
        File.saveCloud<pcl::PointXYZINormal>(merge_cloud, save_name);
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "Rm_Cluster");

    RmCluster<pcl::PointXYZINormal> rc;

    rc.main();

    return 0;
}
