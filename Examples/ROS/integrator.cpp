#include <ros/ros.h>
#include <ros/package.h>

#include <Util.h>
#include <File.h>
#include <PCL.h>

template<typename T_p>
class Integrator{
    private:
        ros::NodeHandle nh;
        
        std::string package_path;
        std::string cloud_path;
        std::string tf_name;
        std::string map_name;

    public:
        Integrator();
        GRAPH_SLAM::File File;
        GRAPH_SLAM::Util Util;
        GRAPH_SLAM::PCL<T_p> PCL;

        void main();
};

template<typename T_p>
Integrator<T_p>::Integrator()
    : nh("~")
{
    // path
    package_path = ros::package::getPath("graph_slam");
    nh.param<std::string>("cloud_path", cloud_path, "/data/remove/");
    nh.param<std::string>("tf_name", tf_name, "/data/csv/gicp.csv");
    nh.param<std::string>("map_name", map_name, "/data/map/gicp.pcd");
    cloud_path.insert(0, package_path);
    tf_name.insert(0, package_path);
    map_name.insert(0, package_path);
}

template<typename T_p>
void Integrator<T_p>::main()
{
    std::cout<<"Integrator"<<std::endl;

    typename pcl::PointCloud<T_p>::Ptr merge_cloud(new pcl::PointCloud<T_p>);

    // loadTF
    std::vector< ID > transforms;
    File.loadTF(transforms, tf_name);
    std::cout<<transforms.size()<<std::endl;

    for (auto itr=transforms.begin(); itr!=transforms.end(); itr++)
    {
        std::cout<<"ID:"<<itr->id<<" "<<std::endl;
        Util.printTF(itr->transform);

        std::string cloud_name = cloud_path + std::to_string(itr->id) + ".pcd";
        typename pcl::PointCloud<T_p>::Ptr cloud(new pcl::PointCloud<T_p>);
        File.loadCloud<T_p>(cloud, cloud_name);

        typename pcl::PointCloud<T_p>::Ptr trans_cloud(new pcl::PointCloud<T_p>);
        PCL.TransformPointCloud(cloud, trans_cloud, itr->transform);

        *merge_cloud += *trans_cloud;
    }

    // saveCloud
    File.saveCloud<T_p>(merge_cloud, map_name);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "Integrator");

    Integrator<pcl::PointXYZINormal> Ig;

    Ig.main();

    return 0;
}
