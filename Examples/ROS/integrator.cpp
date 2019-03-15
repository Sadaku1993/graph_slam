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
        std::string tf_path;
        std::string map_path;

        std::string tf_name;
        std::string map_name;

        int skip;

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
    nh.param<std::string>("tf_path", tf_path, "/data/csv/");
    nh.param<std::string>("map_path", map_path, "/data/map/");
    nh.param<int>("skip", skip, 1);

    cloud_path.insert(0, package_path);
    tf_path.insert(0, package_path);
    map_path.insert(0, package_path);
}

template<typename T_p>
void Integrator<T_p>::main()
{
    std::cout<<"Integrator"<<std::endl<<std::endl;

    // loadTF
    std::vector< ID > transforms;
    std::vector<std::string> file_lists;
    File.search_dir(tf_path, file_lists);
    std::cout<<"CSV file list "<<std::endl;
    for(auto itr=file_lists.begin(); itr!=file_lists.end(); itr++){
        std::cout<<"  "<<*itr<<std::endl;
    }
    std::cout<<std::endl;

    std::cout<<"Please serect csv file: ";
    std::string name_csv;
    std::getline(std::cin, name_csv);
    tf_name = tf_path + name_csv;
    
    std::cout<<"Please enter map file: ";
    std::string name_map;
    std::getline(std::cin, name_map);
    map_name = map_path + name_map;

    File.loadTF(transforms, tf_name);
    std::cout<<transforms.size()<<std::endl;

    typename pcl::PointCloud<T_p>::Ptr merge_cloud(new pcl::PointCloud<T_p>);

    for (auto itr=transforms.begin(); itr!=transforms.end(); itr++)
    {
        if(itr->id % skip != 0) continue;

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
    std::cout<<"Map is saved at\n  "<<map_name<<std::endl;
    File.saveCloud<T_p>(merge_cloud, map_name);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "Integrator");

    Integrator<pcl::PointXYZINormal> Ig;

    Ig.main();

    return 0;
}
