#include <ros/ros.h>
#include <ros/package.h>

#include <Gicp.h>
#include <Util.h>
#include <File.h>
#include <PCL.h>

template<typename T_p>
class NodeEdge{
    private:
        ros::NodeHandle nh;
        std::string package_path;
        std::string cloud_path;
        std::string tf_path;

        int count;

    public:
        NodeEdge();
        
        GRAPH_SLAM::Gicp<T_p> Gicp;
        GRAPH_SLAM::Util Util;
        GRAPH_SLAM::File File;
        GRAPH_SLAM::PCL<T_p> PCL;

        void first();
        void main();
};

template<typename T_p>
NodeEdge<T_p>::NodeEdge()
    : nh("~")
{
    package_path = ros::package::getPath("graph_slam");
    nh.param<std::string>("cloud_path", cloud_path, "/data/remove/");
    nh.param<std::string>("tf_path", tf_path, "/data/csv/");
    cloud_path.insert(0, package_path);
    tf_path.insert(0, package_path);
    
    count = 0;
}


template<typename T_p>
void NodeEdge<T_p>::first()
{
    std::cout<<"First"<<std::endl;

    // saveTF
    std::string tf_name = tf_path + "gicp.csv";

    // loadTF
    std::string odometry_name = tf_path + "odometry.csv";
    std::vector< ID > transforms;
    File.loadTF(transforms, odometry_name);

    std::ofstream ofs(tf_name, std::ios::app);
    ofs << "VERTEX_SE3:QUAT" <<" "<< 0 <<" "
        << 0.0 <<" "<< 0.0 <<" "<< 0.0 <<" "<< 0.0 <<" "<< 0.0 <<" "<< 0.0 <<" "<< 1.0 << std::endl;
    ofs << "EDGE_SE3:QUAT" <<" "<< 0   <<" "<< 0   <<" "
        << 0.0 <<" "<< 0.0 <<" "<< 0.0 <<" "<< 0.0 <<" "<< 0.0 <<" "<< 0.0 <<" "<< 1.0 << " "
        << 1.0 <<" "<< 0.0 <<" "<< 0.0 <<" "<< 0.0 <<" "<< 0.0 <<" "<< 0.0 <<" "
        << 1.0 <<" "<< 0.0 <<" "<< 0.0 <<" "<< 0.0 <<" "<< 0.0 <<" "
        << 1.0 <<" "<< 0.0 <<" "<< 0.0 <<" "<< 0.0 <<" "
        << 1.0 <<" "<< 0.0 <<" "<< 0.0 <<" "
        << 1.0 <<" "<< 0.0 <<" "
        << 1.0 << std::endl;
    ofs.close();

    // 積算
    Eigen::Matrix4f integration_matrix = Eigen::Matrix4f::Identity();

    for(auto itr=transforms.begin(); itr!=transforms.end()-1; itr++)
    {
        std::cout<< itr->id << " " << (itr+1)->id << std::endl;
        
        std::ofstream ofs(tf_name, std::ios::app);

        // load PointCloud
        typename pcl::PointCloud<T_p>::Ptr source_cloud(new pcl::PointCloud<T_p>);
        typename pcl::PointCloud<T_p>::Ptr target_cloud(new pcl::PointCloud<T_p>);
        File.loadCloud<T_p>(source_cloud, cloud_path+std::to_string(itr->id)+".pcd");
        File.loadCloud<T_p>(target_cloud, cloud_path+std::to_string((itr+1)->id)+".pcd");
        
        // load TF
        tf::Transform source_transform = itr->transform;
        tf::Transform target_transform = (itr+1)->transform;

        // transform PointCloud
        typename pcl::PointCloud<T_p>::Ptr transform_source_cloud(new pcl::PointCloud<T_p>);
        typename pcl::PointCloud<T_p>::Ptr transform_target_cloud(new pcl::PointCloud<T_p>);
        PCL.TransformPointCloud(source_cloud, transform_source_cloud, source_transform);
        PCL.TransformPointCloud(target_cloud, transform_target_cloud, target_transform);

        // Gicp
        Eigen::Matrix4f gicp_matrix;
        tf::Transform gicp_transform;
        Gicp.gicp(transform_target_cloud, transform_source_cloud, gicp_matrix);
        gicp_transform = Util.eigen2tf(gicp_matrix);
        std::cout<<"GICP"<<std::endl;
        Util.printTF(gicp_matrix);

        // Odometry
        tf::Transform odom_transform;
        Util.relative(source_transform, target_transform, odom_transform);
        std::cout<<"Odometry"<<std::endl;
        Util.printTF(odom_transform);

        ofs.close();
    }
}

template<typename T_p>
void NodeEdge<T_p>::main()
{
    std::cout<<"main"<<std::endl;

    first();
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "NodeEdge");

    NodeEdge<pcl::PointXYZINormal> ne;

    ne.main();

    return 0;
}
