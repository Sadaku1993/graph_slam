#include <ros/ros.h>
#include <ros/package.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include <tf_conversions/tf_eigen.h>

#include <Gicp.h>
#include <File.h>
#include <LoopCloser.h>

template<typename T_p>
class Update{
    private:
        ros::NodeHandle nh;

        int THRESHOLD;
        double DISTANCE;
    
        std::string package_path;
        std::string cloud_path;
        std::string tf_path;

    public:
        Update();
        GRAPH_SLAM::LoopCloser lc;
        GRAPH_SLAM::Gicp<T_p> Gicp;
        GRAPH_SLAM::File File;
        void main();
};

template<typename T_p>
Update<T_p>::Update()
    : nh("~")
{
    package_path = ros::package::getPath("graph_slam");
    nh.param<std::string>("cloud_path", cloud_path, "/data/remove/");
    nh.param<std::string>("tf_path", tf_path, "/data/csv/");
    cloud_path.insert(0, package_path);
    tf_path.insert(0, package_path);

    nh.param<int>("THRESHOLD", THRESHOLD, 10);
    nh.param<double>("DISTANCE", DISTANCE, 1.5);
}

template<typename T_p>
void Update<T_p>::main()
{
    std::string csv_file = tf_path + "gicp.csv";

    // Loop Detector
    std::vector<LoopNode> loop_node = lc.main(THRESHOLD, DISTANCE, csv_file);

    for(auto itr=loop_node.begin(); itr!=loop_node.end(); itr++)
    {
        std::cout<<itr->source_id<<" "<<itr->target_id<<std::endl;

        // ------------Load PointCloud and Transform---------
        typename pcl::PointCloud<T_p>::Ptr source_cloud(new pcl::PointCloud<T_p>);
        typename pcl::PointCloud<T_p>::Ptr target_cloud(new pcl::PointCloud<T_p>);
        File.loadCloud<T_p>(source_cloud, cloud_path+std::to_string(itr->source_id)+".pcd");
        File.loadCloud<T_p>(target_cloud, cloud_path+std::to_string(itr->target_id)+".pcd");

        // ----------------Relative transform----------------
        tf::Transform edge_transform = itr->source_transform.inverseTimes(itr->target_transform);

        // -------------------GICP---------------------------
        typename pcl::PointCloud<T_p>::Ptr trans_cloud(new pcl::PointCloud<T_p>);
        pcl_ros::transformPointCloud(*target_cloud, *trans_cloud, edge_transform);

        Eigen::Matrix4d gicp_matrix;
        Eigen::Affine3d gicp_affine;
        Gicp.gicp(source_cloud, trans_cloud, gicp_matrix, 1.0);
        gicp_affine = gicp_matrix;
        tf::Transform gicp_transform;
        tf::transformEigenToTF(gicp_affine, gicp_transform);

        Eigen::Affine3d edge_affine;
        tf::transformTFToEigen(edge_transform, edge_affine);

        Eigen::Affine3d affine;
        tf::Transform transform;

        Eigen::Matrix4d check = gicp_matrix - Eigen::Matrix4d::Identity();

        if(3.0< check.norm()){
            std::cout<<"Matching Miss"<<std::endl;
            continue;
        }

        affine = gicp_affine.inverse() * edge_affine;
        tf::transformEigenToTF(affine, transform);
        std::cout<<"New Gicp\n"<<affine.matrix()<<std::endl;
        
        // ofs << "EDGE_SE3:QUAT" <<" "<< itr->id <<" "<< (itr+1)->id <<" "
        //     <<transform.getOrigin().x()<<" "
        //     <<transform.getOrigin().y()<<" "
        //     <<transform.getOrigin().z()<<" "
        //     <<transform.getRotation().x()<<" "
        //     <<transform.getRotation().y()<<" "
        //     <<transform.getRotation().z()<<" "
        //     <<transform.getRotation().w()<<" "
        //     <<1.0<<" "<<0.0<<" "<<0.0<<" "<<0.0<<" "<<0.0<<" "<<0.0<<" "
        //     <<1.0<<" "<<0.0<<" "<<0.0<<" "<<0.0<<" "<<0.0<<" "
        //     <<1.0<<" "<<0.0<<" "<<0.0<<" "<<0.0<<" "
        //     <<1.0<<" "<<0.0<<" "<<0.0<<" "
        //     <<1.0<<" "<<0.0<<" "
        //     <<1.0
        //     <<std::endl;
        // ofs.close();
    }

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "LoopCloser");

    Update<pcl::PointXYZINormal> up;

    up.main();

    return 0;
}
