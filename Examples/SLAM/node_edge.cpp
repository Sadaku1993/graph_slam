#include <ros/ros.h>
#include <ros/package.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/PoseArray.h>
#include <Eigen/Core>
#include <Eigen/LU>

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

        ros::Publisher pub_odom;
        ros::Publisher pub_gicp;

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

    pub_odom = nh.advertise<geometry_msgs::PoseArray>("odometry", 1);
    pub_gicp = nh.advertise<geometry_msgs::PoseArray>("gicp", 1);
    
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

    geometry_msgs::PoseArray odom_array;
    geometry_msgs::PoseArray gicp_array;

    std::ofstream ofs(tf_name, std::ios::trunc);
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
    
    Eigen::Vector3d vector = Eigen::Vector3d::Zero(3);
    Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();

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

        // Affine
        Eigen::Affine3d source_affine;
        Eigen::Affine3d target_affine;
        tf::transformTFToEigen(source_transform, source_affine);
        tf::transformTFToEigen(target_transform, target_affine);

        // Translation
        Eigen::Vector3d source_translation = source_affine.translation();
        Eigen::Vector3d target_translation = target_affine.translation();
        
        // rotation
        Eigen::Matrix3d source_rotation = source_affine.rotation();
        Eigen::Matrix3d target_rotation = target_affine.rotation();

        // relative
        Eigen::Affine3d relative = source_affine.inverse() * target_affine;
        Eigen::Vector3d relative_translation = target_translation - source_translation;
        Eigen::Matrix3d relative_rotation = source_rotation.inverse() * target_rotation;

        // transform pointcloud
        typename pcl::PointCloud<T_p>::Ptr trans_cloud(new pcl::PointCloud<T_p>);
        pcl_ros::transformPointCloud(*source_cloud, *trans_cloud, relative.matrix());

        // Gicp
        Eigen::Matrix4d gicp_matrix;
        Gicp.gicp(source_cloud, trans_cloud, gicp_matrix);


        //-----------------------------------------------------------------------------

        /*
        // odometry
        tf::Transform odom_transform = source_transform.inverseTimes(target_transform);
        Eigen::Matrix4f odom_matrix = Util.tf2eigen(odom_transform);

        // transform PointCloud
        typename pcl::PointCloud<T_p>::Ptr trans_cloud(new pcl::PointCloud<T_p>);
        PCL.TransformPointCloud(source_cloud, trans_cloud, odom_transform);

        // Gicp
        Eigen::Matrix4f gicp_matrix;
        tf::Transform gicp_transform;
        Gicp.gicp(trans_cloud, target_cloud, gicp_matrix);
        */

        // ------------Check-----------
        Eigen::Matrix4f final_matrix;
        
        Eigen::Matrix4f check_matrix;
        check_matrix = gicp_matrix - Eigen::Matrix4f::Identity();
        if(check_matrix.norm() < 0.5){
            final_matrix = gicp_matrix*odom_matrix;
        }
        else{
            final_matrix = odom_matrix;
            std::cout<<"Matching Miss"<<std::endl;
        }

        tf::Transform final_transform = Util.eigen2tf(final_matrix);

        // ------------Integration-------------
        // Translation
        Eigen::Translation<float, 3> translation;
        translation = Eigen::Translation<float, 3>(final_matrix(0, 3), final_matrix(1, 3), final_matrix(2, 3));
        // Rotation
        Eigen::Matrix3f rotation;
        rotation << final_matrix(0, 0), final_matrix(0, 1), final_matrix(0, 2),
                    final_matrix(1, 0), final_matrix(1, 1), final_matrix(1, 2),
                    final_matrix(2, 0), final_matrix(2, 1), final_matrix(2, 2);
        Eigen::Quaternionf quaternion(rotation);
        // Affine
        Eigen::Affine3f affine;
        affine = translation * quaternion;
        // Integration
        integration_matrix = affine * integration_matrix;
        tf::Transform integration_transform = Util.eigen2tf(integration_matrix);

        geometry_msgs::Pose odom_pose;
        geometry_msgs::Pose gicp_pose;

        odom_pose.position.x    = static_cast<float>(target_transform.getOrigin().x());
        odom_pose.position.y    = static_cast<float>(target_transform.getOrigin().y());
        odom_pose.position.z    = static_cast<float>(target_transform.getOrigin().z());
        odom_pose.orientation.x = static_cast<float>(target_transform.getRotation().x());
        odom_pose.orientation.y = static_cast<float>(target_transform.getRotation().y());
        odom_pose.orientation.z = static_cast<float>(target_transform.getRotation().z());
        odom_pose.orientation.w = static_cast<float>(target_transform.getRotation().w());
        odom_array.poses.push_back(odom_pose);
        odom_array.header.frame_id = "map";
        odom_array.header.stamp = ros::Time::now();
        pub_odom.publish(odom_array);

        gicp_pose.position.x    = static_cast<float>(integration_transform.getOrigin().x());
        gicp_pose.position.y    = static_cast<float>(integration_transform.getOrigin().y());
        gicp_pose.position.z    = static_cast<float>(integration_transform.getOrigin().z());
        gicp_pose.orientation.x = static_cast<float>(integration_transform.getRotation().x());
        gicp_pose.orientation.y = static_cast<float>(integration_transform.getRotation().y());
        gicp_pose.orientation.z = static_cast<float>(integration_transform.getRotation().z());
        gicp_pose.orientation.w = static_cast<float>(integration_transform.getRotation().w());
        gicp_array.poses.push_back(gicp_pose);
        gicp_array.header.frame_id = "map";
        gicp_array.header.stamp = ros::Time::now();
        pub_gicp.publish(gicp_array);

        // absulute
        ofs << "VERTEX_SE3:QUAT" <<" "<< (itr+1)->id << " "
            << integration_matrix(0, 3) <<" "
            << integration_matrix(1, 3) <<" "
            << integration_matrix(2, 3) <<" "
            << quaternion.x() <<" "
            << quaternion.y() <<" "
            << quaternion.z() <<" "
            << quaternion.w()
            << std::endl;

        // relative
        ofs << "EDGE_SE3:QUAT" <<" "<< itr->id <<" "<< (itr+1)->id <<" "
            << final_transform.getOrigin().x() <<" "
            << final_transform.getOrigin().y() <<" " 
            << final_transform.getOrigin().z() <<" "
            << final_transform.getRotation().x() <<" "
            << final_transform.getRotation().y() <<" "
            << final_transform.getRotation().z() <<" "
            << final_transform.getRotation().w() <<" "
            << 1.0 <<" "<< 0.0 <<" "<< 0.0 <<" "<< 0.0 <<" "<< 0.0 <<" "<< 0.0 <<" "
            << 1.0 <<" "<< 0.0 <<" "<< 0.0 <<" "<< 0.0 <<" "<< 0.0 <<" "
            << 1.0 <<" "<< 0.0 <<" "<< 0.0 <<" "<< 0.0 <<" "
            << 1.0 <<" "<< 0.0 <<" "<< 0.0 <<" "
            << 1.0 <<" "<< 0.0 <<" "
            << 1.0 << std::endl;
        
        ofs.close();

        // Show
        std::cout<<"Final"<<std::endl;
        Util.printTF(final_matrix);

        std::cout<<"Odometry"<<std::endl;
        Util.printTF(odom_matrix);

        std::cout<<"Integration"<<std::endl;
        Util.printTF(integration_matrix);


        printf("\n");
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
