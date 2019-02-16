#include <ros/ros.h>
#include <ros/package.h>

#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseArray.h>

#include <tf_conversions/tf_eigen.h>

#include <Eigen/Core>
#include <Eigen/LU>

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include <Gicp.h>
#include <File.h>

template<typename T_p>
class NodeEdge{
    private:
        ros::NodeHandle nh;
        std::string package_path;
        std::string cloud_path;
        std::string tf_path;

        ros::Publisher pub_odom;
        ros::Publisher pub_gicp;

        ros::Publisher pub_source_cloud;
        ros::Publisher pub_target_cloud;
        ros::Publisher pub_relative_cloud;
        ros::Publisher pub_integrate_cloud;

        int count;

    public:
        NodeEdge();
        
        GRAPH_SLAM::Gicp<T_p> Gicp;
        GRAPH_SLAM::File File;

        void main();
        void pub_pose(tf::transform, 
                      geometry_msgs::PoseArray&,
                      ros::Publisher);
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
    pub_source_cloud = nh.advertise<sensor_msgs::PointCloud2>("source_cloud", 1);
    pub_target_cloud = nh.advertise<sensor_msgs::PointCloud2>("target_cloud", 1);
    pub_relative_cloud = nh.advertise<sensor_msgs::PointCloud2>("relative_cloud", 1);
    pub_integrate_cloud = nh.advertise<sensor_msgs::PointCloud2>("integrate_cloud", 1);
    
    count = 0;
}

template<typename T_p>
void NodeEdge<T_p>::pub_pose(tf::Transform transorm,
                             geometry_msgs::PoseArray& pose_array,
                             ros::Publisher pub)
{
  geometry_msgs::Pose pose;
  pose.position.x = static_cast<float>(transform.getOrigin().x());
  pose.position.y = static_cast<float>(transform.getOrigin().y());
  pose.position.z = static_cast<float>(transform.getOrigin().z());
  pose.orientation.x = static_cast<float>(transform.getRotation().x());
  pose.orientation.y = static_cast<float>(transform.getRotation().y());
  pose.orientation.z = static_cast<float>(transform.getRotation().z());
  pose.orientation.w = static_cast<float>(transform.getRotation().w());

  pose_array.poses.push_back(pose);
  pose_array.header.frame_id = "map";
  pose_array.header.stamp = ros::Time::now();
  pub.publish(pose_array);
}

template<typename T_p>
void NodeEdge<T_p>::main()
{
  std::cout<<"NodeEdge"<<std::endl;

  std::strinng tf_name = tf_path + "gicp.csv";

  std::string odometry_name = tf_path + "odometry.csv";
  std::vector< ID > transforms;
  File.loadTF(transforms, odometry_name);

  geometry_msgs::PoseArray odom_array;
  geoemtry_msgs::PoseArray gicp_array;

  Eigen::Affine3d affine;
  affine = Eigen::Matrix4d::Identity;

  for(auto itr=transforms.begin(); itr!=transforms.end()-1; itr++)
  {
    std::cout<< itr->id << " " << (itr+1)->id <<std::endl;

    // --------------------Load PointCloud and Transform------------------- 
    typename pcl::PointCloud<T_p>::Ptr source_cloud(new pcl::PointCloud<T_p>);
    typename pcl::PointCloud<T_p>::Ptr target_cloud(new pcl::PointCloud<T_p>);
    File.loadCloud<T_p>(source_cloud, cloud_path+std::to_string(itr->id)+".pcd");
    File.loadCloud<T_p>(target_cloud, cloud_path+std::to_string((itr+1)->id)+".pcd");

    tf::Transform source_transform = itr->transform;
    tf::Transform target_transform = (itr+1)->transform;

    Eigen::Affine3d source_affine;
    Eigen::Affine3d target_affine;
    tf::transformTFToEigen(source_transform, source_affine);
    tf::transformTFToEigen(target_transform, target_affine);

    // -------------------Relative-------------------
    Eigen::Affine3d relative_affine = target_affine.inverse() * source_affine;
    tf::Transform relative_transform;
    tf::transformEigenToTF(relative_affine, relative_transform);
    
    // -------------------GICP----------------------
    typename pcl::PointCloud<T_p>::Ptr trans_cloud(new pcl::PointCloud<T_p>);
    pcl_ros::transformPointCloud(*source_cloud, *trans_cloud, relative_transform);
    Eigen::Matrix4d gicp_matrix;
    Eigen::Affine3d gicp_affine;
    Gicp.gicp(trans_cloud, target_cloud, gicp_matrix);
    gicp_affine = gicp_matrix;

    // --------------------Edge---------------------
    Eigen::Affine3d edge_affine;
    Eigen::Vector3d edge_translation = -source_affine.translation() + 
                                       target_translation.translation();
    Eigen::Matrix3d edge_rotation = source_affine.rotation().inverse() * 
                                    target_affine.rotation();

    // -------------------Matching------------------
    Eigen::Matrix4d check = gicp_matrix - Eigen::Matrix4d::Identity();
    if(check.norm() < 1.0){
      edge_translation = gicp_affine.translation() + edge_translation;
      edge_rotation = gicp_affine.rotation() * edge_rotation;
    }

    edge_affine = edge_translation * edge_rotation;
    
    std::cout<<"edge\n"<<edge_affine.matrix()<<std::endl;

    // -------------------Integrate------------------
    affine = edge_affine * affine;
    tf::Transform transform;
    tf::transformEigenToTF(affine, transform);
    
    pub_pose(target_transform, odom_array, pub_odom);
    pub_pose(transform, gicp_array, pub_gicp);

    printf("\n");
  }
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "NodeEdge2");

  NodeEdge<pcl::PointXYZINormal> ne;

  ne.main();

  return 0;
}
