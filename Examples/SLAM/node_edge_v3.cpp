#include <ros/ros.h>
#include <ros/package.h>

#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseArray.h>

#include <tf_conversions/tf_eigen.h>
#include <tf2_ros/static_transform_broadcaster.h>

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

        int count;

    public:
        NodeEdge();
        
        GRAPH_SLAM::Gicp<T_p> Gicp;
        GRAPH_SLAM::File File;

        void main();
        void pub_pose(tf::transform, 
                      geometry_msgs::PoseArray&,
                      ros::Publisher);
        void tf_broadcast(tf::transform,
                          std::string
                          std::string);
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
void NodeEdge::tf_broadcast(tf::Transform tf,
                            std::string header_frame
                            std::string child_frame)
{
  static tf2_ros::TransformBroadcaster br;
  geometry_msgs::TransformStamped transformStamped;

  transformStamped.header.stamp = ros::Time::now();
  transformStamped.header.frame_id = header_frame;
  transformStamped.child_frame_id = child_frame;

  transformStamped.transform.translation.x = tf.getOrigin().x();
  transformStamped.transform.translation.y = tf.getOrigin().y();
  transformStamped.transform.translation.z = tf.getOrigin().z();
  transformStamped.transform.rotation.x = tf.getRotation().x();
  transformStamped.transform.rotation.y = tf.getRotation().y();
  transformStamped.transform.rotation.z = tf.getRotation().z();
  transformStamped.transform.rotation.w = tf.getRotation().w();

  br.sendTransform(transformStamped);
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

    tf::Transform edge_transform = source_transform.inverseTimes(target_transform);

    tf_broadcast(source_transform, "map", "source_frame");
    tf_broadcast(target_transform, "map", "target_frame");
    tf_broadcast(edge_transform, "source_frame", "edge");
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "NodeEdge2");

  NodeEdge<pcl::PointXYZINormal> ne;

  ne.main();

  return 0;
}
