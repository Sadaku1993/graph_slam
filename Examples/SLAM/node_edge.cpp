#include <ros/ros.h>
#include <ros/package.h>

#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/TransformStamped.h>

#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
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

        ros::Publisher pub_cloud;

        ros::Publisher pub_cloud_odom;
        ros::Publisher pub_cloud_gicp;

        tf::TransformListener listener;
  
        int count;

    public:
        NodeEdge();
        
        GRAPH_SLAM::Gicp<T_p> Gicp;
        GRAPH_SLAM::File File;

        void main();
        void pub_pose(tf::Transform, 
                      geometry_msgs::PoseArray&,
                      ros::Publisher);
        void pubCloud(typename pcl::PointCloud<T_p>::Ptr,
                       ros::Publisher);
        void tf_broadcast(tf::Transform,
                          std::string,
                          std::string);
        void tf_listener(std::string,
                         std::string,
                         tf::StampedTransform&);
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
    pub_cloud_odom = nh.advertise<sensor_msgs::PointCloud2>("cloud_odom", 1);
    pub_cloud_gicp = nh.advertise<sensor_msgs::PointCloud2>("cloud_gicp", 1);
    count = 0;
}

template<typename T_p>
void NodeEdge<T_p>::pub_pose(tf::Transform transform,
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
void NodeEdge<T_p>::pubCloud(typename pcl::PointCloud<T_p>::Ptr cloud,
                              ros::Publisher pub)
{
    sensor_msgs::PointCloud2 pc;
    pcl::toROSMsg(*cloud, pc);
    pc.header.frame_id = "map";
    pc.header.stamp = ros::Time::now();
    pub.publish(pc);
}

template<typename T_p>
void NodeEdge<T_p>::tf_broadcast(tf::Transform tf,
                                 std::string header_frame,
                                 std::string child_frame)
{
  static tf2_ros::StaticTransformBroadcaster static_broadcaster;
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

  static_broadcaster.sendTransform(transformStamped);
}

template<typename T_p>
void NodeEdge<T_p>::tf_listener(std::string source_frame,
                                std::string target_frame,
                                tf::StampedTransform& transform)
{
    try{
        ros::Time time = ros::Time(0);
        listener.waitForTransform(source_frame, target_frame, time, ros::Duration(1.0));
        listener.lookupTransform(source_frame, target_frame, time, transform);
    }
    catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
    }
}

template<typename T_p>
void NodeEdge<T_p>::main()
{
  std::cout<<"NodeEdge"<<std::endl;

  std::string tf_name = tf_path + "gicp.csv";

  std::string odometry_name = tf_path + "odometry.csv";
  std::vector< ID > transforms;
  File.loadTF(transforms, odometry_name);

  geometry_msgs::PoseArray odom_array;
  geometry_msgs::PoseArray gicp_array;

  Eigen::Affine3d absolute_affine;
  absolute_affine = Eigen::Matrix4d::Identity();

  std::ofstream ofs(tf_name, std::ios::trunc);
  ofs << "VERTEX_SE3:QUAT" <<" "<< 0 <<" "
      << 0.0 <<" "<< 0.0 <<" "<< 0.0 <<" "<< 0.0 <<" "<< 0.0 <<" "<< 0.0 <<" "<< 1.0 << std::endl;


  // ofs << "EDGE_SE3:QUAT" <<" "<< 0 <<" "<< 0 <<" "
  //     <<0.0<<" "<<0.0<<" "<<0.0<<" "<<0.0<<" "<<0.0<<" "<<0.0<<" "<<1.0<<" "
  //     <<1.0<<" "<<0.0<<" "<<0.0<<" "<<0.0<<" "<<0.0<<" "<<0.0<<" "
  //     <<1.0<<" "<<0.0<<" "<<0.0<<" "<<0.0<<" "<<0.0<<" "
  //     <<1.0<<" "<<0.0<<" "<<0.0<<" "<<0.0<<" "
  //     <<1.0<<" "<<0.0<<" "<<0.0<<" "
  //     <<1.0<<" "<<0.0<<" "
  //     <<1.0
  //     <<std::endl;

  ofs.close();

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

    // -------------------GICP---------------------------
    typename pcl::PointCloud<T_p>::Ptr trans_cloud(new pcl::PointCloud<T_p>);
    pcl_ros::transformPointCloud(*target_cloud, *trans_cloud, edge_transform);
    
    Eigen::Matrix4d gicp_matrix;
    Eigen::Affine3d gicp_affine;
    Gicp.gicp(source_cloud, trans_cloud, gicp_matrix, 0.5);
    gicp_affine = gicp_matrix;
    tf::Transform gicp_transform;
    tf::transformEigenToTF(gicp_affine, gicp_transform);

    Eigen::Affine3d edge_affine;
    tf::transformTFToEigen(edge_transform, edge_affine);
    
    Eigen::Affine3d affine;
    tf::Transform transform;

    Eigen::Matrix4d check = gicp_matrix - Eigen::Matrix4d::Identity();
    if(check.norm() < 1.0){
        affine = gicp_affine.inverse() * edge_affine;
    }
    else{
        std::cout<<"Matching Miss"<<std::endl;
        affine = edge_affine;
    }
    tf::transformEigenToTF(affine, transform);
    
    absolute_affine = absolute_affine * affine;
    tf::Transform absolute_transform;
    tf::transformEigenToTF(absolute_affine, absolute_transform);

    pub_pose(target_transform, odom_array, pub_odom);
    pub_pose(absolute_transform, gicp_array, pub_gicp);

    typename pcl::PointCloud<T_p>::Ptr absolute_cloud(new pcl::PointCloud<T_p>);
    pcl_ros::transformPointCloud(*target_cloud, *absolute_cloud, absolute_transform);
    pubCloud(absolute_cloud, pub_cloud_gicp);

    typename pcl::PointCloud<T_p>::Ptr odom_cloud(new pcl::PointCloud<T_p>);
    pcl_ros::transformPointCloud(*target_cloud, *odom_cloud, target_transform);
    pubCloud(odom_cloud, pub_cloud_odom);

    // VERTEX_SE3:QUAT
    std::ofstream ofs(tf_name, std::ios::app);
    ofs << "VERTEX_SE3:QUAT" <<" "<< (itr+1)->id << " "
        << absolute_transform.getOrigin().x()<<" "
        << absolute_transform.getOrigin().y()<<" "
        << absolute_transform.getOrigin().z()<<" "
        << absolute_transform.getRotation().x()<<" "
        << absolute_transform.getRotation().y()<<" "
        << absolute_transform.getRotation().z()<<" "
        << absolute_transform.getRotation().w()
        << std::endl;

    // EDGE_SE3:QUAT
    ofs << "EDGE_SE3:QUAT" <<" "<< (itr+1)->id <<" "<< itr->id <<" "
      <<transform.getOrigin().x()<<" "
      <<transform.getOrigin().y()<<" "
      <<transform.getOrigin().z()<<" "
      <<transform.getRotation().x()<<" "
      <<transform.getRotation().y()<<" "
      <<transform.getRotation().z()<<" "
      <<transform.getRotation().w()
      <<std::endl;

    // ofs << "EDGE_SE3:QUAT" <<" "<< (itr+1)->id <<" "<< itr->id <<" "
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
    ofs.close();

    /*
    std::string source_name = std::to_string(itr->id);
    std::string target_name = std::to_string((itr+1)->id);

    if(itr->id == 0){
        tf_broadcast(transform, "map", target_name);
        pub_pose(source_transform, odom_array, pub_odom);
    }
    else{
        tf_broadcast(transform, source_name, target_name);
        pub_pose(target_transform, odom_array, pub_odom);
    }

    tf::StampedTransform stamped_absolute_transform;
    tf_listener("map", target_name, stamped_absolute_transform);
    tf::Transform absolute_transform;
    absolute_transform.setOrigin(stamped_absolute_transform.getOrigin());
    absolute_transform.setRotation(stamped_absolute_transform.getRotation());
    typename pcl::PointCloud<T_p>::Ptr absolute_cloud(new pcl::PointCloud<T_p>);
    pcl_ros::transformPointCloud(*target_cloud, *absolute_cloud, absolute_transform);
    pubCloud(absolute_cloud, pub_cloud);
    */
    
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "NodeEdge");

  NodeEdge<pcl::PointXYZINormal> ne;

  ne.main();

  return 0;
}
