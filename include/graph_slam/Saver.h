#include <ros/ros.h>
#include <ros/package.h> 
#include <std_msgs/Bool.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/io/pcd_io.h>
#include <fstream>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

template<typename T_p>
class Saver{
    private:
        ros::NodeHandle nh;

        typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, nav_msgs::Odometry> sync_subs;
        message_filters::Subscriber<nav_msgs::Odometry> odom_sub;
        message_filters::Subscriber<sensor_msgs::PointCloud2> lidar_sub;
        message_filters::Synchronizer<sync_subs> sync;

        tf::TransformListener listener;
        tf::StampedTransform transform;
        
        nav_msgs::Odometry old_odom;

        double distance;
        double threshold;

        std::string package_path;
        std::string tf_path;
        
        std::string target_frame;
        std::string source_frame;

        int count;

        bool first_flag;

    public:
        Saver();
        void Callback(const sensor_msgs::PointCloud2::ConstPtr&,
                      const nav_msgs::Odometry::ConstPtr&);
        bool tf_listener();
        bool save_process(const sensor_msgs::PointCloud2::ConstPtr&);
};

template<typename T_p>
Saver<T_p>::Saver()
    : nh("~"),
      odom_sub(nh, "/odom", 10), lidar_sub(nh, "/cloud", 10),
      sync(sync_subs(10), lidar_sub, odom_sub)
{
    sync.registerCallback(boost::bind(&Saver::Callback, this, _1, _2));

    package_path = ros::package::getPath("graph_slam");

    nh.getParam("target_frame", target_frame);
    nh.getParam("source_frame", source_frame);
    
    nh.param<double>("threshold", threshold, 2.0);

    distance = 0.0;
    count = 0;
    first_flag = true;

    tf_path = package_path + "/data/csv/odometry.csv";
}

template<typename T_p>
void Saver<T_p>::Callback(const sensor_msgs::PointCloud2::ConstPtr& cloud,
                          const nav_msgs::Odometry::ConstPtr& odom)
{
    // std::cout<<"Callback"<<std::endl;

    if(first_flag){
        if(save_process(cloud)) first_flag = false;
        else first_flag = true;
    }
    else{
        double dt = sqrt( pow((odom->pose.pose.position.x - old_odom.pose.pose.position.x), 2) + 
                pow((odom->pose.pose.position.y - old_odom.pose.pose.position.y), 2) );
        distance += dt; 
        old_odom = *odom;
    }

    if(threshold<distance){
        save_process(cloud);
        distance = 0.0;
    }
}


template<typename T_p>
bool Saver<T_p>::tf_listener()
{
    try{
        ros::Time now = ros::Time::now();
        listener.waitForTransform(target_frame, source_frame, now, ros::Duration(1.0));
        listener.lookupTransform(target_frame, source_frame,  now, transform);
        return true;
    }
    catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
        return false;
    }
}

template<typename T_p>
bool Saver<T_p>::save_process(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    std::string file_name = std::to_string(count);
    
    // tflistener
    if(!tf_listener()) return false;
    double x = transform.getOrigin().x();
    double y = transform.getOrigin().y();
    double z = transform.getOrigin().z();
    double q_x = transform.getRotation().x();
    double q_y = transform.getRotation().y();
    double q_z = transform.getRotation().z();
    double q_w = transform.getRotation().w();
    double roll, pitch, yaw;
    tf::Matrix3x3(tf::Quaternion(q_x, q_y, q_z, q_w)).getRPY(roll ,pitch, yaw);
    printf("Node:%d x:%.2f y:%.2f z:%.2f roll:%.2f pitch:%.2f yaw:%.2f\n", count, x, y, z, roll, pitch, yaw);

    // save PointCloud
    pcl::PointCloud<T_p> cloud;
    pcl::fromROSMsg(*msg, cloud);
    cloud.width = 1;
    cloud.height = cloud.points.size();
    pcl::io::savePCDFile(package_path+"/data/pcd/"+file_name+".pcd", cloud);

    std::ofstream ofs(tf_path, std::ios::app);
    ofs << "VERTEX_SE3:QUAT" <<" "<< count << " "
        << transform.getOrigin().x()   <<" "
        << transform.getOrigin().y()   <<" "
        << transform.getOrigin().z()   <<" "
        << transform.getRotation().x() <<" "
        << transform.getRotation().y() <<" "
        << transform.getRotation().z() <<" "
        << transform.getRotation().w() 
        << std::endl;
    ofs.close();
    
    count+=1;

    return true;
}
