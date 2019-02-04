/*
 *
 * PointCloud, tfを保存する
 *
 * 一定の移動の後保存する
 *
 * author:Yudai Sadakuni
 */

#include <ros/ros.h>
#include <ros/package.h> 
#include <sensor_msgs/PointCloud2.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/io/pcd_io.h>
#include <fstream>

class DataSaver{
    private:
        ros::NodeHandle nh;
        ros::Subscriber cloud_sub;
        ros::Subscriber odom_sub;
        std::string target_frame;
        std::string source_frame;
        tf::TransformListener listener;
        tf::StampedTransform  transform;
        sensor_msgs::Image image;
        sensor_msgs::PointCloud2 pc2;

        nav_msgs::Odometry old_odom;
        double distance;
        bool odom_flag;
        double threshold;

        std::string package_path;
        int count;

    public:
        DataSaver();
        void imageCallback(const sensor_msgs::Image::ConstPtr&);
        void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr&);
        void odomCallback(const nav_msgs::Odometry::ConstPtr&);
        bool tf_listener();
        void save_process();
};

DataSaver::DataSaver()
    : nh("~")
{
    cloud_sub = nh.subscribe("/cloud", 10, &DataSaver::cloudCallback, this);
    odom_sub = nh.subscribe("/odom", 1, &DataSaver::odomCallback, this);
    nh.getParam("target_frame", target_frame);
    nh.getParam("source_frame", source_frame);
    nh.param<double>("threshold", threshold, 2.0);
    distance = threshold;
    odom_flag = false;

    package_path = ros::package::getPath("mapping");
    count = 0;
}

void DataSaver::cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    // std::cout<<"cloud callback"<<std::endl;
    pc2 = *msg;
}

void DataSaver::odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    // std::cout<<"odom callback"<<std::endl;
    // 移動量を算出
    if(!odom_flag){
        old_odom = *msg;
        odom_flag = true;
    }
    else{
        double dt = sqrt( pow((msg->pose.pose.position.x - old_odom.pose.pose.position.x), 2) + 
                pow((msg->pose.pose.position.y - old_odom.pose.pose.position.y), 2) );
        distance += dt; 
        old_odom = *msg;
    }

    // しきい値以上移動したらデータを保存
    if(threshold<distance){
        save_process();
        distance = 0;
    }
}

bool DataSaver::tf_listener()
{
    // tflistener
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

void DataSaver::save_process()
{
    std::cout<<"file_path : "<<package_path<<std::endl;
    std::string file_name = std::to_string(count);

    // tf_listener
    bool success = tf_listener();
    if(!success) return;
    double x = transform.getOrigin().x();
    double y = transform.getOrigin().y();
    double z = transform.getOrigin().z();
    double q_x = transform.getRotation().x();
    double q_y = transform.getRotation().y();
    double q_z = transform.getRotation().z();
    double q_w = transform.getRotation().w();
    double roll, pitch, yaw;
    tf::Matrix3x3(tf::Quaternion(q_x, q_y, q_z, q_w)).getRPY(roll ,pitch, yaw);
    printf("tf:x:%.2f y:%.2f z:%.2f roll:%.2f pitch:%.2f yaw:%.2f\n", x, y, z, roll, pitch, yaw);

    // save PointCloud
    pcl::PointCloud<pcl::PointXYZI> cloud;
    pcl::fromROSMsg(pc2, cloud);
    cloud.width = 1;
    cloud.height = cloud.points.size();
    pcl::io::savePCDFile(package_path+"/data/cloud/"+file_name+".pcd", cloud);

    // tf
    std::ofstream log;
    log.open(package_path+"/data/tf/"+file_name+".csv" ,std::ios::trunc);
    log << x   << ", " 
        << y   << ", "
        << z   << ", "
        << q_x << ", "
        << q_y << ", "
        << q_z << ", "
        << q_w;
    log.close();

    count++;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "data_saver");

    DataSaver ds;

    ros::spin();

    return 0;
}
