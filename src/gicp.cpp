/* gicp_v2.cpp
 *
 * save the relative coordinates between nodes 
 * and the absolute coordinates of each node
 *
 * author Yudai Sadakuni
 */

#include <ros/ros.h>
#include <ros/package.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/gicp.h>

#include <graph_slam/function.h>
#include <graph_slam/normal_estimation.h>
#include <graph_slam/util.h>

#include <fstream>

#include <unistd.h>
#include <sys/types.h>
#include <pwd.h>

using namespace std;

template<typename T_p>
class Gicp{
    private:
        ros::NodeHandle nh;
        std::string package_path;
        std::string cloud_path;
        std::string tf_path;
    public:
        Gicp();

        Function<T_p> Fc;
        Util Ul;

        void gicp(typename pcl::PointCloud<T_p>::Ptr& source_cloud, 
                  typename pcl::PointCloud<T_p>::Ptr& target_cloud, 
                  Eigen::Matrix4f& transformation_matrix);
        void odom_trans(tf::Transform source_frame,
                        tf::Transform target_frame,
                        tf::Transform& transform);
        void main();
};

template<typename T_p>
Gicp<T_p>::Gicp()
    : nh("~")
{
    package_path = ros::package::getPath("mapping");
    nh.param<std::string>("cloud_path", cloud_path, "/data/remove");
    nh.param<std::string>("tf_path", tf_path, "/data/tf");

    cloud_path.insert(0, package_path);
    tf_path.insert(0, package_path);
}

template<typename T_p>
void Gicp<T_p>::gicp(typename pcl::PointCloud<T_p>::Ptr& source_cloud, 
                     typename pcl::PointCloud<T_p>::Ptr& target_cloud, 
                     Eigen::Matrix4f& transformation_matrix)
{
    pcl::GeneralizedIterativeClosestPoint<T_p, T_p> gicp; 
    gicp.setMaxCorrespondenceDistance (0.7);
    gicp.setMaximumIterations (100);        
    gicp.setTransformationEpsilon (1e-8);   
    gicp.setEuclideanFitnessEpsilon (1e-8);
    gicp.setInputSource(source_cloud);
    gicp.setInputTarget(target_cloud);
    pcl::PointCloud<T_p> Final;
    gicp.align(Final);

    transformation_matrix = gicp.getFinalTransformation();
}

template<typename T_p>
void Gicp<T_p>::odom_trans(tf::Transform source_transform,
                           tf::Transform target_transform,
                           tf::Transform& transform)
{
    tf::Vector3 source = source_transform.getOrigin();
    tf::Vector3 target = target_transform.getOrigin();
    
    double s_roll, s_pitch, s_yaw;
    double t_roll, t_pitch, t_yaw;
    tf::Matrix3x3(source_transform.getRotation()).getRPY(s_roll, s_pitch, s_yaw);
    tf::Matrix3x3(target_transform.getRotation()).getRPY(t_roll, t_pitch, t_yaw);
    
    tf::Vector3 vector(target.x() - source.x(), 
                       target.y() - source.y(), 
                       target.z() - source.z());
    tf::Quaternion quaternion = tf::createQuaternionFromRPY(t_roll - s_roll,
                                                            t_pitch - s_pitch,
                                                            t_yaw - s_yaw);
    transform.setOrigin(vector);
    transform.setRotation(quaternion);
}


template<typename T_p>
void Gicp<T_p>::main()
{
    std::cout<<"main"<<std::endl;

    int file_size = Fc.file_count_boost(cloud_path.c_str());

    std::cout<<"file size"<<file_size<<std::endl;
    
    // bfr.csv is save at home directory
    struct passwd *pw = getpwuid(getuid());
    const char *homedir = pw->pw_dir;
    std::string file_path = std::string(homedir) + "/bfr.csv";
    
    std::ofstream ofs(file_path, std::ios::trunc);
    
    // absolute coordinates save
    ofs << "VERTEX_SE3:QUAT" <<" "<< 0 <<" "
        << 0.0 <<" "<< 0.0 <<" "<< 0.0 <<" "<< 0.0 <<" "<< 0.0 <<" "<< 0.0 <<" "<< 1.0 << std::endl;

    // relative coordinates save
    ofs << "EDGE_SE3:QUAT" <<" "<< 0   <<" "<< 0   <<" "
        << 0.0 <<" "<< 0.0 <<" "<< 0.0 <<" "<< 0.0 <<" "<< 0.0 <<" "<< 0.0 <<" "<< 1.0 << " "
        << 1.0 <<" "<< 0.0 <<" "<< 0.0 <<" "<< 0.0 <<" "<< 0.0 <<" "<< 0.0 <<" "
        << 1.0 <<" "<< 0.0 <<" "<< 0.0 <<" "<< 0.0 <<" "<< 0.0 <<" "
        << 1.0 <<" "<< 0.0 <<" "<< 0.0 <<" "<< 0.0 <<" "
        << 1.0 <<" "<< 0.0 <<" "<< 0.0 <<" "
        << 1.0 <<" "<< 0.0 <<" "
        << 1.0 << std::endl;

    // 積算
    Eigen::Matrix4f integration_matrix = Eigen::Matrix4f::Identity();

    for(int i=0;i<file_size-1;i++){
        std::cout << "\033[2J"; //画面クリア
        printf("\033[%d;%dH", 1, 1); //カーソル位置を、高さ1行目、横1行目に移動
        
        std::cout<<"node:"<<i<<" --- "<<i+1<<std::endl<<std::endl;
        typename pcl::PointCloud<T_p>::Ptr source_cloud(new pcl::PointCloud<T_p>);
        typename pcl::PointCloud<T_p>::Ptr target_cloud(new pcl::PointCloud<T_p>);
        tf::Transform source_transform;
        tf::Transform target_transform;

        std::string source_cloud_name = cloud_path + "/" + std::to_string(i) + ".pcd";
        std::string target_cloud_name = cloud_path + "/" + std::to_string(i+1) + ".pcd";
        std::string source_tf_name = tf_path + "/" + std::to_string(i) + ".csv";
        std::string target_tf_name = tf_path + "/" + std::to_string(i+1) + ".csv";

        // load 
        bool source_cloud_flag = Fc.loadCloud(source_cloud, source_cloud_name);
        bool target_cloud_flag = Fc.loadCloud(target_cloud, target_cloud_name);
        bool source_tf_flag = Fc.loadTF(source_transform, source_tf_name);
        bool target_tf_flag = Fc.loadTF(target_transform, target_tf_name);

        if(!source_cloud_flag || !target_cloud_flag || !source_tf_flag || !target_tf_flag)
            continue;

        // transform PointCloud
        typename pcl::PointCloud<T_p>::Ptr transform_source_cloud(new pcl::PointCloud<T_p>);
        typename pcl::PointCloud<T_p>::Ptr transform_target_cloud(new pcl::PointCloud<T_p>);
        Fc.transform_pointcloud(source_cloud, transform_source_cloud, source_transform);
        Fc.transform_pointcloud(target_cloud, transform_target_cloud, target_transform);

        printf("\n");

        // gicp
        Eigen::Matrix4f gicp_matrix;
        tf::Transform gicp_transform;
        gicp(target_cloud, source_cloud, gicp_matrix);
        gicp_transform = Ul.eigen2tf(gicp_matrix);
        std::cout<<"GICP"<<std::endl;
        Ul.printTF(gicp_matrix);
        // Ul.printTF(gicp_transform);

        printf("\n");

        // odometry trasnform
        tf::Transform odom_transform;
        Eigen::Matrix4f odom_matrix;
        odom_trans(source_transform, target_transform, odom_transform);
        odom_matrix = Ul.tf2eigen(odom_transform);
        std::cout<<"Odometry"<<std::endl;
        Ul.printTF(odom_transform);
        // Ul.printTF(odom_matrix);

        printf("\n");

        // Integrate transform matrix
        integration_matrix = integration_matrix * gicp_matrix;
        Eigen::Matrix3f rotation_matrix;
        rotation_matrix << integration_matrix(0, 0), integration_matrix(0, 1), integration_matrix(0, 2),
                           integration_matrix(1, 0), integration_matrix(1, 1), integration_matrix(1, 2),
                           integration_matrix(2, 0), integration_matrix(2, 1), integration_matrix(2, 2);
        Eigen::Quaternionf quaternion;
        quaternion = Ul.mat2quat(rotation_matrix);

        printf("Integration transform matrix\n");
        printf(" Transform vector : \n");
        printf(" t = < %6.3f %6.3f %6.3f >\n", integration_matrix(0, 3), integration_matrix(1, 3), integration_matrix(2, 3) );
        printf(" Quaternion : \n");
        printf(" q = < %6.3f %6.3f %6.3f %6.3f >\n", quaternion.x(), quaternion.y(), quaternion.z(), quaternion.w() );

        // absulute
        ofs << "VERTEX_SE3:QUAT" <<" "<< i+1 << " "
            << integration_matrix(0, 3) <<" "
            << integration_matrix(1, 3) <<" "
            << integration_matrix(2, 3) <<" "
            << quaternion.x() <<" "
            << quaternion.y() <<" "
            << quaternion.z() <<" "
            << quaternion.w()
            << std::endl;

        // relative
        ofs << "EDGE_SE3:QUAT" <<" "<< i <<" "<< i+1 <<" "
            << gicp_transform.getOrigin().x() <<" "
            << gicp_transform.getOrigin().y() <<" " 
            << gicp_transform.getOrigin().z() <<" "
            << gicp_transform.getRotation().x() <<" "
            << gicp_transform.getRotation().y() <<" "
            << gicp_transform.getRotation().z() <<" "
            << gicp_transform.getRotation().w() <<" "
            << 1.0 <<" "<< 0.0 <<" "<< 0.0 <<" "<< 0.0 <<" "<< 0.0 <<" "<< 0.0 <<" "
            << 1.0 <<" "<< 0.0 <<" "<< 0.0 <<" "<< 0.0 <<" "<< 0.0 <<" "
            << 1.0 <<" "<< 0.0 <<" "<< 0.0 <<" "<< 0.0 <<" "
            << 1.0 <<" "<< 0.0 <<" "<< 0.0 <<" "
            << 1.0 <<" "<< 0.0 <<" "
            << 1.0 << std::endl;

    }
    ofs.close();
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "gicp");

    Gicp<pcl::PointXYZINormal> gc;

    gc.main();

    return 0;
}
