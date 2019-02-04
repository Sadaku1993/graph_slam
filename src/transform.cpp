/*
 * transform.cpp
 *
 * transform pointcloud
 *
 * author:Yudai Sadakuni
 *
 */

#include <ros/ros.h>
#include <ros/package.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <tf/tf.h>
#include <pcl_ros/transforms.h>
#include <pcl/io/pcd_io.h>

typedef pcl::PointXYZ PointA;
typedef pcl::PointCloud<PointA> CloudA;
typedef pcl::PointCloud<PointA>::Ptr CloudAPtr;

std::vector<std::string> split(std::string& input, char delimiter)
{
    std::istringstream stream(input);
    std::string field;
    std::vector<std::string> result;
    while (getline(stream, field, delimiter)) {
        result.push_back(field);
    }
    return result;
}

template<typename T_p, typename T_c, typename T_ptr>
class Transform{
    private:
        ros::NodeHandle nh;
        std::string package_path;
        std::string cloud_path;
        std::string tf_path;
        std::string save_path;
    public:
        Transform();
        int file_count_boost(const boost::filesystem::path& root);
        bool loadCloud(T_ptr& cloud, std::string file_name);
        void saveCloud(T_ptr& cloud, std::string save_name);
        bool loadTF(tf::Transform& transform, std::string file_name);
        void transform_pointcloud(T_ptr& input_cloud, T_ptr& trans_cloud, tf::Transform tf);
        void main();
};

template<typename T_p, typename T_c, typename T_ptr>
Transform<T_p, T_c, T_ptr>::Transform()
    : nh("~")
{
    package_path = ros::package::getPath("mapping");

    nh.param<std::string>("cloud_path", cloud_path, "/data/cloud");
    nh.param<std::string>("tf_path", tf_path, "/data/tf");
    nh.param<std::string>("save_path", save_path, "/data/trans_cloud");

    cloud_path.insert(0, package_path);
    tf_path.insert(0, package_path);
    save_path.insert(0, package_path);

}

// path内にデータがいくつがあるか確認
template<typename T_p, typename T_c, typename T_ptr>
int Transform<T_p, T_c, T_ptr>::file_count_boost(const boost::filesystem::path& root) {
    namespace fs = boost::filesystem;
    if (!fs::exists(root) || !fs::is_directory(root)) return 0;
    int result = 0;
    fs::directory_iterator last;
    for (fs::directory_iterator pos(root); pos != last; ++pos) {
        ++result;
        if (fs::is_directory(*pos)) result += file_count_boost(pos->path());
    }
    return result;
}

// Cloudデータを読み込み
template<typename T_p, typename T_c, typename T_ptr>
bool Transform<T_p, T_c, T_ptr>::loadCloud(T_ptr& cloud, std::string file_name)
{
    if(pcl::io::loadPCDFile<T_p> (file_name, *cloud) == -1){
        // std::cout<<file_name<<" is none"<<std::endl;
        return false;
    }else{
        // std::cout<<"Load "<<file_name<<" is success"<<std::endl;
        return true;
    }
}

// Cloudデータを保存
template<typename T_p, typename T_c, typename T_ptr>
void Transform<T_p, T_c, T_ptr>::saveCloud(T_ptr& cloud, std::string file_name)
{
    cloud->width = 1;
    cloud->height = cloud->points.size();

    pcl::io::savePCDFile(file_name, *cloud);
}


// TFを読み込み
template<typename T_p, typename T_c, typename T_ptr>
bool Transform<T_p, T_c, T_ptr>::loadTF(tf::Transform& transform, std::string file_name)
{
    double x; double y; double z;
    double q_x; double q_y; double q_z; double q_w;
    bool flag = false;

    std::ifstream ifs(file_name);
    std::string line;
    while(getline(ifs, line)){
            std::vector<std::string> strvec = split(line , ',');
            x = stof(strvec.at(0));
            y = stof(strvec.at(1));
            z = stof(strvec.at(2));
            q_x = stof(strvec.at(3));
            q_y = stof(strvec.at(4));
            q_z = stof(strvec.at(5));
            q_w = stof(strvec.at(6));
            flag = true;
    }

    transform.setOrigin(tf::Vector3(x, y, z));
    transform.setRotation(tf::Quaternion(q_x, q_y, q_z, q_w));
    double roll, pitch, yaw;
    tf::Matrix3x3(tf::Quaternion(q_x, q_y, q_z, q_w)).getRPY(roll ,pitch, yaw);
    printf("tf:x:%.2f y:%.2f z:%.2f roll:%.2f pitch:%.2f yaw:%.2f\n", x, y, z, roll, pitch, yaw);
    return flag;
}


// transform pointcloud
template<typename T_p, typename T_c, typename T_ptr>
void Transform<T_p, T_c, T_ptr>::transform_pointcloud(T_ptr& cloud,
                                                      T_ptr& trans_cloud,
                                                      tf::Transform tf)
{
    pcl_ros::transformPointCloud(*cloud, *trans_cloud, tf);
}


// main
template<typename T_p, typename T_c, typename T_ptr>
void Transform<T_p, T_c, T_ptr>::main()
{

    int file_size = file_count_boost(cloud_path.c_str());
    
    for(int i=0;i<file_size;i++){
        T_ptr cloud(new T_c);
        tf::Transform transform;

        std::string cloud_name = cloud_path + "/" + std::to_string(i) + ".pcd";
        std::string tf_name = tf_path + "/" + std::to_string(i) + ".csv";
        std::string save_name = save_path + "/" + std::to_string(i) + ".pcd";

        // load
        bool cloud_flag = loadCloud(cloud, cloud_name);
        bool tf_flag = loadTF(transform, tf_name);
        if(!cloud_flag && !tf_flag)
            continue;

        // transform PointCloud
        T_ptr trans_cloud(new T_c);
        transform_pointcloud(cloud, trans_cloud, transform);

        // save PointCloud
        saveCloud(trans_cloud, save_name);
    }
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "transform_pointcloud");

    Transform<PointA, CloudA, CloudAPtr> tf;

    tf.main();

    return 0;
}
