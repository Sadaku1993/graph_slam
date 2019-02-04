#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl_ros/transforms.h>

#include <tf/tf.h>

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

template<typename T_p>
class Function{
    public:
        int file_count_boost(const boost::filesystem::path& root);
        
        bool loadCloud(typename pcl::PointCloud<T_p>::Ptr& cloud, std::string file_name);
        void saveCloud(typename pcl::PointCloud<T_p>::Ptr& cloud, std::string save_name);

        bool loadTF(tf::Transform& transform, std::string file_name);

        void transform_pointcloud(typename pcl::PointCloud<T_p>::Ptr& input_cloud, 
                                  typename pcl::PointCloud<T_p>::Ptr& trans_cloud, 
                                  tf::Transform tf);
};

// path内にデータがいくつがあるか確認
template<typename T_p>
int Function<T_p>::file_count_boost(const boost::filesystem::path& root) {
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
template<typename T_p>
bool Function<T_p>::loadCloud(typename pcl::PointCloud<T_p>::Ptr& cloud, std::string file_name)
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
template<typename T_p>
void Function<T_p>::saveCloud(typename pcl::PointCloud<T_p>::Ptr& cloud, std::string file_name)
{
    cloud->width = 1;
    cloud->height = cloud->points.size();

    pcl::io::savePCDFile(file_name, *cloud);
}

// TFを読み込み
template<typename T_p>
bool Function<T_p>::loadTF(tf::Transform& transform, std::string file_name)
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
template<typename T_p>
void Function<T_p>::transform_pointcloud(typename pcl::PointCloud<T_p>::Ptr& cloud,
                                         typename pcl::PointCloud<T_p>::Ptr& trans_cloud,
                                         tf::Transform tf)
{
    pcl_ros::transformPointCloud(*cloud, *trans_cloud, tf);
}
