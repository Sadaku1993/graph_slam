#include <File.h>

namespace GRAPH_SLAM{

File::File(){}

std::vector<std::string> File::split(std::string& input, char delimiter){
    std::istringstream stream(input);
    std::string field;
    std::vector<std::string> result;
    while (getline(stream, field, delimiter)) {
        result.push_back(field);
    }
    return result;
}

int File::file_count_boost(const boost::filesystem::path& root){
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

// loadCloud
template<typename T_p>
bool File::loadCloud(typename pcl::PointCloud<T_p>::Ptr& cloud,
                     std::string file_name){
    if(pcl::io::loadPCDFile<T_p> (file_name, *cloud) == -1) return false;
    else return true;
}
template bool File::loadCloud<pcl::PointXYZI>(pcl::PointCloud<pcl::PointXYZI>::Ptr&, std::string);
template bool File::loadCloud<pcl::PointXYZINormal>(pcl::PointCloud<pcl::PointXYZINormal>::Ptr&, std::string);

// saveCloud
template<typename T_p>
bool File::saveCloud(typename pcl::PointCloud<T_p>::Ptr& cloud,
                     std::string file_name){
    cloud->width = 1;
    cloud->height = cloud->points.size();

    if(pcl::io::savePCDFile(file_name, *cloud)) return true;
    else return false;
}
template bool File::saveCloud<pcl::PointXYZI>(pcl::PointCloud<pcl::PointXYZI>::Ptr&, std::string);
template bool File::saveCloud<pcl::PointXYZINormal>(pcl::PointCloud<pcl::PointXYZINormal>::Ptr&, std::string);

// loadTF
bool File::loadTF(std::vector<tf::Transform>& transforms,
                  std::string file_path)
{
    std::ifstream ifs(file_path);

    if(ifs.fail()){
        std::cout<<" File is None "<<std::endl;
        exit(0);
    }

    std::string line;
    while(getline(ifs, line)){
        std::vector<std::string> strvec = split(line, ' ');
        if(strvec.at(0) != "VERTEX_SE3:QUAT")
            continue;

        tf::Transform tf;
        tf::Vector3 tf_vector(std::stof(strvec.at(2)),
                              std::stof(strvec.at(3)),
                              std::stof(strvec.at(4)));
        tf::Quaternion tf_quaternion(std::stof(strvec.at(5)),
                                  std::stof(strvec.at(6)),
                                  std::stof(strvec.at(7)),
                                  std::stof(strvec.at(8)));
        tf.setOrigin(tf_vector);
        tf.setRotation(tf_quaternion);
        transforms.push_back(tf);
    }
}


}
