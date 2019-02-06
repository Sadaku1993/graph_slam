#ifndef FILE_H
#define FILE_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include <iostream>
#include <fstream>
#include <tf/tf.h>

namespace GRAPH_SLAM
{

class File{
    public:
        File();

        std::vector<std::string> split(std::string& input, char delimiter);
        int file_count_boost(const boost::filesystem::path& root);

        template<typename T_p>
        bool loadCloud(typename pcl::PointCloud<T_p>::Ptr&, std::string);
        
        template<typename T_p>
        bool saveCloud(typename pcl::PointCloud<T_p>::Ptr&, std::string);

        bool loadTF(std::vector< tf::Transform >&, std::string);

    private:
};

}

#endif
