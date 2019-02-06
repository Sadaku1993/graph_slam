#ifndef GICP_H
#define GICP_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <tf/tf.h>
#include <pcl/registration/gicp.h>

#include <fstream>

#include <unistd.h>
#include <sys/types.h>
#include <pwd.h>

namespace GRAPH_SLAM
{

template<typename T_p>
class Gicp
{
public:
    Gicp();
    
    void gicp(typename pcl::PointCloud<T_p>::Ptr& source_cloud, 
              typename pcl::PointCloud<T_p>::Ptr& target_cloud, 
              Eigen::Matrix4f& transformation_matrix);

private:

};

template class Gicp<pcl::PointXYZINormal>;

} // namespace Gicp
#endif // GICP
