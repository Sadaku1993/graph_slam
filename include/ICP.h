#ifndef ICP_H
#define ICP_H

#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/registration/icp.h>

namespace GRAPH_SLAM
{

template<typename T_p>
class ICP
{
    public:
        ICP();

        void icp(typename pcl::PointCloud<T_p>::Ptr& source_cloud,
                 typename pcl::PointCloud<T_p>::Ptr& target_cloud,
                 Eigen::Matrix4d& transform_matrix);

    private:

};

template class ICP<pcl::PointXYZ>;

}

#endif
