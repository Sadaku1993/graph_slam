#include <ICP.h>

namespace GRAPH_SLAM
{

template<typename T_p>
ICP<T_p>::ICP(){}

template<typename T_p>
void ICP<T_p>::icp(typename pcl::PointCloud<T_p>::Ptr& source_cloud,
                   typename pcl::PointCloud<T_p>::Ptr& target_cloud,
                   Eigen::Matrix4d& matrix)
{
    pcl::IterativeClosestPoint<T_p, T_p> icp;
    icp.setMaxCorrespondenceDistance (0.50);
    icp.setMaximumIterations (100);        
    icp.setTransformationEpsilon (1e-8);   
    icp.setEuclideanFitnessEpsilon (1e-8);
    icp.setInputCloud(source_cloud);
    icp.setInputTarget(target_cloud);

    pcl::PointCloud<T_p> Final;
    icp.align(Final);

    Eigen::Matrix4f icp_matrix = icp.getFinalTransformation();

    matrix = icp_matrix.cast<double>();
}

}

