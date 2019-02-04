#include <Gicp.h>

namespace GRAPH_SLAM
{

    template<typename T_p>
    Gicp<T_p>::Gicp(){}


    template<typename T_p>
    void Gicp<T_p>::gicp(typename pcl::PointCloud<T_p>::Ptr& source_cloud, 
            typename pcl::PointCloud<T_p>::Ptr& target_cloud, 
            Eigen::Matrix4f& transformation_matrix)
    {
        std::cout<<"gicp"<<std::endl;

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

} //GRAPH_SLAM
