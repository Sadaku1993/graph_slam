#ifndef PCL_H
#define PCL_H

#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/io/pcd_io.h>
#include <tf/tf.h>
#include <pcl_ros/transforms.h>

#define MIN(x, y) ((x) < (y) ? (x) : (y))
#define MAX(x, y) ((x) > (y) ? (x) : (y))

struct Cluster{
    float x; 
    float y; 
    float z;
    float width;
    float height;
    float depth;
    float curvature;
    Eigen::Vector3f min_p;
    Eigen::Vector3f max_p;
};

namespace GRAPH_SLAM
{

template<typename T_p>
class PCL{
public:
    PCL();

    void min_max(typename pcl::PointCloud<T_p>::Ptr&,
                 typename pcl::PointCloud<T_p>::Ptr&,
                 typename pcl::PointCloud<T_p>::Ptr&,
                 int grid_dimentions,
                 double threshold,
                 double cell_size);
    void remove_cluster(typename pcl::PointCloud<T_p>::Ptr&,
                        typename pcl::PointCloud<T_p>::Ptr&,
                        double leaf_size,
                        int min_size,
                        int max_size,
                        double torelance);
    void getClusterInfo(typename pcl::PointCloud<T_p>::Ptr&,
                            Cluster&);
    
    void NormalEstimation(typename pcl::PointCloud<T_p>::Ptr&,
                          typename pcl::PointCloud<pcl::PointXYZINormal>::Ptr&,
                          double search_radius);

    void TransformPointCloud(typename pcl::PointCloud<T_p>::Ptr& cloud,
                             typename pcl::PointCloud<T_p>::Ptr& trans_cloud,
                             tf::Transform tf);

private:

};

template class PCL<pcl::PointXYZI>;
template class PCL<pcl::PointXYZINormal>;

} // namespace GRAPH_SLAM

#endif // PCL
