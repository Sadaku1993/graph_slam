#include <pcl/point_cloud.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

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

template<typename T_p>
class Clustering{
    public:
        void getClusterInfo(typename pcl::PointCloud<T_p>::Ptr&,
                            Cluster& cluster);
};

template<typename T_p>
void Clustering<T_p>::getClusterInfo(typename pcl::PointCloud<T_p>::Ptr& pt,
                                      Cluster& cluster)
{
    Eigen::Vector3f centroid;
    centroid[0]=pt->points[0].x;
    centroid[1]=pt->points[0].y;
    centroid[2]=pt->points[0].z;
    
    Eigen::Vector3f min_p;
    min_p[0]=pt->points[0].x;
    min_p[1]=pt->points[0].y;
    min_p[2]=pt->points[0].z;

    Eigen::Vector3f max_p;
    max_p[0]=pt->points[0].x;
    max_p[1]=pt->points[0].y;
    max_p[2]=pt->points[0].z;

    for(size_t i=1;i<pt->points.size();i++){
        centroid[0]+=pt->points[i].x;
        centroid[1]+=pt->points[i].y;
        centroid[2]+=pt->points[i].z;
        if (pt->points[i].x<min_p[0]) min_p[0]=pt->points[i].x;
        if (pt->points[i].y<min_p[1]) min_p[1]=pt->points[i].y;
        if (pt->points[i].z<min_p[2]) min_p[2]=pt->points[i].z;

        if (pt->points[i].x>max_p[0]) max_p[0]=pt->points[i].x;
        if (pt->points[i].y>max_p[1]) max_p[1]=pt->points[i].y;
        if (pt->points[i].z>max_p[2]) max_p[2]=pt->points[i].z;
    }

    cluster.x=centroid[0]/(float)pt->points.size();
    cluster.y=centroid[1]/(float)pt->points.size();
    cluster.z=centroid[2]/(float)pt->points.size();
    cluster.depth  = max_p[0]-min_p[0];
    cluster.width  = max_p[1]-min_p[1];
    cluster.height = max_p[2]-min_p[2]; 
    cluster.min_p = min_p;
    cluster.max_p = max_p;
}
