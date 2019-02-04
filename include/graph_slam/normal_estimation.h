// #include <pcl/point_types.h>
// #include <pcl/filters/extract_indices.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/features/normal_3d_omp.h>

template<typename T_p>
class NormalEstimation{
    private:
        double search_radius;

    public:
        NormalEstimation();
        void normal_estimation(typename pcl::PointCloud<T_p>::Ptr&,
                               typename pcl::PointCloud<T_p>::Ptr&);
};

template<typename T_p>
NormalEstimation<T_p>::NormalEstimation()
{
    search_radius = 0.80;
}

// Normal Estimation
template<typename T_p>
void NormalEstimation<T_p>::normal_estimation(typename pcl::PointCloud<T_p>::Ptr& cloud, 
                                              typename pcl::PointCloud<T_p>::Ptr& normal_cloud)
{
    pcl::NormalEstimationOMP<T_p, pcl::Normal> ne;
    ne.setInputCloud(cloud);
    typename pcl::search::KdTree<T_p>::Ptr tree (new pcl::search::KdTree<T_p> ());
    ne.setSearchMethod (tree);
    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
    ne.setRadiusSearch (search_radius);
    ne.compute(*normals);

    T_p tmp;
    for(size_t i=0;i<cloud->points.size();i++)
    {
		tmp.x = cloud->points[i].x;
		tmp.y = cloud->points[i].y;
		tmp.z = cloud->points[i].z;
		if(!std::isnan(normals->points[i].normal_x)){
			tmp.normal_x = normals->points[i].normal_x;
		}
		else{
			tmp.normal_x = 0.0;
		}
		if(!std::isnan(normals->points[i].normal_y)){
			tmp.normal_y = normals->points[i].normal_y;
		}
		else{
			tmp.normal_y = 0.0;
		}
		if(!std::isnan(normals->points[i].normal_z)){
			tmp.normal_z = normals->points[i].normal_z;
		}
		else{
			tmp.normal_z = 0.0;
		}
		if(!std::isnan(normals->points[i].curvature)){
			tmp.curvature = normals->points[i].curvature;
		}
		else{
			tmp.curvature = 0.0;
		}
		normal_cloud->points.push_back(tmp);
	}
}
