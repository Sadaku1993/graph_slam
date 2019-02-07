#include <PCL.h>

namespace GRAPH_SLAM{

template<typename T_p>
PCL<T_p>::PCL(){}

// min max algorithm
template<typename T_p>
void PCL<T_p>::min_max(typename pcl::PointCloud<T_p>::Ptr& cloud, 
                       typename pcl::PointCloud<T_p>::Ptr& ground_cloud, 
                       typename pcl::PointCloud<T_p>::Ptr& obstacle_cloud,
                       int grid_dimentions,
                       double threshold,
                       double cell_size)
{
    float min[grid_dimentions][grid_dimentions];
    float max[grid_dimentions][grid_dimentions];
    bool init[grid_dimentions][grid_dimentions];

    memset(&min,  0, grid_dimentions*grid_dimentions);
    memset(&max,  0, grid_dimentions*grid_dimentions); 
    memset(&init, 0, grid_dimentions*grid_dimentions);

// #pragma omp parallel for
    for(size_t i=0; i<cloud->points.size(); i++)
    {
        int x = (grid_dimentions/2) + cloud->points[i].x/cell_size;
        int y = (grid_dimentions/2) + cloud->points[i].y/cell_size;

        if(0<=x && x<grid_dimentions && 0<=y && y<grid_dimentions)
        {
            if(!init[x][y]){
                min[x][y] = cloud->points[i].z;
                max[x][y] = cloud->points[i].z;
                init[x][y] = true;
            }
            else{
                min[x][y] = MIN(min[x][y], cloud->points[i].z);
                max[x][y] = MAX(max[x][y], cloud->points[i].z);
            }
        }
    }

    for(size_t i=0;i<cloud->points.size();i++)
    {
        int x = (grid_dimentions/2) + cloud->points[i].x/cell_size;
        int y = (grid_dimentions/2) + cloud->points[i].y/cell_size;

        if(0<=x && x<grid_dimentions && 0<=y && y<grid_dimentions){
            if(init[x][y] && max[x][y]-min[x][y]<threshold)
                ground_cloud->points.push_back(cloud->points[i]);
            else
                obstacle_cloud->points.push_back(cloud->points[i]);
        }else
            obstacle_cloud->points.push_back(cloud->points[i]);
    }
}

// Remove Cluster
template<typename T_p>
void PCL<T_p>::remove_cluster(typename pcl::PointCloud<T_p>::Ptr& cloud, 
                              typename pcl::PointCloud<T_p>::Ptr& cloud_removed,
                              double leaf_size,
                              int min_size,
                              int max_size,
                              double tolerance
                              )
{
    //Downsample//
    pcl::VoxelGrid<T_p> vg;
    typename pcl::PointCloud<T_p>::Ptr ds_cloud(new pcl::PointCloud<T_p>);
    vg.setInputCloud (cloud);  
    vg.setLeafSize (leaf_size, leaf_size, leaf_size);
    vg.filter (*ds_cloud);

    //downsampled point's z =>0
    std::vector<float> tmp_z;
    tmp_z.resize(ds_cloud->points.size());
	for(int i=0;i<(int)ds_cloud->points.size();i++){
        tmp_z[i]=ds_cloud->points[i].z;
		ds_cloud->points[i].z  = 0.0;
    }

    //Clustering
    typename pcl::search::KdTree<T_p>::Ptr tree (new pcl::search::KdTree<T_p>);
    tree->setInputCloud (ds_cloud);
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<T_p> ec;
    ec.setClusterTolerance (tolerance);
    ec.setMinClusterSize (min_size);
    ec.setMaxClusterSize (max_size);
    ec.setSearchMethod (tree);
    ec.setInputCloud(ds_cloud);
    ec.extract (cluster_indices);
    
    //reset z value
	for(int i=0;i<(int)ds_cloud->points.size();i++)
        ds_cloud->points[i].z=tmp_z[i];

    // remove cluster
    std::vector<int> id_rm;
    std::vector<int> id_cluster;

    // check cluster size
    for(const auto& it : cluster_indices){
        typename pcl::PointCloud<T_p>::Ptr cluster_cloud(new pcl::PointCloud<T_p>);
        for(const auto& pit : it.indices){
            cluster_cloud->points.push_back(ds_cloud->points[pit]);
        }
        Cluster cluster;
        getClusterInfo(cluster_cloud, cluster);
        
        // pick up human size cluster
        if(0.3<cluster.width && cluster.width<1.0 && 
           0.3<cluster.depth && cluster.depth<1.0 && 
           1.0<cluster.height && cluster.height<2.0)
        {
            for(const auto& pit : it.indices){
                id_cluster.push_back(pit);
            }
        }
    }

    std::sort(id_cluster.begin(), id_cluster.end());

    int npoints = ds_cloud->points.size();
    
    std::vector<int>::iterator it = id_cluster.begin();

    for(int i=0;i<npoints; ++i){
        if(it == id_cluster.end() || i != *it){
            cloud_removed->points.push_back(ds_cloud->points[i]);
        }
        else{
            ++it;
        }
    }
}

template<typename T_p>
void PCL<T_p>::getClusterInfo(typename pcl::PointCloud<T_p>::Ptr& pt,
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

template<typename T_p>
void PCL<T_p>::NormalEstimation(typename pcl::PointCloud<T_p>::Ptr& cloud,
                                typename pcl::PointCloud<pcl::PointXYZINormal>::Ptr& normal_cloud,
                                double search_radius)
{
    pcl::NormalEstimationOMP<T_p, pcl::Normal> ne;
    ne.setInputCloud(cloud);
    typename pcl::search::KdTree<T_p>::Ptr tree (new pcl::search::KdTree<T_p> ());
    ne.setSearchMethod (tree);
    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
    ne.setRadiusSearch (search_radius);
    ne.compute(*normals);

    pcl::PointXYZINormal tmp;
    for(size_t i=0;i<cloud->points.size();i++)
    {
        tmp.x = cloud->points[i].x;
        tmp.y = cloud->points[i].y;
        tmp.z = cloud->points[i].z;
        tmp.intensity = cloud->points[i].intensity;
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

template<typename T_p>
void PCL<T_p>::TransformPointCloud(typename pcl::PointCloud<T_p>::Ptr& cloud,
                                   typename pcl::PointCloud<T_p>::Ptr& trans_cloud,
                                   tf::Transform tf)
{
    pcl_ros::transformPointCloud(*cloud, *trans_cloud, tf);
}

}
