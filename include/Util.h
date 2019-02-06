#ifndef UTIL_H
#define UTIL_H

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>

#include <pcl_ros/transforms.h>

namespace GRAPH_SLAM
{

class Util{
    public:
        Util();

        Eigen::Vector3f mat2rpy(Eigen::Matrix3f matrix);
        Eigen::Quaternionf mat2quat(Eigen::Matrix3f matrix);
        
        Eigen::Quaternionf rpy2quat(Eigen::Vector3f euler);
        Eigen::Matrix3f rpy2mat(Eigen::Vector3f euler);

        Eigen::Matrix3f quat2mat(Eigen::Quaternionf quaternion);
        Eigen::Vector3f quat2rpy(Eigen::Quaternionf quaternion);

        Eigen::Matrix4f tf2eigen(tf::Transform transform);
        tf::Transform eigen2tf(Eigen::Matrix4f matrix);

        template<typename T_p>
        void transform_pointcloud(typename pcl::PointCloud<T_p>::Ptr&,
                                  typename pcl::PointCloud<T_p>::Ptr&,
                                  tf::Transform tf);

        template<typename T>
        void printTF(T transform);
};

} //namespace GRAPH_SLAM

#endif // Util
