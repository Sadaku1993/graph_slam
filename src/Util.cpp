#include <Util.h>

namespace GRAPH_SLAM
{

    Util::Util(){}

    // Matrix to ...
    Eigen::Vector3f Util::mat2rpy(Eigen::Matrix3f matrix)
    {
        Eigen::Vector3f euler;
        euler = matrix.eulerAngles(0, 1, 2);
        return euler;
    }

    Eigen::Quaternionf Util::mat2quat(Eigen::Matrix3f matrix)
    {
        Eigen::Quaternionf quaternion(matrix);
        return quaternion;
    }


    // Euler to ...
    Eigen::Quaternionf Util::rpy2quat(Eigen::Vector3f euler)
    {
        Eigen::Matrix3f matrix;
        matrix = Eigen::AngleAxisf(euler(0), Eigen::Vector3f::UnitX())
            * Eigen::AngleAxisf(euler(1), Eigen::Vector3f::UnitY())
            * Eigen::AngleAxisf(euler(2), Eigen::Vector3f::UnitZ());
        Eigen::Quaternionf quaternion(matrix);
        return quaternion;
    }

    Eigen::Matrix3f Util::rpy2mat(Eigen::Vector3f euler)
    {
        Eigen::Matrix3f matrix;
        matrix = Eigen::AngleAxisf(euler(0), Eigen::Vector3f::UnitX())
            * Eigen::AngleAxisf(euler(1), Eigen::Vector3f::UnitY())
            * Eigen::AngleAxisf(euler(2), Eigen::Vector3f::UnitZ());
        return matrix;
    }


    // Quaternion to ...
    Eigen::Matrix3f Util::quat2mat(Eigen::Quaternionf quaternion)
    {
        Eigen::Matrix3f matrix = quaternion.toRotationMatrix();
        return matrix;
    }

    Eigen::Vector3f Util::quat2rpy(Eigen::Quaternionf quaternion)
    {
        Eigen::Matrix3f matrix = quaternion.toRotationMatrix();
        Eigen::Vector3f euler = matrix.eulerAngles(0, 1, 2);
        return euler;
    }

    // TF to Eigen
    Eigen::Matrix4f Util::tf2eigen(tf::Transform transform)
    {
        Eigen::Vector3d vector;
        Eigen::Matrix3d rotation;
        tf::vectorTFToEigen(tf::Vector3(transform.getOrigin()), vector);
        tf::matrixTFToEigen(tf::Matrix3x3(transform.getRotation()), rotation);

        Eigen::Matrix4f matrix;
        for(size_t i=0;i<3;i++)for(size_t j=0;j<3;j++)matrix(i, j) = rotation(i, j);
        matrix(0, 3) = vector(0);
        matrix(1, 3) = vector(1);
        matrix(2, 3) = vector(2);

        return matrix;
    }

    // eigen to TF
    tf::Transform Util::eigen2tf(Eigen::Matrix4f matrix)
    {
        Eigen::Matrix3f rotation_matrix;
        rotation_matrix << matrix(0, 0), matrix(0, 1), matrix(0, 2),
                        matrix(1, 0), matrix(1, 1), matrix(1, 2),
                        matrix(2, 0), matrix(2, 1), matrix(2, 2);

        Eigen::Quaternionf quaternionf = mat2quat(rotation_matrix);

        tf::Vector3 vector(matrix(0, 3), 
                matrix(1, 3), 
                matrix(2, 3));
        tf::Quaternion quaternion(quaternionf.x(),
                quaternionf.y(),
                quaternionf.z(),
                quaternionf.w());

        tf::Transform transform;
        transform.setOrigin(vector);
        transform.setRotation(quaternion);

        return transform;
    }


    template<typename T_p>
    void Util::transform_pointcloud(typename pcl::PointCloud<T_p>::Ptr& cloud,
                                    typename pcl::PointCloud<T_p>::Ptr& trans_cloud,
                                    tf::Transform tf)
    {
        pcl_ros::transformPointCloud(*cloud, *trans_cloud, tf);
    }

    template<>
    void Util::printTF<tf::Transform>(tf::Transform transform)
    {
        tf::Vector3 vector = transform.getOrigin();
        tf::Quaternion quaternion = transform.getRotation();
        double roll, pitch, yaw;
        tf::Matrix3x3(transform.getRotation()).getRPY(roll, pitch, yaw);

        printf(" Transform vector : \n");
        printf("  t = < %6.3f %6.3f %6.3f >\n", vector.x(), vector.y(), vector.z());
        printf(" Quaternion : \n");
        printf("  q = < %6.3f %6.3f %6.3f %6.3f >\n", quaternion.x(), quaternion.y(), quaternion.z(), quaternion.w());
        printf(" Euler : \n");
        printf("  e = < %6.3f %6.3f %6.3f >\n", roll, pitch, yaw);
    }


    template<>
    void Util::printTF<Eigen::Matrix4f>(Eigen::Matrix4f transform)
    {
        Eigen::Matrix3f matrix;

        Eigen::Vector3f vector;
        Eigen::Quaternionf quaternion;
        Eigen::Vector3f euler;

        matrix << transform(0, 0), transform(0, 1), transform(0, 2),
               transform(1, 0), transform(1, 1), transform(1, 2),
               transform(2, 0), transform(2, 1), transform(2, 2);

        vector << transform(0, 3), transform(1, 3), transform(2, 3);

        quaternion = mat2quat(matrix);
        euler = mat2rpy(matrix);

        printf(" Transform vector : \n");
        printf("  t = < %6.3f %6.3f %6.3f >\n", vector(0), vector(1), vector(2));
        printf(" Quaternion : \n");
        printf("  q = < %6.3f %6.3f %6.3f %6.3f >\n", quaternion.x(), quaternion.y(), quaternion.z(), quaternion.w());
        printf(" Euler : \n");
        printf("  e = < %6.3f %6.3f %6.3f >\n", euler(0), euler(1), euler(2));
    }

} //namespace GRAPH_SLAM
