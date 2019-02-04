#include <ros/ros.h>
#include <ros/package.h>
#include <tf/tf.h>
#include <mapping/util.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "test");

    Util ul;

    double roll = 0;
    double pitch = 0;
    double yaw = 3.14;

    Eigen::Vector3f euler;
    Eigen::Matrix3f matrix;
    Eigen::Quaternionf quaternion;

    euler(0) = roll; euler(1) = pitch; euler(2) = yaw;
    matrix = ul.rpy2mat(euler);
    quaternion = ul.rpy2quat(euler);

    std::cout<<"euler to .."<<std::endl;
    std::cout<<"euler\n"
             <<" x:"<<euler(0)<<std::endl
             <<" y:"<<euler(1)<<std::endl
             <<" z:"<<euler(2)<<std::endl;
    std::cout<<"matrix\n"<<matrix<<std::endl;
    std::cout<<"quaternion\n"
             <<" x:"<<quaternion.x()<<std::endl
             <<" y:"<<quaternion.y()<<std::endl
             <<" z:"<<quaternion.z()<<std::endl
             <<" w:"<<quaternion.w()<<std::endl;

    euler = ul.mat2rpy(matrix);
    quaternion = ul.mat2quat(matrix);
    std::cout<<"matrix to .."<<std::endl;
    std::cout<<"euler\n"
             <<" x:"<<euler(0)<<std::endl
             <<" y:"<<euler(1)<<std::endl
             <<" z:"<<euler(2)<<std::endl;
    std::cout<<"matrix\n"<<matrix<<std::endl;
    std::cout<<"quaternion\n"
             <<" x:"<<quaternion.x()<<std::endl
             <<" y:"<<quaternion.y()<<std::endl
             <<" z:"<<quaternion.z()<<std::endl
             <<" w:"<<quaternion.w()<<std::endl;


    euler = ul.quat2rpy(quaternion);
    matrix = ul.quat2mat(quaternion);

    euler = ul.mat2rpy(matrix);
    quaternion = ul.mat2quat(matrix);
    std::cout<<"quaternion to .."<<std::endl;
    std::cout<<"euler\n"
             <<" x:"<<euler(0)<<std::endl
             <<" y:"<<euler(1)<<std::endl
             <<" z:"<<euler(2)<<std::endl;
    std::cout<<"matrix\n"<<matrix<<std::endl;
    std::cout<<"quaternion\n"
             <<" x:"<<quaternion.x()<<std::endl
             <<" y:"<<quaternion.y()<<std::endl
             <<" z:"<<quaternion.z()<<std::endl
             <<" w:"<<quaternion.w()<<std::endl;


    return 0;
}
