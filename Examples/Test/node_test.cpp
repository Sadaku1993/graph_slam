#include <ros/ros.h>
#include <ros/package.h>
#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>
#include <File.h>
#include <Util.h>

class NodeTest{
    
    private:
        ros::NodeHandle nh;

        std::string package_path;
        std::string cloud_path;
        std::string tf_path;

    public:
        NodeTest();
        
        GRAPH_SLAM::Util Util;
        GRAPH_SLAM::File File;

        void main();
};


NodeTest::NodeTest()
    : nh("~")
{
    package_path = ros::package::getPath("graph_slam");
    nh.param<std::string>("cloud_path", cloud_path, "/data/remove/");
    nh.param<std::string>("tf_path", tf_path, "/data/csv/");
    cloud_path.insert(0, package_path);
    tf_path.insert(0, package_path);
}

void NodeTest::main()
{
    std::cout<<"main"<<std::endl;

    // loadTF
    std::string odometry_name = tf_path + "odometry.csv";
    std::vector< ID > transforms;
    File.loadTF(transforms, odometry_name);

    Eigen::Vector3d integration_translation = Eigen::Vector3d::Zero(3);
    Eigen::Matrix3d integration_rotation = Eigen::Matrix3d::Identity();

    for(auto itr=transforms.begin(); itr!=transforms.end()-1; itr++)
    {
        tf::Transform source_transform = itr->transform;
        tf::Transform target_transform = (itr+1)->transform;

        Eigen::Affine3d source_affine;
        Eigen::Affine3d target_affine;
        tf::transformTFToEigen(target_transform, target_affine);
        tf::transformTFToEigen(source_transform, source_affine);
        Eigen::Matrix4d target_matrix = target_affine.matrix();
        Eigen::Matrix4d source_matrix = source_affine.matrix();
  
        tf::Transform transform = source_transform.inverseTimes(target_transform);
        Util.printTF(transform);

        Eigen::Matrix4d test = source_matrix.inverse() * target_matrix;
        std::cout<<test<<std::endl;

        Eigen::Affine3d affine;
        tf::transformTFToEigen(transform, affine);
        Eigen::Matrix4d matrix = affine.matrix();

        Eigen::Vector3d translation = affine.translation();
        Eigen::Matrix3d rotation = affine.rotation();

        std::cout<<"  source matrix\n  "<<source_matrix<<std::endl;
        std::cout<<"  target matrix\n  "<<target_matrix<<std::endl;

        std::cout<<"  affine\n  "<<affine.matrix().cast<float>()<<std::endl;

        // integration_translation = integration_translation + translation;
        // integration_rotation = rotation * integration_rotation;

        // Eigen::Translation<double, 3> trans(integration_translation);

        // Eigen::Affine3d integration_affine = trans * integration_rotation;

        // integration_matrix = integration_affine.matrix();


        /*
        Eigen::Matrix4f relative_matrix = affine.cast<float> ();
        
        // translation
        Eigen::Translation<float, 3> translation = Eigen::Translation<float, 3>(relative_matrix(0, 3), relative_matrix(1, 3), relative_matrix(2, 3));
        // rotation
        Eigen::Matrix3f rotation;
        rotation << relative_matrix(0, 0), relative_matrix(0, 1), relative_matrix(0, 2),
                    relative_matrix(1, 0), relative_matrix(1, 1), relative_matrix(1, 2),
                    relative_matrix(2, 0), relative_matrix(2, 1), relative_matrix(2, 2);
        Eigen::Quaternionf quaternion(rotation);
        // affine
        Eigen::Affine3f matrix;
        matrix = translation*quaternion;

        integration_matrix = matrix * integration_matrix;
        */

        // std::cout<<"  target matrix\n  "<<target_matrix<<std::endl;
 
        printf("\n");
    }
    
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "NodeTest");

    NodeTest nt;

    nt.main();

    return 0;
}
