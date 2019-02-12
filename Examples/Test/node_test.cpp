#include <ros/ros.h>
#include <ros/package.h>
#include <tf/tf.h>
#include <tf_eigen.h>
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

    Eigen::Matrix4f integration_matrix = Eigen::Matrix4f::Identity();

    for(auto itr=transforms.begin(); itr!=transforms.end()-1; itr++)
    {
        tf::Transform source_transform = itr->transform;
        tf::Transform target_transform = (itr+1)->transform;

        Eigen::Matrix4f source_matrix = Util.tf2eigen(source_transform);
        Eigen::Matrix4f target_matrix = Util.tf2eigen(target_transform);

        tf::Transform relative_transform = source_transform.inverseTimes(target_transform);
        Eigen::Matrix4f relative_matrix = Util.tf2eigen(relative_transform);

        std::cout<<"relative matrix\n"<<relative_matrix<<std::endl;

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

        Util.printTF(integration_matrix);
        Util.printTF(target_matrix);

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
