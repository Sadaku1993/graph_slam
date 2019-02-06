#include <ros/ros.h>
#include <ros/package.h>
#include <System.h>

class Slam3d
{
    private:
        ros::NodeHandle nh;
        std::string package_path;
        std::string cloud_path;
        std::string tf_path;
    public:
        Slam3d();

        GRAPH_SLAM::System SLAM;
        void main();
};

Slam3d::Slam3d()
    : nh("~")
{
    package_path = ros::package::getPath("graph_slam");
    nh.param<std::string>("cloud_path", cloud_path, "/data/remove");
    nh.param<std::string>("tf_path", tf_path, "/data/tf");

    cloud_path.insert(0, package_path);
    tf_path.insert(0, package_path);
}

void Slam3d::main()
{
    SLAM.main(cloud_path);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "slam3d");

    Slam3d sd;

    sd.main();

    return 0;
}
