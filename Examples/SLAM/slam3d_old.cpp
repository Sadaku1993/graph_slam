#include <ros/ros.h>
#include <ros/package.h>
#include <System.h>
#include <G2O.h>

class Slam3d
{
    private:
        ros::NodeHandle nh;
        std::string package_path;
        std::string cloud_path;
        std::string tf_path;
    public:
        Slam3d();

        GRAPH_SLAM::System System;
        GRAPH_SLAM::G2O G2O;
        void main();
};

Slam3d::Slam3d()
    : nh("~")
{
    package_path = ros::package::getPath("graph_slam");
    nh.param<std::string>("cloud_path", cloud_path, "/data/remove/");
    nh.param<std::string>("tf_path", tf_path, "/data/csv/");

    cloud_path.insert(0, package_path);
    tf_path.insert(0, package_path);
}

void Slam3d::main()
{
  std::string gicp_csv = tf_path + "gicp.csv";
  std::string g2o_csv = tf_path + "g2o.csv";

  std::cout<<gicp_csv<<std::endl;
  std::cout<<g2o_csv<<std::endl;

  G2O.g2o(gicp_csv, g2o_csv);

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "slam3d");

    Slam3d sd;

    sd.main();

    return 0;
}
