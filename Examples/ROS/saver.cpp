#include <graph_slam/Saver.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "Saver");

    Saver<pcl::PointXYZINormal> sv;

    ros::spin();

    return 0;
}
