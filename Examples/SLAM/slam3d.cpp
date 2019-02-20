#include <GraphSlam.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "GraphSlam");

    SLAM3D s3;

    s3.main();

    return 0;
}
