#include <System.h>

int main(int argc, char** argv)
{
    std::cout<<"slam3d"<<std::endl;

    GRAPH_SLAM::System SLAM;

    SLAM.main();

    return 0;
}
