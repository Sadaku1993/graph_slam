#include "System.h"

namespace GRAPH_SLAM
{

System::System()
{
    mpG2O = new G2O();
    mpGicp = new Gicp<pcl::PointXYZINormal>();
}

void System::main()
{
    std::cout<<"System"<<std::endl;
}

} //namespace GRAPH_SLAM
