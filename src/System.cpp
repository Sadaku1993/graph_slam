#include "System.h"

namespace GRAPH_SLAM
{

System::System()
{
    mpG2O  = new G2O();
    mpGicp = new Gicp<pcl::PointXYZINormal>();
    mpUtil = new Util();
    mpFile = new File();
    mpPCL  = new PCL<pcl::PointXYZINormal>();
}

void System::main(std::string path)
{
    std::cout<<"System"<<std::endl;

    int file_size = mpFile->file_count_boost(path.c_str());
    std::cout<<"file size"<<file_size<<std::endl;
}

} //namespace GRAPH_SLAM
