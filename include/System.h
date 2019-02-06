#ifndef SYSTEM_H
#define SYSTEM_H

#include <string>
#include <thread>
#include <iostream>
#include <G2O.h>
#include <Gicp.h>
#include <Util.h>
#include <File.h>
#include <PCL.h>

namespace GRAPH_SLAM
{

class G2O;

template<typename T_p> 
class Gicp;

class Util;

class File;

template<typename T_p>
class PCL;

class System
{
public:
    System();

    void main(std::string path);

private:
    G2O* mpG2O;
    Gicp<pcl::PointXYZINormal>* mpGicp;
    Util* mpUtil;
    File* mpFile;
    PCL<pcl::PointXYZINormal>* mpPCL;
};

}  // namespace GRAPH_SLAM

#endif
