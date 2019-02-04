#ifndef SYSTEM_H
#define SYSTEM_H

#include <string>
#include <thread>
#include <iostream>
#include <G2O.h>
#include <Gicp.h>

namespace GRAPH_SLAM
{

class G2O;

template<typename T_p> 
class Gicp;

class System
{
public:
    System();

    void main();

private:

    G2O* mpG2O;
    Gicp<pcl::PointXYZINormal>* mpGicp;
};

}  // namespace GRAPH_SLAM

#endif
