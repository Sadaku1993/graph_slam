#ifndef SYSTEM_H
#define SYSTEM_H

#include <string>
#include <thread>
#include <iostream>
#include <G2O.h>

namespace GRAPH_SLAM
{

class G2O;
// class Gicp;

class System
{
public:
    System();

    void main();

private:

    G2O* mpG2O;
    // Gicp* mpGicp;
};

}  // namespace GRAPH_SLAM

#endif
