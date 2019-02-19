#ifndef G2O_H
#define G2O_H

#include <iostream>
#include <cmath>

#include "g2o/g2o/types/slam3d/vertex_se3.h"
#include "g2o/g2o/types/slam3d/vertex_pointxyz.h"
#include "g2o/g2o/types/slam3d/edge_se3.h"
#include "g2o/g2o/types/slam3d/edge_se3_pointxyz.h"
#include "g2o/g2o/types/slam3d/se3quat.h"
#include "g2o/g2o/types/slam3d/parameter_se3_offset.h"

#include "g2o/g2o/core/sparse_optimizer.h"
#include "g2o/g2o/core/block_solver.h"
#include "g2o/g2o/core/factory.h"
#include "g2o/g2o/core/optimization_algorithm_factory.h"
#include "g2o/g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/g2o/solvers/csparse/linear_solver_csparse.h"


#include <unistd.h>
#include <sys/types.h>
#include <pwd.h>

using namespace std;
using namespace g2o;

namespace GRAPH_SLAM
{

class G2O{
    private:

    public:
        void main();
        void g2o(std::string, std::string);
};

} // namespace GRAPH_SLAM

#endif // G2O
