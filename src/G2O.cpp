#include<G2O.h>

namespace GRAPH_SLAM
{

void G2O::g2o(std::string bfr_file_name,
              std::string aft_file_name)
{
  std::cout<<"g2o"<<std::endl;

  typedef BlockSolver< BlockSolverTraits<-1, -1> >  SlamBlockSolver;
  typedef LinearSolverCSparse<SlamBlockSolver::PoseMatrixType> SlamLinearSolver;

  // allocating the optimizer
  SparseOptimizer optimizer;
  auto linearSolver = g2o::make_unique<SlamLinearSolver>();
  linearSolver->setBlockOrdering(false);
  OptimizationAlgorithmGaussNewton* solver = new OptimizationAlgorithmGaussNewton(
      g2o::make_unique<SlamBlockSolver>(std::move(linearSolver)));

  optimizer.setAlgorithm(solver);

  optimizer.load(bfr_file_name.c_str());

  VertexSE3* firstRobotPose = dynamic_cast<VertexSE3*>(optimizer.vertex(0));
  firstRobotPose->setFixed(true);
  optimizer.setVerbose(true);

  std::cerr << "Optimizing" << std::endl;
  optimizer.initializeOptimization();
  optimizer.optimize(10);
  std::cerr << "done." << std::endl;

  optimizer.save(aft_file_name.c_str());

  optimizer.clear();
}


void G2O::main()
{
    std::cout<<"start"<<std::endl;

    // bfr.csv and aft.csv is saved at home directory
    struct passwd *pw = getpwuid(getuid());
    const char *homedir = pw->pw_dir;
    std::string bfr_file_path = std::string(homedir) + "/bfr.csv";
    std::string aft_file_path = std::string(homedir) + "/aft.csv";
    
    typedef BlockSolver< BlockSolverTraits<-1, -1> >  SlamBlockSolver;
    typedef LinearSolverCSparse<SlamBlockSolver::PoseMatrixType> SlamLinearSolver;

    // allocating the optimizer
    SparseOptimizer optimizer;
    auto linearSolver = g2o::make_unique<SlamLinearSolver>();
    linearSolver->setBlockOrdering(false);
    OptimizationAlgorithmGaussNewton* solver = new OptimizationAlgorithmGaussNewton(
            g2o::make_unique<SlamBlockSolver>(std::move(linearSolver)));

    optimizer.setAlgorithm(solver);

    optimizer.load(bfr_file_path.c_str());
    // optimizer.load(file_path);

    // prepare and run the optimization
    // fix the first robot pose to account for gauge freedom
    VertexSE3* firstRobotPose = dynamic_cast<VertexSE3*>(optimizer.vertex(0));
    firstRobotPose->setFixed(true);
    optimizer.setVerbose(true);

    std::cerr << "Optimizing" << std::endl;
    optimizer.initializeOptimization();
    optimizer.optimize(10);
    std::cerr << "done." << std::endl;

    optimizer.save(aft_file_path.c_str());
    // optimizer.save("aft.csv");

    // freeing the graph memory
    optimizer.clear();
}

} //namespace G2O
