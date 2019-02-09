#include <iostream>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/gicp.h>


void print4x4Matrix (const Eigen::Matrix4d & matrix)
{
  printf ("Rotation matrix :\n");
  printf ("    | %6.3f %6.3f %6.3f | \n", matrix (0, 0), matrix (0, 1), matrix (0, 2));
  printf ("R = | %6.3f %6.3f %6.3f | \n", matrix (1, 0), matrix (1, 1), matrix (1, 2));
  printf ("    | %6.3f %6.3f %6.3f | \n", matrix (2, 0), matrix (2, 1), matrix (2, 2));
  printf ("Translation vector :\n");
  printf ("t = < %6.3f, %6.3f, %6.3f >\n\n", matrix (0, 3), matrix (1, 3), matrix (2, 3));
}

class Registration{
    public:
        Registration(std::string source, std::string target);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_target;
        void icp();
        void gicp();

        void main();

    private:
};

Registration::Registration(std::string source, std::string target)
    : cloud_source(new pcl::PointCloud<pcl::PointXYZ>), 
      cloud_target(new pcl::PointCloud<pcl::PointXYZ>)
{
    if(pcl::io::loadPCDFile(source, *cloud_source) < 0)
    {
        std::cerr << "Fail to read " <<source <<" file. Please pass path to the test." << std::endl;
        exit(0);
    }

    if(pcl::io::loadPCDFile(target, *cloud_target) < 0)
    {
        std::cerr << "Fail to read " << target<<" file. Please pass path to the test." << std::endl;
        exit(0);
    }
}

void Registration::icp()
{
    std::cout<<"ICP ";
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    // Set the input source and target
    icp.setInputSource(cloud_source);
    icp.setInputTarget(cloud_target);
    
    // set the maximum number of iterations(criterion 1)
    icp.setMaximumIterations(50);

    pcl::PointCloud<pcl::PointXYZ> Final;
    icp.align(Final);
    std::cout<<"had converted:"<<icp.hasConverged()<<" score: "<<icp.getFitnessScore() << std::endl;
    std::cout<<icp.getFinalTransformation()<<std::endl;

    std::string file_name = "icp.pcd";
    pcl::io::savePCDFile(file_name, Final);
}

void Registration::gicp()
{
    std::cout<<"GICP"<<std::endl;
}

void Registration::main()
{
    std::cout<<"start"<<std::endl;

    icp();
}


int main(int argc, char** argv)
{
    if(argc < 2)
    {
        std::cerr<<"No test files given. Please Download 'bun0.pcd', 'bin4.pcd' pass thier path to the test"<<std::endl;
        return (-1);
    }

    Registration rg(argv[1], argv[2]);

    rg.main();

    return 0;
}
