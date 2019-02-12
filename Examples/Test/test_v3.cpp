#include <ros/ros.h>
#include <ros/package.h>
#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>
#include <File.h>

class Test{
  private:
    ros::NodeHandle nh;

    std::string package_path;
    std::string tf_path;

  public:
    Test();

    GRAPH_SLAM::Util Util;

    void main();
    void printTF(tf::Transform);
};


Test::Test()
  : nh("~")
{
  package_path = ros::package::getPath("graph_slam");
  nh.param<std::string>("tf_path", tf_path, "/data/csv");
  tf_path.insert(0, package_path);
}

void Test::printTF(tf::Transform transform)
{
  std::cout<<"transform"<<std::endl;
}
  
void Test::main()
{
  std::cout<<"main"<<std::endl;

  // loadTF
  std::string odometry_name = tf_path + "odometry.csv";
  std::vector< ID > transforms;
  File.loadTF(transforms, odometry_name);

  for (auto itr=transforms.begin(); itr!=transforms.end()-1; itr++)
  {
    std::cout<<"Node:"<<itr->id<<" "<<(itr+1)->id<<std::endl;
    tf::Transform source_transform = itr->transform();
    tf::Transform target_transform = (itr+1)->transform();

    Eigen::Affine3d source_affine;
    Eigen::Affine3d target_affine;

    tf::transformTFToEigen(source_transform, source_affine);
    tf::transformTFToEigen(target_transform, target_affine);

    Eigen::Affine3d relative = source_affine.inverse() * target_affine;
    
    Eigen::Affine3d reconstruct = source_affine * relative;

    std::cout<<"source affine"<<std::endl;
    std::cout<<source_affine.matrix()<<std::endl;

    std::cout<<"target affine"<<std::endl;
    std::cout<<target_affine.matrix()<<std::endl;

    std::cout<<"relative"<<std::endl;
    std::cout<<relative.matrix()<<std::endl;

    std::cout<<"reconstruct"<<std::endl;
    std::cout<<reconstruct.matrix()<<std::endl;

  }

}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "text_v3");

  Test test;

  test.main();

  return 0;
}
