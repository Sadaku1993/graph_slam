#include <ros/ros.h>
#include <ros/package.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseArray.h>

#include <File.h>

template<typename T_p>
class LoopCloser{
  private:
    ros::NodeHandle nh;
    std::string package_path;
    std::string tf_path;
    int THRESHPLD;

  public:
    LoopCloser();

    GRAPH_SLAM::File File;
    void main();
};

template<typename T_p>
LoopCloser<T_p>::LoopCloser()
  :nh("~")
{
    package_path = ros::package::getPath("graph_slam");
    nh.param<std::string>("tf_path", tf_path, "/data/csv/");
    tf_path.insert(0, package_path);

    THRESHPLD = 30;
}

struct NodeEdge{
  int source_id;
  int target_id;
  tf::Transform source_transform;
  tf::Transform target_transform;
};

template<typename T_p>
void LoopCloser<T_p>::main()
{
  std::cout<<"Loop Closer"<<std::endl;

  std::string tf_name = tf_path + "gicp.csv";

  std::vector< ID > transforms;
  File.loadTF(transforms, tf_name);

  std::cout<<"Node Num : "<<transforms.size()<<std::endl;

  std::vector< NodeEdge > loop_data;
  bool init[int(transforms.size())] = {false};

  for(size_t i=0;i<transforms.size(); i++){
    double min_distance = INFINITY;
    NodeEdge node_edge;
    bool flag = false;
    for(size_t j=0;j<transforms.size(); j++){

      if(init[i] && init[j]) break;

      tf::Transform source_transform = transforms[i].transform;
      tf::Transform target_transform = transforms[j].transform;
      tf::Transform edge_transform = source_transform.inverseTimes(target_transform);

      double distance = sqrt( pow(edge_transform.getOrigin().x(), 2) + 
                              pow(edge_transform.getOrigin().y(), 2) + 
                              pow(edge_transform.getOrigin().z(), 2) );

      // Node間が近いものは考慮しない
      if(min_distance<distance && abs(int(i-j)) < THRESHPLD) continue;

      min_distance = distance;
      node_edge.source_id = i;
      node_edge.target_id = j;
      node_edge.source_transform = source_transform;
      node_edge.target_transform = target_transform;
      flag = true;
    }

    if(flag){
      init[node_edge.source_id] = true;
      init[node_edge.target_id] = true;
      loop_data.push_back(node_edge);
    }
  }


  for(auto itr=loop_data.begin(); itr!=loop_data.end(); itr++)
  {
    std::cout<<itr->source_id<<" "<<itr->target_id<<std::endl;
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "LoopCloser");

  LoopCloser<pcl::PointXYZINormal> lc;

  lc.main();

  return 0;
}
