#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/TransformStamped.h>

class Test{
  private:
    ros::NodeHandle nh;

    tf::Transform target_frame;
    tf::Transform source_frame;

    tf::TransformListener listener;

  public:
    Test();

    bool transformListener(tf::Transform&, std::string, std::string);
    void broadcast(tf::Transform, std::string, std::string);
    void main();
};

Test::Test()
  : nh("~")
{}

bool Test::transformListener(tf::Transform& transform,
                    std::string source_frame,
                    std::string target_frame)
{
  tf::StampedTransform stampedTransform;

  try{
    ros::Time now = ros::Time::now();
    listener.waitForTransform(source_frame, target_frame, now, ros::Duration(1.0));
    listener.lookupTransform(source_frame, target_frame,  now, stampedTransform);
    transform.setOrigin(stampedTransform.getOrigin());
    transform.setRotation(stampedTransform.getRotation());
    return true;
  }
  catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
      return false;
  }
}

void Test::broadcast(tf::Transform transform,
                     std::string frame_id,
                     std::string child_frame_id)
{
  static tf2_ros::TransformBroadcaster br;
  geometry_msgs::TransformStamped transformStamped;
  transformStamped.header.stamp = ros::Time::now();
  transformStamped.header.frame_id = frame_id;
  transformStamped.child_frame_id = child_frame_id;
  tf::transformTFToMsg(transform, transformStamped.transform);
  br.sendTransform(transformStamped);
}

void Test::main()
{
  tf::Transform transform;
  if(!transformListener(transform, "frame1", "frame2")) return;

  tf::Transform source_transform;
  if(!transformListener(source_transform, "map", "frame1")) return;

  tf::Transform target_transform;
  if(!transformListener(target_transform, "map", "frame2")) return;

  Eigen::Affine3d affine, source_affine, target_affine;
  tf::transformTFToEigen(transform, affine);
  tf::transformTFToEigen(source_transform, source_affine);
  tf::transformTFToEigen(target_transform, target_affine);
  
  std::cout<<"affine:"<<std::endl;
  std::cout<<affine.matrix()<<std::endl;

  std::cout<<"source affine"<<std::endl;
  std::cout<<source_affine.matrix()<<std::endl;

  std::cout<<"target affine"<<std::endl;
  std::cout<<target_affine.matrix()<<std::endl;

  // Eigen::Vector3d translation = affine.translation();
  // Eigen::Matrix3d rotation = affine.rotation();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "text_v2");

  Test test;

  ros::Rate rate(10);

  while(ros::ok())
  {
    test.main();
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
