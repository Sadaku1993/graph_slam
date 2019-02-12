#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/TransformStamped.h>

class Test{
  private:
    ros::NodeHandle nh;

    tf::Transform target_frame;
    tf::Transform source_frame;

  public:
    Test();

    void transformListener(tf::Transform&, std::string, std::string);
    void broadcast(tf::Transform, std::string, std::string);
    void main();
};

Test::Test()
  : nh("~")
{}

void Test::transformListener(tf::Transform& transform,
                    std::string target_frame,
                    std::string source_frame)
{
  tf::TransformListener listener;
  tf::StampedTransform stampedTransform;

  try{
    ros::Time now = ros::Time::now();
    listener.waitforTransform(target_frame, source_frame, 
                              now, ros::Duration(1.0));
    listener.lookupTransform(target_frame, source_frame, 
                             now, stampedTransform);
  }
  catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
  }

  transform.setOrigin(stampedTransform.getOrigin());
  transform.setRotation(stampedTransform.getRotation());
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
  transformListener(transform, "frame2", "frame1");

  geometry_msgs::Transform msg;
  tf::transformTFToMsg(transform, msg);

  std::cout<<msg<<std::endl;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "text_v2");

  Test test;

  ros::Rate rate(1);

  while(ros::ok())
  {
    test.main();

    rate.sleep(1);

    ros::spinOnce();
  }

  return 0;
}
