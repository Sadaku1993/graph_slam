#ifndef GRAPHSLAM_H
#define GRAPHSLAM_H

#include <ros/ros.h>
#include <ros/package.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include <tf_conversions/tf_eigen.h>

#include <Gicp.h>
#include <G2O.h>
#include <File.h>
#include <LoopCloser.h>

class SLAM3D{
    private:
        ros::NodeHandle nh;
        std::string package_path;
        std::string cloud_path;
        std::string tf_path;

        int THRESHOLD;
        double DISTANCE;
        
        int count;
    
    public:
        SLAM3D();

        GRAPH_SLAM::File File;
        GRAPH_SLAM::LoopCloser LoopCloser;;
        GRAPH_SLAM::Gicp<pcl::PointXYZINormal> Gicp;
        GRAPH_SLAM::G2O G2O;

        void loop_detector(size_t);
        
        template<typename T_p>
        void gicp(LoopNode, std::string);
        
        void g2o(std::string, std::string);
        void main();
};

#endif
