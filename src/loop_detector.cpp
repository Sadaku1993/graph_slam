/*
 *
 * src : loop_detector.cpp
 * Author : Yudai Sadakuni
 *
 * pkg : mapping
 * createrd : 2019.1.8
 * latestupdate : 2019.1.8
 *
 * memo : 再訪判定に使用. 任意のNodeから一定範囲内にある別のNodeを探し、
 *        ２つのNodeの位置情報と点群情報を保存
 */

#include <iostream>
#include <ros/ros.h>
#include <ros/package.h>

#include <sensor_msgs/PointCloud.h>

#include <Eigen/Core>

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <mapping/util.h>

#include <tf/tf.h>

#include <unistd.h>
#include <sys/types.h>
#include <pwd.h>

// inputをdelimiterでsplit
std::vector<std::string> split(std::string& input, char delimiter)
{
    std::istringstream stream(input);
    std::string field;
    std::vector<std::string> result;
    while (getline(stream, field, delimiter)) {
        result.push_back(field);
    }
    return result;
}

class LoopDetector{
    private:
        ros::NodeHandle nh;

        std::string package_path;

        double distance;
        double dist;

        std::vector< tf::Transform > transforms;

        Util ul;

    public:
        LoopDetector();

        void load(std::string file_path);
        void main();
};

LoopDetector::LoopDetector()
    : nh("~")
{
    package_path = ros::package::getPath("mapping");

    nh.param<double>("distance", distance, 5.0);
    nh.param<double>("dist", dist, 20);
}

void LoopDetector::load(std::string file_path)
{
    std::ifstream ifs(file_path);

    if(ifs.fail()){
        std::cout<<" File is None "<<std::endl;
        exit(0);
    }

    std::string line;
    while(getline(ifs, line)){
        std::vector<std::string> strvec = split(line, ' ');
        if(strvec.at(0) != "VERTEX_SE3:QUAT")
            continue;

        tf::Transform tf;
        tf::Vector3 tf_vector(std::stof(strvec.at(2)),
                              std::stof(strvec.at(3)),
                              std::stof(strvec.at(4)));
        tf::Quaternion tf_quaternion(std::stof(strvec.at(5)),
                                  std::stof(strvec.at(6)),
                                  std::stof(strvec.at(7)),
                                  std::stof(strvec.at(8)));
        tf.setOrigin(tf_vector);
        tf.setRotation(tf_quaternion);
        transforms.push_back(tf);
    }
}

void LoopDetector::main()
{
    // bfr.csv and aft.csv are saved at home directory
    struct passwd *pw = getpwuid(getuid());
    const char *homedir = pw->pw_dir;
    std::string bfr_path = std::string(homedir) + "/bfr.csv";
    std::string aft_path = std::string(homedir) + "/aft.csv";

    load(aft_path);

    std::cout << transforms.size() << std::endl;

    for(size_t i=0;i<transforms.size();i++){

        size_t id = 0;
        float min_dist = INFINITY;
        for(size_t j=0;j<transforms.size();j++){
            if(i==j) continue;
            float delta_x = transforms[i].getOrigin().x() - transforms[j].getOrigin().x();
            float delta_y = transforms[i].getOrigin().y() - transforms[j].getOrigin().y();
            float delta_z = transforms[i].getOrigin().z() - transforms[j].getOrigin().z();
            float delta = sqrt(delta_x*delta_x + delta_y*delta_y + delta_z*delta_z);
            if(delta<min_dist){
                id = j;
                min_dist = delta;
            }
        }
        
        if(distance<min_dist) continue;
        std::cout<< i <<" "<< id <<" "<< min_dist << std::endl;
    }

    std::string mv = "mv "+aft_path+' '+bfr_path;
    std::cout<<mv<<std::endl;
    std::system(mv.c_str());

    std::string g2o = std::string(homedir)+"/g2o/bin/tutorial_slam3d";
    std::cout<<g2o<<std::endl;
    std::system(g2o.c_str());

    transforms.clear();
    load(aft_path);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "loop_detector");

    LoopDetector ld;
    
    ld.main();

    return 0;
}
