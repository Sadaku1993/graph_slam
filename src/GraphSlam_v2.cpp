#include "GraphSlam.h"

SLAM3D::SLAM3D()
    : nh("~")
{
    package_path = ros::package::getPath("graph_slam");
    nh.param<std::string>("cloud_path", cloud_path, "/data/remove/");
    nh.param<std::string>("tf_path", tf_path, "/data/csv/");
    cloud_path.insert(0, package_path);
    tf_path.insert(0, package_path);

    nh.param<int>("THRESHOLD", THRESHOLD, 10);
    nh.param<double>("DISTANCE", DISTANCE, 1.5);

    count = 0;
}

void SLAM3D::loop_detector(size_t size)
{
    bool loop[size] = {false};

    for(size_t i=0;i<size;i++){
        // load CSV File
        std::string bfr_csv = tf_path + std::to_string(count) + ".csv";
        std::string aft_csv = tf_path + std::to_string(count+1) + ".csv";
        std::vector< ID > transforms;
        File.loadTF(transforms, bfr_csv);
        
        if(loop[i]) continue;

        std::vector<double> list;
        for(size_t j=0;j<size;j++){

            tf::Transform source_transform = transforms[i].transform;
            tf::Transform target_transform = transforms[j].transform;
            tf::Transform edge_transform = source_transform.inverseTimes(target_transform);

            double distance = sqrt( pow(edge_transform.getOrigin().x(), 2) + 
                    pow(edge_transform.getOrigin().y(), 2) +  
                    pow(edge_transform.getOrigin().z(), 2) );
            if(abs(int(i-j)) < THRESHOLD|| loop[j]) distance = INFINITY;
            list.push_back(distance);
        }
        std::vector<double>::iterator minIt = std::min_element(list.begin(), list.end());
        size_t minIndex = std::distance(list.begin(), minIt);

        if(DISTANCE<*minIt) continue;
        loop[i] = true; 
        loop[minIndex] = true;

        LoopNode ln;
        ln.source_id = i;
        ln.target_id = minIndex;
        ln.source_transform = transforms[i].transform;
        ln.target_transform = transforms[minIndex].transform;

        // GICP
        gicp<pcl::PointXYZINormal>(ln, bfr_csv);

        // G2O
        g2o(bfr_csv, aft_csv);

        count++;
    }
}

void SLAM3D::g2o(std::string bfr_csv, std::string aft_csv)
{
    G2O.g2o(bfr_csv, aft_csv);
}

template<typename T_p>
void SLAM3D::gicp(LoopNode ln, std::string csv)
{
    // ------------Load PointCloud and Transform---------
    typename pcl::PointCloud<T_p>::Ptr source_cloud(new pcl::PointCloud<T_p>);
    typename pcl::PointCloud<T_p>::Ptr target_cloud(new pcl::PointCloud<T_p>);
    File.loadCloud<T_p>(source_cloud, cloud_path+std::to_string(ln.source_id)+".pcd");
    File.loadCloud<T_p>(target_cloud, cloud_path+std::to_string(ln.target_id)+".pcd");

    // ----------------Relative transform----------------
    tf::Transform edge_transform = ln.source_transform.inverseTimes(ln.target_transform);

    // -------------------GICP---------------------------
    typename pcl::PointCloud<T_p>::Ptr trans_cloud(new pcl::PointCloud<T_p>);
    pcl_ros::transformPointCloud(*target_cloud, *trans_cloud, edge_transform);

    Eigen::Matrix4d gicp_matrix;
    Eigen::Affine3d gicp_affine;
    Gicp.gicp(source_cloud, trans_cloud, gicp_matrix, 1.0);
    gicp_affine = gicp_matrix;
    tf::Transform gicp_transform;
    tf::transformEigenToTF(gicp_affine, gicp_transform);

    Eigen::Affine3d edge_affine;
    tf::transformTFToEigen(edge_transform, edge_affine);

    Eigen::Matrix4d check = gicp_matrix - Eigen::Matrix4d::Identity();
    if(3.0< check.norm()){
        std::cout<<"Matching Miss"<<std::endl;
        return;
    }
    
    Eigen::Affine3d affine;
    tf::Transform transform;
    affine = gicp_affine.inverse() * edge_affine;
    tf::transformEigenToTF(affine, transform);
    
    std::ofstream ofs(csv, std::ios::app);
    ofs << "EDGE_SE3:QUAT" <<" "<< ln.source_id <<" "<< ln.target_id <<" "
        <<transform.getOrigin().x()<<" "
        <<transform.getOrigin().y()<<" "
        <<transform.getOrigin().z()<<" "
        <<transform.getRotation().x()<<" "
        <<transform.getRotation().y()<<" "
        <<transform.getRotation().z()<<" "
        <<transform.getRotation().w()<<" "
        <<1.0<<" "<<0.0<<" "<<0.0<<" "<<0.0<<" "<<0.0<<" "<<0.0<<" "
        <<1.0<<" "<<0.0<<" "<<0.0<<" "<<0.0<<" "<<0.0<<" "
        <<1.0<<" "<<0.0<<" "<<0.0<<" "<<0.0<<" "
        <<1.0<<" "<<0.0<<" "<<0.0<<" "
        <<1.0<<" "<<0.0<<" "
        <<1.0
        <<std::endl;
    ofs.close();
}

void SLAM3D::main()
{
    // ---------------First Graph Optimization--------------
    std::string gicp_csv = tf_path + "gicp.csv";
    std::string g2o_csv = tf_path + std::to_string(count) + ".csv";
    g2o(gicp_csv, g2o_csv);

    // ---------------------LoopClose-----------------------
    std::vector< ID > transforms;
    File.loadTF(transforms, g2o_csv);
    loop_detector(transforms.size());
}
