#include "LoopCloser.h"

namespace GRAPH_SLAM
{

LoopCloser::LoopCloser()
{
    mpFile = new File();
}

std::vector<LoopNode> LoopCloser::main(int THRESHPLD, double DISTANCE, std::string CSV)
{
    std::cout<<"Loop Closer"<<std::endl;

    std::vector< ID > transforms;
    mpFile->loadTF(transforms, CSV);

    bool flag[transforms.size()] = {false};

    std::vector< LoopNode > LoopNodes;

    for(size_t i=0;i<transforms.size();i++){
        std::vector<double> list;
        if(flag[i]) continue;
        for(size_t j=0;j<transforms.size();j++){
            tf::Transform source_transform = transforms[i].transform;
            tf::Transform target_transform = transforms[j].transform;
            tf::Transform edge_transform = source_transform.inverseTimes(target_transform);

            double distance = sqrt( pow(edge_transform.getOrigin().x(), 2) + 
                    pow(edge_transform.getOrigin().y(), 2) +  
                    pow(edge_transform.getOrigin().z(), 2) );
            if(abs(int(i-j)) < THRESHPLD || flag[j]) distance = INFINITY;
            list.push_back(distance);
        }
        std::vector<double>::iterator minIt = std::min_element(list.begin(), list.end());
        size_t minIndex = std::distance(list.begin(), minIt);
        flag[i] = true;
        flag[minIndex] = true;

        if(DISTANCE<*minIt) continue;
        // std::cout<<"  Node "<<i<<" "<<minIndex<<" "<<*minIt<<std::endl;

        LoopNode ln;
        ln.source_id = i;
        ln.target_id = minIndex;
        ln.source_transform = transforms[i].transform;
        ln.target_transform = transforms[minIndex].transform;
        LoopNodes.push_back(ln);
    }

    return LoopNodes;
}

void LoopCloser::loop_closer(int THRESHPLD, double DISTANCE, std::vector<ID> transforms)
{
    bool loop[transforms.size()] = {false};
}

}
