#ifndef LOOPCLOSER_H
#define LOOPCLOSER_H

#include <tf/tf.h>
#include <File.h>

struct LoopNode{
  int source_id;
  int target_id;
  tf::Transform source_transform;
  tf::Transform target_transform;
};

namespace GRAPH_SLAM{

class File;

class LoopCloser{
    private:
        File* mpFile;
    
    public:
        LoopCloser();
        std::vector<LoopNode> main(int, double, std::string);
};

}

#endif
