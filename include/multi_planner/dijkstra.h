#pragma once
#include "ros/ros.h"
#include "nav_msgs/Path.h"
#include "traverse_expander.h"
using namespace std;
class DijkstraExpantion:public TraverseExpander{
private:
    vector<uint32_t> *current_,*next_;//index buffer 

public:
    DijkstraExpantion();
    ~DijkstraExpantion();
    bool plan(uint16_t sx,uint16_t sy,uint16_t gx,uint16_t gy,nav_msgs::Path &res);
    bool get_path();
    float get_cost(uint32_t s,uint32_t g);


    
};






