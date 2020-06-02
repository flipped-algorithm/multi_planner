#pragma  once

#include "expander.h"

class TraverseExpander:public Expander{
    
public:
    TraverseExpander(/* args */);
    virtual ~TraverseExpander();
    void push_neigbor(uint32_t index,vector<uint32_t> &buffer,bool *pending );
    virtual float get_cost(uint32_t s,uint32_t g)=0;
};





















