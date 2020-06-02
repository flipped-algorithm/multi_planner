
#include "multi_planner/traverse_expander.h"
#define PUSH(x)  if(is_in_map(x)&&!mask_[x] &&!pending[x] &&map_[x]<50){buffer.emplace_back(x),pending[x]=true;float c=get_cost(index,x);if(c <costs_[x]) costs_[x]=c;}

TraverseExpander::TraverseExpander(/* args */)
{
}

TraverseExpander::~TraverseExpander()
{
}

void TraverseExpander::push_neigbor(uint32_t index,vector<uint32_t> &buffer,bool *pending ){
    if(index%nx_!=0)    PUSH(index-1)
    if(index%nx_!=ny_-1)PUSH(index+1)
    if(index%ny_!=0)    PUSH(index-nx_)
    if(index%ny_!=nx_-1)PUSH(index+nx_)
}


