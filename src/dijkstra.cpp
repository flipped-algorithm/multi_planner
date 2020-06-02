#include "multi_planner/dijkstra.h"
#include "geometry_msgs/PointStamped.h"


DijkstraExpantion::DijkstraExpantion()
{
    current_=new vector<uint32_t>;
    next_=new vector<uint32_t>;

}

DijkstraExpantion::~DijkstraExpantion()
{
    delete current_;
    delete next_;
}

bool DijkstraExpantion::plan(uint16_t sx,uint16_t sy,uint16_t gx,uint16_t gy,nav_msgs::Path &res)
{
    
    bool sta=false;
    bool pending[ns_]={false};

    printf("planner sx=%d, sy=%d, gx=%d, gy=%d\r\n",sx,sy,gx,gy);


    //current_->emplace_back(to_index(gx,gy));
    mask_[to_index(gx,gy)]=true;
    costs_[to_index(gx,gy)]=0;
    push_neigbor(to_index(gx,gy),*current_,pending);
    
    
    

    while(!current_->empty()){
        for(int i=0;i<current_->size();i++){
           
            push_neigbor((*current_)[i],*next_,pending);
             mask_[(*current_)[i]]=true;

        }
        vector<uint32_t> *tmp=current_;
        current_=next_;
        next_=tmp;
        next_->clear();
    }

    for(int j=9;j>=0;j--){
        for(int i=0;i<10;i++){
            printf("  %8.0f",costs_[to_index(i,j)]);
        }
        printf("\r\n");
    }

    return sta;
}

float DijkstraExpantion::get_cost(uint32_t s,uint32_t g){
    
    if(map_[g]>50){//obstacle
        return INF_VALUE;
    }
    //return c.cost_+hypot(c.x_-x,c.y_-y)*map_->info.resolution;
    return costs_[s]+1.0;

}