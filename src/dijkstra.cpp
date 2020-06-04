#include "multi_planner/dijkstra.h"
#include "geometry_msgs/PointStamped.h"
#include "multi_planner/node.h"
#include "vector"

DijkstraExpantion::DijkstraExpantion()
{
    current_=new vector<uint32_t>;
    next_=new vector<uint32_t>;

    //for four direction
    Node n;
    n.x=0;n.y=-1;neighbor_.emplace_back(n);
    n.x=-1;n.y=0;neighbor_.emplace_back(n);
    n.x=1;n.y=0;neighbor_.emplace_back(n);
    n.x=0;n.y=1;neighbor_.emplace_back(n);
    
    //for eight direction
    //n.x_=-1;n.y_=-1;neighbor_.emplace_back(n);
    //n.x_=1;n.y_=-1;neighbor_.emplace_back(n);
    //n.x_=-1;n.y_=1;neighbor_.emplace_back(n);
    //n.x_=1;n.y_=1;neighbor_.emplace_back(n);
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

//    for(int j=9;j>=0;j--){
//        for(int i=0;i<10;i++){
//            printf("  %8.0f",costs_[to_index(i,j)]);
//        }
//        printf("\r\n");
//    }

    get_path(sx,sy,gx,gy,res);

    return sta;
}

float DijkstraExpantion::get_cost(uint32_t s,uint32_t g){
    
    if(map_[g]>CRITICAL_VALUE){//obstacle
        return INF_VALUE;
    }
    //return c.cost_+hypot(c.x_-x,c.y_-y)*map_->info.resolution;
    return costs_[s]+1.0;

}

bool DijkstraExpantion::get_path(uint16_t sx,uint16_t sy,uint16_t gx,uint16_t gy,nav_msgs::Path &res){  
    uint32_t start_index= to_index(gx,gy);
    int cx,cy,mx,my;
    float min_value=INF_VALUE;

    cx=sx;
    cy=sy;

    geometry_msgs::PoseStamped tp;
    tp.header.frame_id="/map";
    tp.pose.orientation.w=1.0;
    tp.pose.position.x=cx*0.05f;
    tp.pose.position.y=cy*0.05f;
    res.poses.emplace_back(tp);
    
    while (to_index(cx,cy)!=start_index)
    {
        bool hited = false;
        //printf("cx=%d, cy=%d,len=%f\r\n",cx,cy,costs_[to_index(cx,cy)]);
        for(int k=0;k<neighbor_.size();k++){
            int i=cx+neighbor_[k].x;
            int j=cy+neighbor_[k].y;
            if((i>=0 && i <nx_)&&(j>=0 && j < ny_)  ){
                uint32_t index=to_index(i,j);

                if(costs_[index]<min_value){
                    min_value=costs_[index];
                    mx=i;
                    my=j;
                    hited=true;
                }
            }
        }
        if(!hited){
            printf("test\r\n");
            return false;
            break;
        }

        cx=mx;
        cy=my;

        tp.pose.position.x=cx*resolution_;
        tp.pose.position.y=cy*resolution_;
        res.poses.emplace_back(tp);
    }
    
}