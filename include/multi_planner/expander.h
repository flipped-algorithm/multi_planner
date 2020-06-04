#pragma onece
#include <iostream>
#include <nav_msgs/OccupancyGrid.h>
#include <queue>
#define INF_VALUE      100000
//obstacle value
#define CRITICAL_VALUE 50     

using namespace std;
class Expander
{

public:
    Expander()
    {
        nx_ = 10;
        ny_ = 10;
        ns_ = nx_ * ny_;
        map_ =nullptr;
        mask_=nullptr;
        costs_=nullptr;
    }
    virtual ~Expander(){
        delete mask_;
        delete map_;
        delete costs_;
    }
    inline uint32_t to_index(uint16_t x, uint16_t y){
        return x + y * nx_;
    }

    void set_map(nav_msgs::OccupancyGrid &m)
    {
        resolution_=m.info.resolution;
        nx_=m.info.width;
        ny_=m.info.height;
        ns_ = nx_ * ny_;

        if(map_!=nullptr){
            delete map_;
        }
        map_=new int8_t[ns_];
        map_=m.data.data();

        if(mask_ != nullptr){
            delete mask_;
        }
        mask_=new bool[ns_];
        std::fill(mask_,mask_+ns_,false);

        if(costs_!=nullptr){
            delete costs_;
        }
        costs_=new float[ns_];
        std::fill(costs_,costs_+ns_,INF_VALUE);
    }

    inline bool is_in_map(uint32_t index){
        return index<ns_;     
    }

protected:
    uint32_t nx_, ny_,ns_; // map size,in pixel
    float    *costs_;      //cost from start to each point
    bool     *mask_;       //visited or not
    int8_t   *map_;        //map data
    float    resolution_;
};
