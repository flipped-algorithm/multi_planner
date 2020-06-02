#pragma once

#include <iostream>

#define INF_COST 100000

class Node
{
public:
    uint16_t x_, y_; //site
    float cost_;

    Node(){
        x_=0;
        y_=0;
        cost_=0;
    }

    bool operator<(const Node& n) const
    {
        return cost_ > n.cost_;
    }

    bool operator==(const Node& n)const {
        return (x_==n.x_) && (y_==n.y_);
    }
};

