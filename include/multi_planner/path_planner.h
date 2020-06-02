#pragma once
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <iostream>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
#include <queue>
#include <algorithm>
#include <vector>

#include "dijkstra.h"
#include "multi_planner/dijkstra.h"

using namespace std;
using namespace ros;

using namespace geometry_msgs;


/*
 *test map size: 200x200 pixel
 *
 * 
 * */

class PathPlanner{
public:
    PathPlanner();
    ~PathPlanner();

    void exec(const nav_msgs::OccupancyGridConstPtr& grid);
private:
    NodeHandle nh_;
    Publisher pub_dijkstra_,pub_A_star_,pub_D_star_,pub_RRT_,pub_RRT_star_,pub_marker;
    Subscriber sub_map_;
    Point start,goal;
    int map_width,map_height;
    float resolution;

    DijkstraExpantion da;


    void set_marker(Point &start,Point &goal);

};