#include <boost/bind.hpp>
#include "multi_planner/path_planner.h"


PathPlanner::PathPlanner()
{

    pub_dijkstra_ = nh_.advertise<nav_msgs::Path>("dijkstra", 10);
    pub_A_star_ = nh_.advertise<nav_msgs::Path>("A_star", 10);
    pub_D_star_ = nh_.advertise<nav_msgs::Path>("D_star", 10);
    pub_RRT_ = nh_.advertise<nav_msgs::Path>("RRT", 10);
    pub_RRT_star_ = nh_.advertise<nav_msgs::Path>("RRT_star", 10);
    pub_marker = nh_.advertise<visualization_msgs::Marker>("marker", 1);

 //   start.x = 2;
 //   start.y = 14;
 //   goal.x = 2;
 //   goal.y = 16;
    start.x = 0;
    start.y = 0;
    goal.x = 9;
    goal.y = 9;
    sub_map_ = nh_.subscribe<nav_msgs::OccupancyGrid>("map", 10, boost::bind(&PathPlanner::exec, this, _1));

    

}

PathPlanner::~PathPlanner()
{
    sub_map_.~Subscriber();
    pub_dijkstra_.~Publisher();
    pub_A_star_.~Publisher();
    pub_D_star_.~Publisher();
    pub_RRT_.~Publisher();
    pub_RRT_star_.~Publisher();

    nh_.~NodeHandle();
}

void PathPlanner::exec(const nav_msgs::OccupancyGridConstPtr &grid)
{
    ROS_INFO("\n geted map:\n resolution: %f\n widthxheight: %dx%d\n", grid->info.resolution, grid->info.width, grid->info.height);

    set_marker(start, goal);
    map_width = grid->info.width;
    map_height = grid->info.height;
    resolution = grid->info.resolution;

    nav_msgs::OccupancyGrid mp= *grid;
    da.set_map(mp);
    nav_msgs::Path da_path;
    da_path.header.frame_id="/map";
    da_path.header.stamp=ros::Time::now();
    ROS_INFO("start dijkstra planning...");
    da.plan(20 ,260,20,320,da_path);
    ROS_INFO("dijkstra planning finished.");
    pub_dijkstra_.publish(da_path);

}

void PathPlanner::set_marker(Point &s, Point &g)
{

    visualization_msgs::Marker mk;

    mk.header.frame_id = "/map";
    mk.header.stamp = ros::Time::now();
    mk.ns = "marker";

    mk.action = visualization_msgs::Marker::ADD;

    mk.pose.orientation.x = 0.0;
    mk.pose.orientation.y = 0.0;
    mk.pose.orientation.z = 0.0;
    mk.pose.orientation.w = 1.0;

    mk.scale.x = 0.4;
    mk.scale.y = 0.4;
    mk.scale.z = 0.1;

    mk.lifetime = ros::Duration();

    mk.pose.position = s;
    //start
    mk.id = 1;
    mk.color.r = 0.0f;
    mk.color.g = 1.0f;
    mk.color.b = 0.0f;
    mk.color.a = 1.0;
    mk.type = visualization_msgs::Marker::CYLINDER;
    pub_marker.publish(mk);
    //goal
    mk.pose.position = g;

    mk.id = 2;
    mk.color.r = 1.0f;
    mk.color.g = 0.0f;
    mk.color.b = 0.0f;
    mk.color.a = 1.0;
    mk.type = visualization_msgs::Marker::CUBE;
    pub_marker.publish(mk);
    ROS_INFO("set marker to map!");
}
