#include <ros/ros.h>
#include "multi_planner/dijkstra.h"
#include "multi_planner/path_planner.h"


//subscribe topic /map
//put map to dijkstra/a star/d star
//get planed trajectory
//publish trajectory


int main(int argv,char *argc[]){

    ROS_INFO("System init...");
    ros::init(argv,argc,"path_planner_node");

    ROS_INFO("launch planner..."); 
    PathPlanner path_planner;

    ROS_INFO("run...");
    ros::spin();
} 