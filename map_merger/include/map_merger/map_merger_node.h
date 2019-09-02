#pragma once
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/MarkerArray.h>

ros::Subscriber octo_sub;
ros::Subscriber map_sub;
ros::Publisher merged_octo_pub;
int map_size = 1000;
double map_res = 0.0500000007451;


