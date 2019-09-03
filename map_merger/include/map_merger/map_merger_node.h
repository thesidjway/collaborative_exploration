#pragma once
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/ColorRGBA.h>
#include <vector>
#include <utility>
#include <mutex>

ros::Subscriber octo_sub;
ros::Subscriber map_sub;
ros::Publisher occupied_pub;
const double map_res = 0.05;
const double octo_res = 0.4;
const int width = 125;
const int width_map = 1000;
const int height = 80;
const int height_map = 640;
const double x_orig = -25;
const double y_orig = -16;

bool received_octomap = false;
int grid_2d[width][height];
visualization_msgs::MarkerArray last_octomap;
std::map<std::pair<int, int>, int> explored;
std::mutex octomap_lock;

void publish_updated(std::map<std::pair<int, int>, int>& explored_to_be_updated);
void map_callback(const nav_msgs::OccupancyGrid::ConstPtr& map);
void octo_callback(const visualization_msgs::MarkerArray::ConstPtr& octomap);


