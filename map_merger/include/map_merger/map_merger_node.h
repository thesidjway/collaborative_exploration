#pragma once

#include <ros/ros.h>
#include <map_merger/MapMerger.h>

ros::Subscriber octo_sub;
ros::Subscriber map_sub;
ros::Publisher occupied_pub;

bool received_octomap = false;
visualization_msgs::MarkerArray last_octomap;
std::map<std::pair<int, int>, int> explored;
std::mutex octomap_lock;
MapMergerParams params_;

void publish_updated(std::map<std::pair<int, int>, int>& explored_to_be_updated);
void map_callback(const nav_msgs::OccupancyGrid::ConstPtr& map);
void octo_callback(const visualization_msgs::MarkerArray::ConstPtr& octomap);


