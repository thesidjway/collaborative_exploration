#pragma once

#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/MarkerArray.h>
#include <costmap_2d/costmap_2d.h>
#include <std_msgs/ColorRGBA.h>
#include <vector>
#include <utility>
#include <mutex>

struct MapMergerParams {
    MapMergerParams() {
    map_res = 0.05;
    octo_res = 0.4;
    width = 125;
    width_map = 1000;
    height = 80;
    height_map = 640;
    x_orig = -25;
    y_orig = -16;
    }
    double map_res = 0.05;
    double octo_res = 0.4;
    int width = 125;
    int width_map = 1000;
    int height = 80;
    int height_map = 640;
    double x_orig = -25;
    double y_orig = -16;
}; 

class MapMerger {
public:
	MapMerger();
	~MapMerger();
    static void generateCostmap(const visualization_msgs::MarkerArray& octomap, 
                         costmap_2d::Costmap2D& costmap);
    static MapMergerParams params_;
};

