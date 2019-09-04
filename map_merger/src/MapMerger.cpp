#include <map_merger/MapMerger.h>

MapMerger::MapMerger() {

}

MapMerger::~MapMerger() {

}

MapMergerParams MapMerger::params_;

void MapMerger::generateCostmap(const visualization_msgs::MarkerArray& octomap,
                                costmap_2d::Costmap2D& costmap) {
    costmap = costmap_2d::Costmap2D(params_.width, params_.height, params_.octo_res, params_.x_orig, params_.y_orig, -1);
    //TODO: Is this function even necessary?
}
