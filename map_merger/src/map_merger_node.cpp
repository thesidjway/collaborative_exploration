#include <map_merger/map_merger_node.h>

void octo_callback(const visualization_msgs::MarkerArray::ConstPtr& octomap) {
    octomap_lock.lock();
    received_octomap = true;
    last_octomap = *octomap;
    for (int i = 0 ; i < last_octomap.markers.size() ; i++) {
        int num_deletions = 0;
        const int vec_size = last_octomap.markers[i].points.size();
        for (int j = 0 ; j < vec_size ; j++) {
            if (last_octomap.markers[i].points[j - num_deletions].z < 0.3) {
                std::pair<int, int> new_cell = std::make_pair<int, int> (int(last_octomap.markers[i].points[j - num_deletions].x / params_.octo_res  - params_.x_orig / params_.octo_res), int(last_octomap.markers[i].points[j - num_deletions].y / params_.octo_res - (params_.y_orig + 0.2) / params_.octo_res ));
                explored[new_cell] = 0;
                last_octomap.markers[i].points.erase(last_octomap.markers[i].points.begin() + j - num_deletions);
                last_octomap.markers[i].colors.erase(last_octomap.markers[i].colors.begin() + j - num_deletions);
                num_deletions++;
            }
        }
    }
    octomap_lock.unlock();
}

void map_callback(const nav_msgs::OccupancyGrid::ConstPtr& map) {
    octomap_lock.lock();
    for (int i = 0; i < params_.width_map; i++) {
        for (int j = 0 ; j < params_.height_map ; j++) {
            int dat = map->data[i + j * params_.width_map];
            if (dat >= 0) {
                std::pair<int, int> new_cell = std::make_pair<int, int> (int(i/8), int(j/8));
                if (dat == 0) {
                    explored[new_cell] = 0;
                } else if (dat == 100) {
                    explored[new_cell] = 100;
                }
            }
        }
    }
    octomap_lock.unlock();
    if (received_octomap) {
        publish_updated(explored);
    }
}

void publish_updated(std::map<std::pair<int, int>, int>&  explored) {
    octomap_lock.lock();
    std_msgs::ColorRGBA clr;
    clr.r = 1;
    clr.g = 1;
    clr.b = 1;
    clr.a = 1;
    std_msgs::ColorRGBA clr2;
    clr2.r = 0;
    clr2.g = 0;
    clr2.b = 0;
    clr2.a = 1;
    for (auto i : explored) {
        if (i.second == 0) {
            geometry_msgs::Point pt;
            pt.x = i.first.first * params_.octo_res + params_.x_orig;
            pt.y = i.first.second * params_.octo_res + params_.y_orig + 0.2;
            pt.z = 0.2;
            last_octomap.markers[last_octomap.markers.size() - 1].points.push_back(pt);
            last_octomap.markers[last_octomap.markers.size() - 1].colors.push_back(clr);
        }
        if (i.second == 100) {
            geometry_msgs::Point pt1, pt2;
            pt1.x = i.first.first * params_.octo_res + params_.x_orig;
            pt1.y = i.first.second * params_.octo_res + params_.y_orig + 0.2;
            pt1.z = 0.2;
            pt2.x = i.first.first * params_.octo_res + params_.x_orig;
            pt2.y = i.first.second * params_.octo_res + params_.y_orig + 0.2;
            pt2.z = 0.6;
            last_octomap.markers[last_octomap.markers.size() - 1].points.push_back(pt1);
            last_octomap.markers[last_octomap.markers.size() - 1].points.push_back(pt2);
            last_octomap.markers[last_octomap.markers.size() - 1].colors.push_back(clr);
            last_octomap.markers[last_octomap.markers.size() - 1].colors.push_back(clr2);
        }
    }
    occupied_pub.publish(last_octomap);
    costmap_2d::Costmap2D costmap;
    MapMerger::generateCostmap(last_octomap, costmap); 
    received_octomap = false;
    octomap_lock.unlock();
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "map_merger");
    ros::NodeHandle nh;
    MapMerger mm;
    octo_sub = nh.subscribe("/husky/map", 1, map_callback);
    map_sub = nh.subscribe("/baroness/nbvPlanner/octomap_occupied", 1, octo_callback);
    occupied_pub = nh.advertise<visualization_msgs::MarkerArray>("/baroness/nbvPlanner/octomap_occupied_merged", 1000);
    ros::spin();
}
