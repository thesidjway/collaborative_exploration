#include <map_merger/map_merger_node.h>
#include <map_merger/MapMerger.h>

void octo_callback(const visualization_msgs::MarkerArray::ConstPtr& octomap) {
	octomap_lock.lock();
	received_octomap = true;
	last_octomap = *octomap;
	for(int i = 0 ; i < last_octomap.markers.size() ; i++) {
		int num_deletions = 0;
		const int vec_size = last_octomap.markers[i].points.size();
		for (int j = 0 ; j < vec_size ; j++) {
			if (last_octomap.markers[i].points[j - num_deletions].z < 0.3 ) {
				last_octomap.markers[i].points.erase(last_octomap.markers[i].points.begin() + j - num_deletions);
				last_octomap.markers[i].colors.erase(last_octomap.markers[i].colors.begin() + j - num_deletions);
				num_deletions++;
			}
		}
	}
	octomap_lock.unlock();
}

void map_callback(const nav_msgs::OccupancyGrid::ConstPtr& map) {
	for(int i = 0; i < width_map; i++) {
		for (int j = 0 ; j < height_map ; j++) {
			int dat = map->data[i + j * width_map];
			if (dat >= 0) {
				grid_2d[int(i/8)][int(j/8)] = dat;
				std::pair<int, int> new_cell = std::make_pair<int, int> (int(i/8), int(j/8));
				if (dat == 0) {
					explored[new_cell] = 0;
				} else if (dat == 100) {
					explored[new_cell] = 100;
				}
			}
		}
	}
	if (received_octomap) {
		publish_updated(explored);
	}
}

void publish_updated(std::map<std::pair<int, int>, int>&  explored) {
	octomap_lock.lock();
	std::cout << "Size explored: " << explored.size() << std::endl;
	std_msgs::ColorRGBA clr;
	clr.r = 1;
	clr.g = 1;
	clr.b = 1;
	clr.a = 1;
	for (auto i : explored) {
		if (i.second == 0) {
			geometry_msgs::Point pt;
			pt.x = i.first.first * octo_res + x_orig;
			pt.y = i.first.second * octo_res + y_orig + 0.2;
			pt.z = 0.2;
			last_octomap.markers[last_octomap.markers.size() - 1].points.push_back(pt);
			last_octomap.markers[last_octomap.markers.size() - 1].colors.push_back(clr);
		}
		if (i.second == 100) {
			geometry_msgs::Point pt1, pt2;
			pt1.x = i.first.first * octo_res + x_orig;
			pt1.y = i.first.second * octo_res + y_orig + 0.2;
			pt1.z = 0.2;
			pt2.x = i.first.first * octo_res + x_orig;
			pt2.y = i.first.second * octo_res + y_orig + 0.2;
			pt2.z = 0.6;
			last_octomap.markers[last_octomap.markers.size() - 1].points.push_back(pt1);
			last_octomap.markers[last_octomap.markers.size() - 1].points.push_back(pt2);
			last_octomap.markers[last_octomap.markers.size() - 1].colors.push_back(clr);
			last_octomap.markers[last_octomap.markers.size() - 1].colors.push_back(clr);
		}
	}
	occupied_pub.publish(last_octomap);
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