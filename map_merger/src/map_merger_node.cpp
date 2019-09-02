#include <map_merger/map_merger_node.h>
#include <map_merger/MapMerger.h>

void octo_callback(const visualization_msgs::MarkerArray::ConstPtr& octomap) {
	for (auto i : octomap->markers) {
		for (auto j: i.points) {
			//std::cout << j << std::endl;
		}
	}
}

void map_callback(const nav_msgs::OccupancyGrid::ConstPtr& map) {
	for(int i = 0; i < 1000; i++) {
		for (int j = 0 ; j < 1000 ; j++) {
			int dat = map->data[i *1000 + j];
			if (dat >= 0) {
				//std::cout << double(i)*map_res << ", " << double(j)*map_res << ": " << dat << std::endl;
			}
		}
	}
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "map_merger");
	ros::NodeHandle nh; 
	MapMerger mm;
	octo_sub = nh.subscribe("/husky/map", 1, map_callback);
	map_sub = nh.subscribe("/baroness/nbvPlanner/octomap_occupied", 1, octo_callback);
	ros::spin();
}