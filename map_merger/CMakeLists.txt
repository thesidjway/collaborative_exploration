cmake_minimum_required(VERSION 2.8.3)
project(map_merger)
set (CMAKE_CXX_STANDARD 11)

find_package(catkin REQUIRED COMPONENTS roscpp nav_msgs geometry_msgs visualization_msgs costmap_2d)

include_directories(
        include/
        ${catkin_INCLUDE_DIRS}
        ${roscpp_INCLUDE_DIRS}
)



catkin_package()

add_executable(map_merger_node src/map_merger_node.cpp src/MapMerger.cpp)
target_link_libraries(map_merger_node ${catkin_LIBRARIES})