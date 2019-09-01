cmake_minimum_required(VERSION 2.8.3)
project(map_merger)

find_package(catkin REQUIRED COMPONENTS)

include_directories(
        include/
        ${catkin_INCLUDE_DIRS}
        ${roscpp_INCLUDE_DIRS}
)



catkin_package()

add_executable(map_merger_node src/map_merger_node.cpp src/MapMerger.cpp)
target_link_libraries(map_merger_node ${catkin_LIBRARIES})