cmake_minimum_required(VERSION 3.0.2)
project(bunker_traversability)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  costmap_2d
  pluginlib
  grid_map_ros
  grid_map_msgs
  pcl_ros
  pcl_conversions
  sensor_msgs
  tf2_ros
  tf2_sensor_msgs
)

find_package(PCL REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES bunker_traversability
  CATKIN_DEPENDS roscpp costmap_2d pluginlib grid_map_ros grid_map_msgs
                 pcl_ros sensor_msgs tf2_ros tf2_sensor_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_library(bunker_traversability
  src/traversability_layer.cpp
)
target_link_libraries(bunker_traversability
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_executable(bunker_elevation_map_node
  src/elevation_map_node.cpp
)
target_link_libraries(bunker_elevation_map_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_executable(adaptive_obstacle_filter_node
  src/adaptive_obstacle_filter_node.cpp
)
target_link_libraries(adaptive_obstacle_filter_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)
