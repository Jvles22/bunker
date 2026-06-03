cmake_minimum_required(VERSION 3.0.2)
project(bunker_gazebo)

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  rospy
  xacro
)

catkin_package()

install(DIRECTORY launch worlds gazebo_models
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
