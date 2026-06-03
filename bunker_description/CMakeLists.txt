cmake_minimum_required(VERSION 3.0.2)
project(bunker_description)

find_package(catkin REQUIRED COMPONENTS
  robot_state_publisher
  rospy
  xacro
)

catkin_package()

install(DIRECTORY launch urdf meshes
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
