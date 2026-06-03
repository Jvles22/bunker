cmake_minimum_required(VERSION 3.0.2)
project(bunker_slam)

find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/square_ramp_traverse.py
  scripts/autonomous_navigation.py
  scripts/analyze_traverse.py
  scripts/param_tuner.py
  scripts/remove_ground.py
  scripts/speed_benchmark.py
  DESTINATION ${CATKIN_PKG_PREFIX}/scripts
)

install(DIRECTORY launch config rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
