cmake_minimum_required(VERSION 2.8.3)
project(moon_launch)

find_package(catkin REQUIRED)

catkin_package(
  CATKIN_DEPENDS moon_gazebo moon_control moon_2dnav
)

install(FILES
  launch/simulation.launch
  launch/teleop.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
