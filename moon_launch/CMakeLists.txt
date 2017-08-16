cmake_minimum_required(VERSION 2.8.3)
project(moon_launch)

find_package(catkin REQUIRED)

catkin_package(
  CATKIN_DEPENDS moon_gazebo moon_control moon_2dnav moon_hardware
)

install(FILES
  launch/gmapping.launch
  launch/navigation.launch
  launch/simulation.launch
  launch/teleop.launch
  launch/real_robot.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
