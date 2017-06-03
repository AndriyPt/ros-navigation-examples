cmake_minimum_required(VERSION 2.8.3)
project(moon_gazebo)

find_package(catkin REQUIRED)

catkin_package(
  CATKIN_DEPENDS gazebo_ros gazebo_ros_control
)

install(FILES
  launch/gazebo.launch
  worlds/moon.world
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
