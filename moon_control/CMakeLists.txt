cmake_minimum_required(VERSION 2.8.3)
project(moon_control)

find_package(catkin REQUIRED)

catkin_package(
  CATKIN_DEPENDS joint_state_controller diff_drive_controller controller_manager teleop_twist_keyboard
)

install(FILES
  config/gazebo/moon_control.yml
  launch/control.launch
  launch/teleop.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)