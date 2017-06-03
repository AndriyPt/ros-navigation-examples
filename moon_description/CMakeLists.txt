cmake_minimum_required(VERSION 2.8.3)
project(moon_description)

find_package(catkin REQUIRED)

catkin_package(
  CATKIN_DEPENDS urdf_tutorial
)

install(FILES
  launch/display.launch
  urdf/common.urdf.xacro
  urdf/wheels.urdf.xacro
  urdf/materials.urdf.xacro
  meshes/moon_base.stl
  robot/moon.gazebo.xacro
  robot/robot.urdf.xacro
  robot/moon.rviz.xacro
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
