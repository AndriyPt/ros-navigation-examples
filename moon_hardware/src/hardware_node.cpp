#include "moon_hardware/robot_hardware.h"
#include <controller_manager/controller_manager.h>
#include <hardware_interface/actuator_state_interface.h>
#include <ros/callback_queue.h>

using moon_hardware::MoonRobotHW;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "moon_hardware_node");
  ros::NodeHandle node_handle;
  ros::CallbackQueue queue;
  node_handle.setCallbackQueue(&queue);

  MoonRobotHW robot;
  robot.init(node_handle);
  controller_manager::ControllerManager controller_manager(&robot, node_handle);

  ros::AsyncSpinner spinner(4, &queue);
  spinner.start();

  ros::Time ts = ros::Time::now();

  ros::Rate rate(10);
  while (ros::ok())
  {
     ros::Duration d = ros::Time::now() - ts;
     ts = ros::Time::now();
     robot.read(ts, d);
     controller_manager.update(ts, d);
     robot.write(ts, d);
     rate.sleep();
  }

  spinner.stop();

  return 0;
}