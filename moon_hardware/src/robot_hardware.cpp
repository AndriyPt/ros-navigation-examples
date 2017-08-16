#include "moon_hardware/robot_hardware.h"
#include <std_msgs/Float32.h>
#include <hardware_interface/robot_hw.h>
#include <string>
#include <vector>

namespace moon_hardware
{

  MoonRobotHW::MoonRobotHW()
  {
  }

  MoonRobotHW::~MoonRobotHW()
  {
  }

//   void MoonRobotHW::jointStateCallback(const first_msgs::JointState::ConstPtr& message)
//   {
//     hardware_motor_position = message->position;
//     hardware_motor_velocity = message->velocity;
//     hardware_motor_effort = message->effort;
//   }

  bool MoonRobotHW::init(ros::NodeHandle& root_nh)
  {
//    publish_effort = root_nh.advertise<std_msgs::Float32>("hardware_set_motor_effort", 1000);
//    motor_joint_state = root_nh.subscribe("hardware_motor_state", 1000, &MoonRobotHW::jointStateCallback, this);

    hardware_interface::JointStateHandle left_wheel_joint_state_handle("back_left_wheel_joint", &position[0],
        &velocity[0], &effort[0]);
    joint_state_interface.registerHandle(left_wheel_joint_state_handle);

    hardware_interface::JointStateHandle right_wheel_joint_state_handle("back_right_wheel_joint", &position[1],
        &velocity[1], &effort[1]);
    joint_state_interface.registerHandle(right_wheel_joint_state_handle);

    registerInterface(&joint_state_interface);

    hardware_interface::JointHandle left_wheel_joint_velocity_handler(
        joint_state_interface.getHandle("back_left_wheel_joint"), &command[0]);
    joint_velocity_interface.registerHandle(left_wheel_joint_velocity_handler);
    hardware_interface::JointHandle right_wheel_joint_velocity_handler(
        joint_state_interface.getHandle("back_right_wheel_joint"), &command[1]);
    joint_velocity_interface.registerHandle(right_wheel_joint_velocity_handler);

    registerInterface(&joint_velocity_interface);
  }

  void MoonRobotHW::read(const ros::Time& time, const ros::Duration& period)
  {
//    position = hardware_motor_position;
//    velocity = hardware_motor_velocity;
//    effort = hardware_motor_effort;
  }

  void MoonRobotHW::write(const ros::Time& time, const ros::Duration& period)
  {
//    std_msgs::Float32 message;
//    message.data = command;
//    publish_effort.publish(message);
  }

}  // first_hardware