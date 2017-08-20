#ifndef MOON_HARDWARE_ROBOT_HARDWARE_H
#define MOON_HARDWARE_ROBOT_HARDWARE_H

#include <mutex>
#include <ros/ros.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include "moon_msgs/MotorsJointState.h"
#include "moon_msgs/SetMotorsVelocity.h"

namespace moon_hardware
{

class MoonRobotHW : public hardware_interface::RobotHW
{
public:
  MoonRobotHW();
  virtual ~MoonRobotHW();
  virtual bool init(ros::NodeHandle& root_nh);
  void read(const ros::Time& time, const ros::Duration& period);
  void write(const ros::Time& time, const ros::Duration& period);

private:
  void jointStateCallback(const moon_msgs::MotorsJointState::ConstPtr& message);

  hardware_interface::JointStateInterface joint_state_interface;
  hardware_interface::VelocityJointInterface joint_velocity_interface;

  ros::Subscriber motors_joint_state;
  ros::Publisher publish_motor_velocities;

  std::mutex joint_states_update_mutex;

  static const int JOINTS_COUNT = 2;

  double command[JOINTS_COUNT];
  double position[JOINTS_COUNT];
  double velocity[JOINTS_COUNT];
  double effort[JOINTS_COUNT];

  double hardware_motor_position[JOINTS_COUNT];
  double hardware_motor_velocity[JOINTS_COUNT];
  double hardware_motor_effort[JOINTS_COUNT];
};

}  // moon_hardware

#endif  // MOON_HARDWARE_ROBOT_HARDWARE_H