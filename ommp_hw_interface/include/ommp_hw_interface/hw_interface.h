#pragma once

#include <boost/scoped_ptr.hpp>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/ros.h>

// Publish Subscribe Includes
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Int64MultiArray.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/MultiArrayLayout.h"



#include <iostream>
#include <map>
#include <string>
#include <chrono>
#include <thread>
#include <mutex>
 

namespace ommp_hw_interface {

/**
 * @brief The OmmpHardwareInterface class handles the interface between the ROS system and the main
 * driver. It contains the read and write methods of the main control loop and registers various ROS
 * topics and services.
 */
class OmmpHardwareInterface : public hardware_interface::RobotHW {
public:
  /**
   * @brief Construct a new Hardware Interface object
   */
  OmmpHardwareInterface(ros::NodeHandle &nh);
  /**
   * @brief Destroy Hardware Interface object
   */
  virtual ~OmmpHardwareInterface() = default;
  /**
   * @brief Handles the setup functionality for the ROS interface. This includes parsing ROS
   * parameters, creating interfaces, starting the main driver and advertising ROS services.
   *
   * @param nh Root level ROS node handle
   * @param nh_local ROS node handle for the robot namespace
   * @returns True, if the setup was performed successfully
   *
   */
  virtual bool init();
  /**
   * @brief Read method of the control loop. Reads a messages from the robot and handles and
   * publishes the ros information as needed.
   *
   * @param time Current time
   * @param period Duration of current control loop iteration
   */
  virtual void read(const ros::Time &time, const ros::Duration &period) override;
  /**
   * @brief Write method of the control loop. Writes target joint positions to the robot to be read
   * by its driver interface.
   *
   * @param time Current time
   * @param period Duration of current control loop iteration
   */
  virtual void write(const ros::Time &time, const ros::Duration &period) override;

protected:
  // Subscriber wheel encoder Callback
  void wheels_velocity_CB(const std_msgs::Float64MultiArray::ConstPtr &vel_feedback_msg);

  // Name of this class
  std::string name_ = "ommp_hardware_interface";
  ros::NodeHandle nh_, nh_priv_;
  // Configuration
  bool velocity_controller_running_;
  // Hardware Interfaces
  hardware_interface::JointStateInterface joint_state_interface_;
  // hardware_interface::PositionJointInterface position_joint_interface_;
  hardware_interface::VelocityJointInterface velocity_joint_interface_;
  // States
  std::vector<double> joint_position_, joint_velocity_, joint_effort_;
  // Commands
  // std::vector<double> joint_position_command_;
  std::vector<double> joint_velocity_command_;
  // private
  size_t num_joints_;
  std::vector<std::string> joint_names_;
  bool first_, verbose_;

  // parameters
  bool fake_execution_;
  // Tranform Factors From encoder to joint_state Universe
  double VEL_FACTOR_; 

  // ros interface
  ros::Publisher wheels_vel_target_pub_;
  ros::Subscriber wheels_vel_feedback_sub_; // * different thread from main control loop but it's ok
  std_msgs::Float32MultiArray wheels_vel_target_cmd_;

  // Velocity from arduino
  std_msgs::Float64MultiArray wheels_vel_feedback_msg_;
  // mutex
  mutable std::mutex mutex_input_;

};

} // namespace ommp_hw_interface