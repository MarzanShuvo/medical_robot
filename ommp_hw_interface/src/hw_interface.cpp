#include <sstream>

#include <ommp_hw_interface/hw_interface.h>
// ROS parameter loading
#include <rosparam_shortcuts/rosparam_shortcuts.h>

namespace ommp_hw_interface {

OmmpHardwareInterface::OmmpHardwareInterface(ros::NodeHandle &nh)
    : nh_(nh), nh_priv_(nh, name_), velocity_controller_running_(true), first_(true) {}

bool OmmpHardwareInterface::init() {
  bool wait;
  // Get Generic Hardware Interface parameters
  std::size_t error = 0;
  error += !rosparam_shortcuts::get(name_, nh_priv_, "joints", joint_names_);
  error += !rosparam_shortcuts::get(name_, nh_priv_, "fake_execution", fake_execution_);
  error += !rosparam_shortcuts::get(name_, nh_priv_, "vel_factor", VEL_FACTOR_);
  error += !rosparam_shortcuts::get(name_, nh_priv_, "wait", wait);
  error += !rosparam_shortcuts::get(name_, nh_priv_, "verbose", verbose_);
  rosparam_shortcuts::shutdownIfError(name_, error);

  // Resize vectors
  num_joints_ = joint_names_.size();

  joint_position_.resize(num_joints_, 0.0);
  joint_velocity_.resize(num_joints_, 0.0);
  joint_effort_.resize(num_joints_, 0.0);

  // joint_position_command_.resize(num_joints_, 0.0);
  joint_velocity_command_.resize(num_joints_, 0.0);

  // Create ros_control interfaces
  for (size_t i = 0; i < num_joints_; ++i) {
    ROS_DEBUG_STREAM("[ommp_hw_interface] Registering handles for joint " << joint_names_[i]);
    try {
      
      // Create joint state handle
      hardware_interface::JointStateHandle joint_state_handle = hardware_interface::JointStateHandle(
          joint_names_[i], &joint_position_[i], &joint_velocity_[i], &joint_effort_[i]);
      // Register handle into joint state interface
      joint_state_interface_.registerHandle(joint_state_handle);

      // Create joint position command handle
      // hardware_interface::JointHandle joint_pos_handle =
      //     hardware_interface::JointHandle(
      //         joint_state_interface_.getHandle(joint_names_[i]),
      //         &joint_position_command_[i]);
      // // Register handle into joint position command interface
      // position_joint_interface_.registerHandle(joint_pos_handle);

      // Create joint position command handle
      hardware_interface::JointHandle joint_vel_handle = hardware_interface::JointHandle(
          joint_state_interface_.getHandle(joint_names_[i]), &joint_velocity_command_[i]);
      // Register handle into joint position command interface
      velocity_joint_interface_.registerHandle(joint_vel_handle);

    } catch (const hardware_interface::HardwareInterfaceException &e) {
      ROS_ERROR_STREAM("[ommp_hw_interface] " << e.what());
      return false;
    }
  }

  // Register interfaces
  registerInterface(&joint_state_interface_);
  // registerInterface(&position_joint_interface_);
  registerInterface(&velocity_joint_interface_);

  // Ros Interface
  wheels_vel_target_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("/wheels_vel_target", 10);
  wheels_vel_feedback_sub_ = nh_.subscribe("/wheels_velocity", 10, &OmmpHardwareInterface::wheels_velocity_CB, this);
  wheels_vel_target_cmd_.data.resize(4);
  wheels_vel_feedback_msg_.data.resize(4);
  // wait for first wheels vel msg from rosserial
  if (!fake_execution_ && wait) {
    auto first_enc_tick =
        ros::topic::waitForMessage<std_msgs::Float64MultiArray>("/wheels_velocity", ros::Duration(40.0));
  }

  ROS_INFO("Hardware Interface Started");
  return true;
}

// encoder callback
void OmmpHardwareInterface::wheels_velocity_CB(const std_msgs::Float64MultiArray::ConstPtr &vel_feedback_msg) {
  {
    std::lock_guard<std::mutex> lock(mutex_input_);
    wheels_vel_feedback_msg_.data[0] = vel_feedback_msg->data[0];
    wheels_vel_feedback_msg_.data[1] = vel_feedback_msg->data[1];
    wheels_vel_feedback_msg_.data[2] = vel_feedback_msg->data[2];
    wheels_vel_feedback_msg_.data[3] = vel_feedback_msg->data[3];
  }
}

void OmmpHardwareInterface::read(const ros::Time &time, const ros::Duration &period) {
  // passthrough command to state a.k.a fake execution
  if (fake_execution_) {
    if (velocity_controller_running_) {
      for (size_t i = 0; i < joint_velocity_command_.size(); i++) {
        joint_velocity_[i] = joint_velocity_command_[i];
        joint_position_[i] += joint_velocity_command_[i] * period.toSec();
      }
    }
    if (verbose_)
      ROS_WARN("Joint Velocity : %f %f %f %f", joint_velocity_[0] / VEL_FACTOR_, joint_velocity_[1] / VEL_FACTOR_,
               joint_velocity_[2] / VEL_FACTOR_, joint_velocity_[3] / VEL_FACTOR_);
    return;
  }

  // real
  // double ENC_TO_VEL_FACTOR = wheel_diameter_ / encoder_ticks_;

  {
    std::lock_guard<std::mutex> lock(mutex_input_);
    for (size_t i = 0; i < 4; i++) {
      joint_velocity_[i] = wheels_vel_feedback_msg_.data[i] / VEL_FACTOR_;
      joint_position_[i] += joint_velocity_[i] * period.toSec();
    }
    if (verbose_)
      ROS_WARN("Joint Velocity : %f %f %f %f", joint_velocity_[0] / VEL_FACTOR_, joint_velocity_[1] / VEL_FACTOR_,
               joint_velocity_[2] / VEL_FACTOR_, joint_velocity_[3] / VEL_FACTOR_);
  }
}

void OmmpHardwareInterface::write(const ros::Time &time, const ros::Duration &period) {
  // passthrough
  if (fake_execution_) {
    if (verbose_)
      ROS_WARN("Joint Velocity Command : %f %f %f %f", joint_velocity_command_[0] * VEL_FACTOR_,
               joint_velocity_command_[1] * VEL_FACTOR_, joint_velocity_command_[2] * VEL_FACTOR_,
               joint_velocity_command_[3] * VEL_FACTOR_);
    return;
  }

  // real
  if (velocity_controller_running_) {
    // transform
    wheels_vel_target_cmd_.data[0] = joint_velocity_command_[0] * VEL_FACTOR_;
    wheels_vel_target_cmd_.data[1] = joint_velocity_command_[1] * VEL_FACTOR_;
    wheels_vel_target_cmd_.data[2] = joint_velocity_command_[0] * VEL_FACTOR_;
    wheels_vel_target_cmd_.data[3] = joint_velocity_command_[1] * VEL_FACTOR_;
    // publish
    wheels_vel_target_pub_.publish(wheels_vel_target_cmd_);
  }
  if (verbose_)
    ROS_WARN("Joint Velocity Command : %f %f %f %f", joint_velocity_command_[0] * VEL_FACTOR_,
             joint_velocity_command_[1] * VEL_FACTOR_, joint_velocity_command_[2] * VEL_FACTOR_,
             joint_velocity_command_[3] * VEL_FACTOR_);
}

} // namespace ommp_hw_interface
