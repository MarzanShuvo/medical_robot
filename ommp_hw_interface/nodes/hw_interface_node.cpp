#include <csignal>
#include <ommp_hw_interface/hw_control_loop.h>
#include <ommp_hw_interface/hw_interface.h>
#include <ros/ros.h>

// hardware interface pointer
boost::shared_ptr<ommp_hw_interface::OmmpHardwareInterface> hw_interface_ptr;
// control loop pointer
boost::shared_ptr<ros_control_boilerplate::HWControlLoop> hw_control_loop_ptr;

// Interrupt signal
void signalHandler(int signum) {

  ROS_WARN_STREAM("[ommp_hw_interface] Interrupt signal (" << signum << ") received.\n");

  hw_interface_ptr.reset();
  hw_control_loop_ptr.reset();

  exit(signum);
}

int main(int argc, char **argv) {
  // Initialize node
  ros::init(argc, argv, "ommp_hardware_interface_node");
  // Async spinner
  // So that callbacks, services dont affect the control thread
  ros::AsyncSpinner spinner(3);
  // start spinner
  spinner.start();
  // node handle
  ros::NodeHandle nh("");

  // register signal SIGINT and signal handler
  // signal(SIGINT, signalHandler);

  // Create the hardware interface
  hw_interface_ptr.reset(new ommp_hw_interface::OmmpHardwareInterface(nh));
  if (!hw_interface_ptr->init()) {
    ROS_ERROR_STREAM("[ommp_hw_interface_node] Could not correctly initialize robot. Exiting");
    exit(1);
  }

  ROS_INFO_STREAM("[ommp_hw_interface_node] HW interface initialized");
  // Pass the base hardware interface into the control loop object
  // Creates the controller manager
  // Calls read() -> update() -> write() with monotonic time
  hw_control_loop_ptr.reset(new ros_control_boilerplate::HWControlLoop(nh, hw_interface_ptr));
  // Start the control loop
  hw_control_loop_ptr->run(); // Blocks until shutdown signal received

  return 0;
}
