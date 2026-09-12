#include <rclcpp/rclcpp.hpp>
#include "voxblox_ros/tsdf_server.h"

#include <gflags/gflags.h>

int main(int argc, char** argv) {
  // Initialize rclcpp first so it can correctly parse and consume ROS-specific arguments (like --ros-args)
  // before gflags modifies the argv ordering.
  rclcpp::init(argc, argv);

  // Let gflags re-parse later if needed (optional)
  gflags::AllowCommandLineReparsing();

  // Init logging first (so FLAGS_* affect glog)
  google::InitGoogleLogging(argv[0]);

  // Parse only non-help flags and REMOVE recognized ones from argv
  gflags::ParseCommandLineNonHelpFlags(&argc, &argv, /*remove_flags=*/true);

  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  rclcpp::Node::SharedPtr node_ptr = rclcpp::Node::make_shared("voxblox_node", options);
  voxblox::TsdfServer node(node_ptr.get());

  rclcpp::spin(node_ptr);
  rclcpp::shutdown();
  return 0;
}
