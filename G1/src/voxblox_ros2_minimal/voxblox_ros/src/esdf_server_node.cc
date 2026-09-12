#include "voxblox_ros/esdf_server.h"

#include <gflags/gflags.h>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv) {
  // ros::init(argc, argv, "voxblox");
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
  auto nh = rclcpp::Node::make_shared("voxblox", options);

  voxblox::EsdfServer node(nh.get());

  rclcpp::spin(nh);
  rclcpp::shutdown();
  return 0;
}
