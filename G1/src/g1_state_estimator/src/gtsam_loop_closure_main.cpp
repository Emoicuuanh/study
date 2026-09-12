#include <rclcpp/rclcpp.hpp>
#include "g1_state_estimator/gtsam_loop_closure_node.h"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GtsamLoopClosureNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
