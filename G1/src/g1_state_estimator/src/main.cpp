#include <rclcpp/rclcpp.hpp>
#include "g1_state_estimator/estimator_node.h"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<StateEstimatorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
