#include "rclcpp/rclcpp.hpp"

int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("Exercise3");

    RCLCPP_INFO(node->get_logger(), "Exercise 3");
    rclcpp::shutdown();
    return 0;
}