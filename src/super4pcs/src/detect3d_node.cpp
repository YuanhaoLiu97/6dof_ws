#include "detect3d_synchronizer/detect3d_synchronizer.h"

#include <memory>
#include <rclcpp/rclcpp.hpp>

int main (int argc, char *argv[]){
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions options;
    auto node = std::make_shared<detect3d_synchronizer::Detect3DSynchronizer>(options);
    RCLCPP_INFO(node->get_logger(), "detect 3d synchronizer start up!");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}