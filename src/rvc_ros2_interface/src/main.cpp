#include <rvc_camera.h>
#include "rvc_camera.cpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor executor;
    RvcCamera rvc_camera;
    executor.add_node(rvc_camera.node);
    executor.spin();
    return 0;
}