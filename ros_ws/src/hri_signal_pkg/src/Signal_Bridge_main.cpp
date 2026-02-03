#include "hri_signal_pkg/Signal_Bridge_Node.hpp"

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Signal_Bridge>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}