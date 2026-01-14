#include "HRI_Drone_HPP/Signal_Processor_Node.hpp"

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    auto signal_processor = std::make_shared<Signal_Processor>();
    rclcpp::spin(signal_processor);
    rclcpp::shutdown();
    return 0;
}