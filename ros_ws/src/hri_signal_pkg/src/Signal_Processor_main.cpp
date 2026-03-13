#include "rclcpp/rclcpp.hpp"
#include "hri_signal_pkg/HRI_Drone_HPP/Signal_Processor_Node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Signal_Processor>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}