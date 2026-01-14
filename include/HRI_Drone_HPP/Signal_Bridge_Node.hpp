#ifndef SIGNAL_BRIDGE_NODE_HPP
#define SIGNAL_BRIDGE_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include <cmath>
#include <vector>

class Signal_Bridge: public rclcpp::Node{
public:
    Signal_Bridge();
    
private:
    void raw_data_pub();
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr raw_pub;
    rclcpp::TimerBase::SharedPtr raw_timer;
    std::vector<float> raw_data;
};

#endif