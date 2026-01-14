#ifndef SIGNAL_PROCESSOR_NODE_HPP
#define SIGNAL_PROCESSOR_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include <chrono>
#include <vector>
#include <memory>

class Signal_Processor : public rclcpp::Node{
public:
    Signal_Processor();
private:
    void Quatarian_data_pub();
    void Flex_data_pub();
    void raw_data_sub(const std_msgs::msg::Float32MultiArray::SharedPtr msg);

    std::vector<float> IMU_data;
    std::vector<float> Magnet_data;
    std::vector<float> Quatarian_data;
    std::vector<float> Flex_data;

    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr Quatarian_pub;
    rclcpp::TimerBase::SharedPtr Quatarian_timer;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr Flex_pub;
    rclcpp::TimerBase::SharedPtr Flex_timer;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr raw_sub;
};
#endif