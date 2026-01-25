#include "HRI_Drone_HPP/Signal_Processor_Node.hpp"
#include "HRI_Drone_HPP/Madgwick_Filter.hpp"

using namespace std::chrono_literals;

Signal_Processor::Signal_Processor() : Node("Signal_Processor_Node"){
    IMU_data = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    Magnet_data = {0.0, 0.0, 0.0};
    Quatarian_data = {0.0, 0.0, 0.0, 0.0};
    Quatarian_pub = this->create_publisher<std_msgs::msg::Float32MultiArray>("Qurtarian_data",1);
    Quatarian_timer = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&Signal_Processor::Quatarian_data_pub, this));

    Flex_data = {0.0};
    Flex_pub = this->create_publisher<std_msgs::msg::Float32MultiArray>("Flex_data",1);
    Flex_timer = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&Signal_Processor::Flex_data_pub,this));

    raw_sub = this->create_subscription<std_msgs::msg::Float32MultiArray>(
        "raw_data",
        1,
        std::bind(&Signal_Processor::raw_data_sub, this, std::placeholders::_1)
    );
}


void Signal_Processor::Quatarian_data_pub(){
    auto msg = std_msgs::msg::Float32MultiArray();
    msg.data = Quatarian_data;
    RCLCPP_INFO(this->get_logger(), "Publishing_Quatarian_data...");
    Quatarian_pub->publish(msg);
}   

void Signal_Processor::Flex_data_pub(){
    auto msg = std_msgs::msg::Float32MultiArray();
    msg.data = Flex_data;
    RCLCPP_INFO(this->get_logger(), "Publishing_Flex_data...");
    Flex_pub->publish(msg);
}

void Signal_Processor::raw_data_sub(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
    if (!msg->data.empty()){
        RCLCPP_INFO(this->get_logger(), "raw_data 메시지: '%f'", msg->data[0]);
    }
}