#ifndef SIGNAL_PROCESSOR_NODE_HPP
#define SIGNAL_PROCESSOR_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "HRI_Drone_HPP/Madgwick_Filter.hpp"
#include <chrono>
#include <vector>
#include <memory>

class Signal_Processor : public rclcpp::Node{
public:
    Signal_Processor();

private:
    void Quaternion_data_pub();
    void Flex_data_pub();
    void raw_imu_data_sub(const sensor_msgs::msg::Imu::SharedPtr msg);
    void raw_magnet_data_sub(const sensor_msgs::msg::MagneticField::SharedPtr msg);
    void raw_flex_data_sub(const std_msgs::msg::Float32::SharedPtr msg);

    sensor_msgs::msg::Imu IMU_data;
    sensor_msgs::msg::MagneticField Magnet_data;
    geometry_msgs::msg::Quaternion Quaternion_data;
    float Flex_data;

MadgwickFilter madgwick_filter_;

    rclcpp::Publisher<geometry_msgs::msg::Quaternion>::SharedPtr Quaternion_pub;
    rclcpp::TimerBase::SharedPtr Quaternion_timer;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr Flex_pub;
    rclcpp::TimerBase::SharedPtr Flex_timer;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr raw_imu_sub;
    rclcpp::Subscription<sensor_msgs::msg::MagneticField>::SharedPtr raw_magnet_sub;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr raw_flex_sub;
};
#endif