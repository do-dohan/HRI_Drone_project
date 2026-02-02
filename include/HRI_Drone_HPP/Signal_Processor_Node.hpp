#ifndef SIGNAL_PROCESSOR_NODE_HPP
#define SIGNAL_PROCESSOR_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/quaternion.hpp"

// 사용자 정의 라이브러리 헤더 포함
// Include user-defined library headers
#include "HRI_Drone_HPP/Madgwick_Filter.hpp"
#include "HRI_Drone_HPP/EMA_Filter.hpp"
#include "HRI_Drone_HPP/Mapping_Utils.hpp"
#include "HRI_Drone_HPP/Quaternion_to_Euler.hpp"

#include <chrono>
#include <vector>
#include <memory>

class Signal_Processor : public rclcpp::Node{
public:
    // 생성자 선언
    // Constructor declaration
    Signal_Processor();

private:
    // --- 주기적 실행 함수 (Timer Callbacks) ---
    // 센서 데이터를 융합하여 각도를 계산하고 퍼블리시하는 함수
    // Function to fuse sensor data, calculate angles, and publish
    void _Filter_Angle();

    // 굴곡 센서 데이터를 처리하고 퍼블리시하는 함수
    // Function to process and publish flex sensor data
    void _Filter_Flex();

    // --- 토픽 구독 콜백 함수 (Subscriber Callbacks) ---
    void _callback_Wrist_IMU(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
    void _callback_ARM_IMU(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
    void _callback_Magnet(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
    void _callback_Flex(const std_msgs::msg::Float32MultiArray::SharedPtr msg);

    // --- 데이터 저장 변수 (Member Variables for Data) ---
    // [0~2]: Accel, [3~5]: Gyro, [6]: Timestamp
    float _raw_Wrist_data[7]; 
    float _raw_ARM_data[7];
    float _raw_Magnet_data[4]; // [0~2]: Mag, [3]: Timestamp
    float _raw_Flex_data[2];   // [0]: Flex, [1]: Timestamp

    // --- 알고리즘 객체 (Algorithm Objects) ---
    // 1. 매드윅 필터: 손목(9축)과 팔(6축)을 위해 2개 생성
    // 1. Madgwick Filter: Create two for Wrist (9-axis) and Arm (6-axis)
    MadgwickFilter _Madgwick_ARM_Filter;
    MadgwickFilter _Madgwick_Wrist_Filter;

    // 2. EMA 필터: 노이즈 제거용 (각 축별로 필요하므로 벡터로 관리)
    // _EMA_Wrist[0].filter(input); -> 가능 (객체이므로 기능 수행)
    // 2. EMA Filter: For noise reduction (Managed as vectors for each axis)
    std::vector<EMA_Filter>  _EMA_Wrist;
    std::vector<EMA_Filter> _EMA_ARM; 
    std::vector<EMA_Filter> _EMA_Magnet;

    // Flex 필터는 2단계 (입력 노이즈 제거 -> 출력 부드럽게)
    EMA_Filter _EMA_Flex_in;
    EMA_Filter _EMA_Flex_out;

    // 매핑 객체
    Quaternion_to_Euler _to_euler;
    IMU_Mapping _Map_Roll;
    IMU_Mapping _Map_Pitch;
    IMU_Mapping _Map_Yaw;
    Flex_Mapping _Map_Flex;

    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr _pub_Angle;
    rclcpp::TimerBase::SharedPtr _timer_Angle;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr _pub_Flex;
    rclcpp::TimerBase::SharedPtr _timer_Flex;

    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr _sub_Wrist_IMU;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr _sub_ARM_IMU;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr _sub_Magnet;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr _sub_Flex;
};
#endif