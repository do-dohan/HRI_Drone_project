#ifndef SIGNAL_BRIDGE_NODE_HPP
#define SIGNAL_BRIDGE_NODE_HPP

// 1. 무거운 rclcpp 대신 가벼운 C 라이브러리 사용
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

// 2. 메시지 타입 헤더 (std_msgs 또는 sensor_msgs)
#include <std_msgs/msg/float32_multi_array.h>
// (나중에 사용자님이 정의한 IMU 메시지로 바꿔야 함)

#include "IMU_Driver.hpp"
#include "Magnet_Driver.hpp"
#include "Flex_Driver.hpp"


// 3. 클래스 대신 구조체(Struct)로 관리하는 게 임베디드 정석
class Signal_Bridge_Node {
public:
    Signal_Bridge_Node();
    ~Signal_Bridge_Node();

    // 초기화 및 실행 함수
    // micro-ROS 시스템과 연결을 설정
    void init(rcl_support_t* support, IMU_Driver& IMU_W, IMU_Driver& IMU_A, Magnet_Driver& Magnet, Flex_Driver& Flex);
    void update_Wrist_IMU(IMU_Driver& IMU_W); // 여기서 센서 값을 쏘게 됨
    void update_Arm_IMU(IMU_Driver& IMU_A); 
    void update_Magnet(Magnet_Driver& Magnet);
    void update_Flex(Flex_Driver& Flex);

private:
    // micro-ROS 전용 노드와 퍼블리셔
    rcl_node_t _node;
    rcl_publisher_t _pub_Wrist_IMU;
    rcl_publisher_t _pub_Arm_IMU;
    rcl_publisher_t _pub_Magnet;
    rcl_publisher_t _pub_Flex;
    
    // 메시지 담을 그릇
    std_msgs__msg__Float32MultiArray _msg_Wrist_IMU;
    std_msgs__msg__Float32MultiArray _msg_Arm_IMU;
    std_msgs__msg__Float32MultiArray _msg_Magnet;
    std_msgs__msg__Float32MultiArray _msg_Flex;

    const char *TAG = "Signal_Bridge";
};

#endif // SIGNAL_BRIDGE_NODE_HPP