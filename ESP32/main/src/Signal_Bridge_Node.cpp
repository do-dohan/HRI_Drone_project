#include "Signal_Bridge_Node.hpp"
#include <esp_log.h>

static const char *TAG = "Signal_Bridge";


Signal_Bridge_Node::Signal_Bridge_Node() {
    _msg_IMU.data.capacity = 6;
    _msg_IMU.data.data = (float*)malloc(_msg_IMU.data.capacity * sizeof(float));
    _msg_IMU.data.size = 0;

    _msg_Magnet.data.capacity = 3;
    _msg_Magnet.data.data = (float*)malloc(_msg_IMU.data.capacity * sizeof(float));
    _msg_Magnet.data.size = 0;

    _msg_Flex.data = 0.0f;
}

Signal_Bridge_Node::~Signal_Bridge_Node(){
    if (_msg_IMU.data.data) free(_msg_IMU.data.data);
    if (_msg_Magnet.data.data) free(_msg_Magnet.data.data);
}

void Signal_Bridge_Node::init(rcl_support_t* support(){
    const char* node_name = "ESP32_Signal_Bridge";
    rclc_node_init_default(&node, node_name, "", support);
    
    rclc_publisher_init_default(&_pub_IMU, &_node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray), "IMU_Data");
    
    rclc_publisher_init_default(&_pub_Magnet, &_node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray), "Magnet_Data");

    rclc_publisher_init_default(&_pub_Flex, &_node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "Flex_Data");

    ESP_LOGI(_TAG, "3개 토픽 퍼블리셔 초기화 완료");

void Signal_Bridge_Node::update_IMU(IMU_Driver& IMU)