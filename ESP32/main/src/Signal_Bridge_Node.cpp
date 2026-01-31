#include "Signal_Bridge_Node.hpp"
#include <esp_log.h>

static const char *TAG = "Signal_Bridge";


Signal_Bridge_Node::Signal_Bridge_Node() {
    _msg_Wrist_IMU.data.capacity = 7; // ax, ay, az, gx, gy, gz, timestamp
    _msg_Wrist_IMU.data.data = (float*)malloc(_msg_Wrist_IMU.data.capacity * sizeof(float));
    _msg_Wrist_IMU.data.size = 0;

    _msg_Arm_IMU.data.capacity = 7; // ax, ay, az, gx, gy, gz, timestamp
    _msg_Arm_IMU.data.data = (float*)malloc(_msg_Arm_IMU.data.capacity * sizeof(float));
    _msg_Arm_IMU.data.size = 0;

    _msg_Magnet.data.capacity = 4; // mx, my, mz, timestamp
    _msg_Magnet.data.data = (float*)malloc(_msg_Magnet.data.capacity * sizeof(float));
    _msg_Magnet.data.size = 0;

    _msg_Flex.data.capacity = 2; // flex, timestamp
    _msg_Flex.data.data = (float*)malloc(_msg_Flex.data.capacity * sizeof(float));
    _msg_Flex.data.size = 0;
}

Signal_Bridge_Node::~Signal_Bridge_Node(){
    // 할당된 메모리 해제 (메모리 누수 방지)
    if (_msg_Wrist_IMU.data.data) free(_msg_Wrist_IMU.data.data);
    if (_msg_Arm_IMU.data.data) free(_msg_Arm_IMU.data.data);
    if (_msg_Magnet.data.data) free(_msg_Magnet.data.data);
    if (_msg_Flex.data.data) free(_msg_Flex.data.data);
}

// Signal_Bridge_Node의 초기화 함수입니다.
// Initialization function of Signal_Bridge_Node.
void Signal_Bridge_Node::init(rcl_support_t* support, IMU_Driver& IMU_W, IMU_Driver& IMU_A, Magnet_Driver& Magnet, Flex_Driver& Flex){
    const char* node_name = "ESP32_Signal_Bridge";

    // 1. 노드 생성
    rclc_node_init_default(&_node, node_name, "", support);

    /* 1. 하드웨어 준비 (begin 호출)
       각 드라이버의 begin()을 실행하여 센서가 잘 연결되었는지 확인합니다.
       Call begin() for each driver to check if sensors are connected. */
    bool is_imu_w_ok = IMU_W.begin();
    bool is_imu_a_ok = IMU_A.begin();
    bool is_mag_ok = Magnet.begin();
    bool is_flex_ok = Flex.begin();

    // 센서 중 하나라도 응답이 없으면 로그를 남깁니다.
    // If any sensor fails to respond, log an error.
    if (!is_imu_w_ok || !is_imu_a_ok || !is_mag_ok || !is_flex_ok) {
        ESP_LOGE(TAG, "센서 초기화 실패. 연결을 확인하세요.")
    }

    /*지자기 데이터를 보낼 퍼블리셔를 기본 설정으로 초기화합니다.
      Initialize the publisher for @ data with default settings.
      rclc_publisher_init_default(
      &_pub_@,                                       // 1. 퍼블리셔 객체의 주소 (리모컨) / Address of the publisher object.
      &_node,                                             // 2. 이 퍼블리셔가 소속될 노드 / The node this publisher belongs to.
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray), // 3. 보낼 데이터의 형식 (설계도) / Message type support.
      "@_Data"                                       // 4. 토픽(통로)의 이름 / Name of the topic.
      ); */

    rclc_publisher_init_default(&_pub_Wrist_IMU, &_node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray), "IMU_Wrist_Data");

    rclc_publisher_init_default(&_pub_Arm_IMU, &_node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray), "IMU_Arm_Data");
    
    rclc_publisher_init_default(&_pub_Magnet, &_node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray), "Magnet_Data");

    rclc_publisher_init_default(&_pub_Flex, &_node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray), "Flex_Data");

    ESP_LOGI(TAG, "3개 토픽 퍼블리셔 초기화 완료");
}

/* 손목에 부착된 IMU 데이터를 업데이트하고 ROS2로 전송하는 함수입니다. 
   Function to update IMU data and send it to ROS2. */
void Signal_Bridge_Node::update_Wrist_IMU(IMU_Driver& IMU_W) {
    // 1. 센서의 최신 값을 갱신합니다. (업데이트 필수!)
    // Update the sensor's latest values.
    IMU_W.update(); // IMU_Driver 내부의 _ax, _gx 등을 최신화함 / Updates internal raw data.

    // 캡슐화된 데이터를 안전하게 가져옴 (인라인 함수 호출)
    // Safely retrieve encapsulated data
    IMU_Driver::IMU_Data data = IMU_W.get_data();

    // 3. ROS 메시지 그릇에 데이터를 담습니다. (int16을 float로 형변환)
    // Put data into the ROS message container. (Cast int16 to float)
    _msg_Wrist_IMU.data.size = 7; // 데이터 개수 고정 / Fixed number of elements.
    _msg_Wrist_IMU.data.data[0] = (float)data.ax;
    _msg_Wrist_IMU.data.data[1] = (float)data.ay;
    _msg_Wrist_IMU.data.data[2] = (float)data.az;
    _msg_Wrist_IMU.data.data[3] = (float)data.gx;
    _msg_Wrist_IMU.data.data[4] = (float)data.gy;
    _msg_Wrist_IMU.data.data[5] = (float)data.gz;
    _msg_Wrist_IMU.data.data[6] = (float)data.timestamp;


    // 4. 데이터를 퍼블리시합니다.
    // Publish the data.
    rcl_publish(&_pub_Wrist_IMU, &_msg_Wrist_IMU, NULL);
}

/* 팔에 부착된 IMU 데이터를 업데이트하고 ROS2로 전송하는 함수입니다. 
   Function to update IMU data and send it to ROS2. */
void Signal_Bridge_Node::update_Arm_IMU(IMU_Driver& IMU_A) {
    // 1. 센서의 최신 값을 갱신합니다. (업데이트 필수!)
    // Update the sensor's latest values.
    IMU_A.update(); // IMU_Driver 내부의 _ax, _gx 등을 최신화함 / Updates internal raw data.

    // 캡슐화된 데이터를 안전하게 가져옴 (인라인 함수 호출)
    // Safely retrieve encapsulated data
    IMU_Driver::IMU_Data data = IMU_A.get_data();

    // 3. ROS 메시지 그릇에 데이터를 담습니다. (int16을 float로 형변환)
    // Put data into the ROS message container. (Cast int16 to float)
    _msg_Arm_IMU.data.size = 7; // 데이터 개수 고정 / Fixed number of elements.
    _msg_Arm_IMU.data.data[0] = (float)data.ax;
    _msg_Arm_IMU.data.data[1] = (float)data.ay;
    _msg_Arm_IMU.data.data[2] = (float)data.az;
    _msg_Arm_IMU.data.data[3] = (float)data.gx;
    _msg_Arm_IMU.data.data[4] = (float)data.gy;
    _msg_Arm_IMU.data.data[5] = (float)data.gz;
    _msg_Arm_IMU.data.data[6] = (float)data.timestamp;


    // 4. 데이터를 퍼블리시합니다.
    // Publish the data.
    rcl_publish(&_pub_Arm_IMU, &_msg_Arm_IMU, NULL);
}


/* 지자기 센서 데이터를 처리하는 함수입니다. 
   Function to handle magnetometer sensor data. */
void Signal_Bridge_Node::update_Magnet(Magnet_Driver& Magnet) {
    // 1. 센서의 최신 값을 갱신합니다. (업데이트 필수!)
    // Update the sensor's latest values.
    Magnet.update(); // Magnet_Driver 내부의 값들을 최신화함 / Updates internal raw data.

    // 캡슐화된 데이터를 안전하게 가져옴 (인라인 함수 호출)
    // Safely retrieve encapsulated data
    Magnet_Driver::Magnet_Data data = Magnet.get_data();

    // 2. 메시지 크기 설정 및 데이터 복사
    // Set message size and copy data.
    _msg_Magnet.data.size = 4; // X, Y, Z 3개 데이터 고정 / Fixed 3 axes.
    _msg_Magnet.data.data[0] = (float)data.mx;
    _msg_Magnet.data.data[1] = (float)data.my;
    _msg_Magnet.data.data[2] = (float)data.mz;
    _msg_Magnet.data.data[3] = (float)data.timestamp;

    // 3. 지자기 데이터 전송
    // Publish magnetic data.
    rcl_publish(&_pub_Magnet, &_msg_Magnet, NULL);
}

/* 굴곡 센서 데이터를 처리하는 함수입니다. 
   Function to handle flex sensor data. */
void Signal_Bridge_Node::update_Flex(Flex_Driver& Flex) {
    // 1. ADC 값을 갱신합니다.
    // Update ADC values.
    Flex.update();

    // 캡슐화된 데이터를 안전하게 가져옴 (인라인 함수 호출)
    // Safely retrieve encapsulated data
    Flex_Driver::Flex_Data data = Flex.get_data();

    // 2. 메시지 크기 설정 및 데이터 복사
    // Set message size and copy data.
    _msg_Flex.data.size = 2;
    _msg_Flex.data.data[0] = (float)data.raw_flex;
    _msg_Flex.data.data[1] = (float)data.timestamp;

    // 4. 굴곡 데이터 전송
    // Publish flex data.
    rcl_publish(&_pub_Flex, &_msg_Flex, NULL);
}