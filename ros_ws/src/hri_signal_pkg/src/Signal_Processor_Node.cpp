#include "hri_signal_pkg/Signal_Processor_Node.hpp"

using namespace std::chrono_literals;

Signal_Processor::Signal_Processor() 
    : Node("Signal_Processor_Node"),
    // 1. 매드윅 필터 초기화 (샘플링 50Hz, 게인 0.1)
    _Madgwick_Wrist_Filter(50.0f, 0.1f),
    _Madgwick_ARM_Filter(50.0f, 0.1f),
    // 2. Flex 필터 초기화
    _EMA_Flex_in(0.2f), // 입력 노이즈 제거용
    _EMA_Flex_out(0.1f) // 출력 부드럽게 만들기용
    {
    // 3. EMA 필터 벡터 초기화 (각 축마다 필터 객체 생성)
    // 손목 & 팔 IMU (6축: Acc 3 + Gyro 3)
    for(int i = 0; i < 6; i++) {
        _EMA_Wrist.emplace_back(0.4f);
        _EMA_ARM.emplace_back(0.4f);
    }
    // 지자기 센서 (3축)
    for(int i = 0; i < 3; i++) {
        _EMA_Magnet.emplace_back(0.2f); // 지자기는 노이즈가 많으므로 낮게 설정
    }
    
    // 4. 퍼블리셔 생성
    // [Angle] Roll, Pitch, Yaw 전송용
    _pub_Angle = this->create_publisher<std_msgs::msg::Float32MultiArray>("drone/cmd_attitude", 10);

    // [Flex] Throttle, Timestamp 전송용
    _pub_Flex = this->create_publisher<std_msgs::msg::Float32MultiArray>("drone/cmd_throttle", 10);

    // 5. 구독자 생성
    _sub_Wrist_IMU = this->create_subscription<std_msgs::msg::Float32MultiArray>(
        "IMU_Wrist_Data", 10, std::bind(&Signal_Processor::_callback_Wrist_IMU, this, std::placeholders::_1));

    _sub_ARM_IMU = this->create_subscription<std_msgs::msg::Float32MultiArray>(
        "IMU_ARM_Data", 10, std::bind(&Signal_Processor::_callback_ARM_IMU, this, std::placeholders::_1));

    _sub_Magnet = this->create_subscription<std_msgs::msg::Float32MultiArray>(
        "Magnet_Data", 10, std::bind(&Signal_Processor::_callback_Magnet, this, std::placeholders::_1)); 

    _sub_Flex = this->create_subscription<std_msgs::msg::Float32MultiArray>(
        "Flex_Data", 10, std::bind(&Signal_Processor::_callback_Flex, this, std::placeholders::_1));

    // 6. 타이머 설정 (각도 20ms, 스로틀 50ms)
    _timer_Angle = this->create_wall_timer(20ms, std::bind(&Signal_Processor::_Filter_Angle, this));
    _timer_Flex = this->create_wall_timer(20ms, std::bind(&Signal_Processor::_Filter_Flex, this));

    // 데이터 배열 0으로 초기화
    std::fill(std::begin(_raw_Wrist_data), std::end(_raw_Wrist_data), 0.0f);
    std::fill(std::begin(_raw_ARM_data), std::end(_raw_ARM_data), 0.0f);
    std::fill(std::begin(_raw_Magnet_data), std::end(_raw_Magnet_data), 0.0f);
    std::fill(std::begin(_raw_Flex_data), std::end(_raw_Flex_data), 0.0f);

    /*
    if(msg->data.size() < 7) return;
    for(int i=0; i<6; i++){
        _raw_arm_imu[i] = _EMA_ARM[i].initialize(initialized_ARM_imu);
    }
        
    if(msg->data.size() < 7) return;
    for(int i=0; i<6; i++){
        _raw_Wrist_imu[i] = _EMA_Wrist[i].initialize(initialized_Wrist_imu);
    }

    if(msg->data.size() < 4) return;
    for(int i=0; i<3; i++){
        _raw_magnet[i] = _EMA_Magnet[i].initialize(initialized_Magnet);
    }

    if(msg->data.size() < 2) return;
    // Flex 데이터 이니셜라이즈
    _raw_Flex_data[0] = EMA_Flex_in.initialize(initialized_arm_imu); 
    }
    */

    RCLCPP_INFO(this->get_logger(), "Signal_Processor_Node_Initialized.");
}

// ========================================================================================
// [구독 콜백] 센서 데이터 수신 및 1차 전처리 (EMA)
// ========================================================================================

void Signal_Processor::_callback_Wrist_IMU(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
    if(msg->data.size() < 7) return;
    for(int i=0; i<6; i++){
        _raw_Wrist_data[i] = _EMA_Wrist[i].filter(msg->data[i]);
    }
    _raw_Wrist_data[6] = msg->data[6]; // Timestamp
}

void Signal_Processor::_callback_ARM_IMU(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
    if(msg->data.size() < 7) return;
    for(int i=0; i<6; i++){
        _raw_ARM_data[i] = _EMA_ARM[i].filter(msg->data[i]);
    }
}

void Signal_Processor::_callback_Magnet(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
    if(msg->data.size() < 4) return;
    for(int i=0; i<3; i++){
        _raw_Magnet_data[i] = _EMA_Magnet[i].filter(msg->data[i]);
    }
    _raw_Magnet_data[3] = msg->data[3];
}

void Signal_Processor::_callback_Flex(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
    if(msg->data.size() < 2) return;
    // Flex 데이터는 여기서 1차 필터링(EMA_Flex_in) 수행
    _raw_Flex_data[0] = _EMA_Flex_in.filter(msg->data[0]); 
    _raw_Flex_data[1] = msg->data[1]; // Timestamp
}

// ========================================================================================
// [타이머 콜백 1] 자세 제어 (Angle Control Loop)
// ========================================================================================

void Signal_Processor::_Filter_Angle() {
    // 1. 매드윅 필터 업데이트
    // 손목 (9축: Gyro, Acc, Mag)
    _Madgwick_Wrist_Filter.update(
        _raw_Wrist_data[3], _raw_Wrist_data[4], _raw_Wrist_data[5], // Gyro
        _raw_Wrist_data[0], _raw_Wrist_data[1], _raw_Wrist_data[2], // Accel
        _raw_Magnet_data[0], _raw_Magnet_data[1], _raw_Magnet_data[2]     // Magnet
    );
    // 팔 (6축: Gyro, Acc) - 지자기 없음
    _Madgwick_ARM_Filter.update(
        _raw_ARM_data[3], _raw_ARM_data[4], _raw_ARM_data[5],
        _raw_ARM_data[0], _raw_ARM_data[1], _raw_ARM_data[2]
    );

    // 2. 쿼터니언 추출 및 상대 회전 계산
    Quaternion_to_Euler::Quat Q_Wrist, Q_ARM;
    _Madgwick_Wrist_Filter.getQuatarnion(Q_Wrist.w, Q_Wrist.x, Q_Wrist.y, Q_Wrist.z);
    _Madgwick_ARM_Filter.getQuatarnion(Q_ARM.w, Q_ARM.x, Q_ARM.y, Q_ARM.z); 

    // 팔(Arm)을 기준으로 손목(Wrist)이 얼마나 꺾였는지 계산 (상대 쿼터니언)
    Quaternion_to_Euler::Quat Q_rel = _to_euler.get_Relative(Q_ARM, Q_Wrist);

    // 3. 오일러 각 변환 (Roll, Pitch, Yaw)
    Quaternion_to_Euler::Euler_Angles Euler_Angle = _to_euler.toEuler(Q_rel.x, Q_rel.y, Q_rel.z, Q_rel.w);

    // 4. 제어 값 매핑 (-1.0 ~ 1.0)
    float map_Roll = _Map_Roll.map(Euler_Angle.roll);
    float map_Pitch = _Map_Pitch.map(Euler_Angle.pitch);
    float map_Yaw = _Map_Yaw.map(Euler_Angle.yaw);

    // 5. 메시지 생성 및 퍼블리시
    std_msgs::msg::Float32MultiArray msg;
    msg.data.resize(4); // Roll, Pitch, Yaw
    msg.data[0] = map_Roll;
    msg.data[1] = map_Pitch;
    msg.data[2] = map_Yaw;
    msg.data[3] = _raw_Wrist_data[6];//타임스탬프를 넣고싶은데 어느데이터꺼를 써야할지 모르겠네;

    _pub_Angle->publish(msg);
}

// ========================================================================================
// [타이머 콜백 2] 스로틀 제어 (Throttle Control Loop)
// ========================================================================================
void Signal_Processor::_Filter_Flex() {
    // 1. 매핑 (Mapping)
    // _raw_Flex_data[0]은 이미 _sub_Flex에서 EMA_Flex_in 필터를 거친 상태입니다.
    float Mapped_flex = _Map_Flex.map(_raw_Flex_data[0]);

    // 2. 2차 필터링 (부드러운 가속을 위해 EMA_Flex_out 적용)
    float final_throttle = _EMA_Flex_out.filter(Mapped_flex);

    // 3. 메시지 생성 (값 + 타임스탬프)
    std_msgs::msg::Float32MultiArray msg;
    msg.data.resize(2); // [Throttle Value, Timestamp]

    msg.data[0] = final_throttle; // 스로틀 제어값 (0.0 ~ 1.0)
    msg.data[1] = _raw_Flex_data[1];    // ESP32에서 온 원본 타임스탬프 사용 (동기화용)

    _pub_Flex->publish(msg);
}
