#include "hri_signal_pkg/HRI_Drone_HPP/Signal_Processor_Node.hpp"

using namespace std::chrono_literals;

// 원시 바이트 배열에서 데이터를 추출하여 정렬된 구조체에 저장하는 함수입니다.
// Function to extract data from raw byte array and store in aligned struct.
bool Signal_Processor::_parse_payload(const uint8_t* payload, size_t payload_len, SensorAligned& out) {
    // 페이로드 길이가 예상된 길이와 다른지 확인합니다.
    // Check if the payload length differs from the expected length.
    if (payload_len != 40) return false;
    
    int off = 0;
    // 람다 함수를 사용하여 int16 및 uint16 데이터를 순차적으로 읽어옵니다.
    // Read int16 and uint16 data sequentially using lambda functions.
    auto rd_i16 = [&](int16_t& v){ v = Signal_Processor::le16s(payload + off); off += 2; };
    auto rd_u16 = [&](uint16_t& v){ v = Signal_Processor::le16(payload + off); off += 2; };

    // 가속도, 자이로, 지자기 데이터를 순서대로 파싱합니다.
    // Parse accelerometer, gyro, and magnetometer data in order.
    for(int i=0;i<3;i++) rd_i16(out.acc[i]);
    for(int i=0;i<3;i++) rd_i16(out.gyro[i]);
    for(int i=0;i<3;i++) rd_i16(out.acc_arm[i]);
    for(int i=0;i<3;i++) rd_i16(out.gyro_arm[i]);
    for(int i=0;i<3;i++) rd_i16(out.mag[i]);

    // Flex 센서, 타임스탬프, 시퀀스, 플래그를 파싱합니다.
    // Parse Flex sensor, timestamp, sequence, and flags.
    rd_u16(out.flex_adc);
    out.button = (payload[off] != 0); off += 1;
    out.t_us = le32(payload + off); off += 4;
    out.seq  = le16(payload + off); off += 2;
    out.flags = payload[off]; off += 1;

    // 오프셋이 정확히 39바이트만큼 이동했는지 확인 후 반환합니다.
    // Check if the offset has moved exactly 39 bytes and return.
    return off == 40;
}

// 클래스 생성자: 필터, 퍼블리셔, 서브스크라이버, 타이머를 초기화합니다.
// Class Constructor: Initialize filters, publishers, subscribers, and timers.
Signal_Processor::Signal_Processor() 
    : Node("Signal_Processor_Node"),

    // [설정] 적응형 베타 객체 초기화 (e0, e1, out0=Static_Beta, out1=Motion_Beta)
    // [Config] Initialize adaptive beta objects (e0, e1, out0=Static_Beta, out1=Motion_Beta)
    
    // Wrist 9축: 정지(0.08) -> 움직임(0.01)
    _Smooth_Beta_Wrist_9Axis(0.06f, 0.27f, 0.08f, 0.01f),
    // Wrist 6축: 정지(0.06) -> 움직임(0.008)
    _Smooth_Beta_Wrist_6Axis(0.05f, 0.25f, 0.06f, 0.008f),
    // Arm 6축: 정지(0.06) -> 움직임(0.012), 오차 범위 넓음(0.30)
    _Smooth_Beta_Arm_6Axis(0.08f, 0.20f, 0.06f, 0.012f),

    // 매드윅 필터를 50Hz 샘플링 속도와 베타 게인 0.1로 초기화합니다.
    // Initialize Madgwick filters with 50Hz sampling rate and 0.1 beta gain.
    _Madgwick_Wrist_Filter(50.0f),
    _Madgwick_ARM_Filter(50.0f)
{
    // [Angle] 웨어러블 기기에서 센서값을 필터링한 값을 전송할 퍼블리셔를 생성합니다.
    // [Angle] Create publisher to send drone attitude control commands (Roll, Pitch, Yaw).
    _pub_Filtered = this->create_publisher<hri_signal_pkg::msg::HRIFiltered>("wearable/filtered", 10);

    _pub_raw = this->create_publisher<std_msgs::msg::Float32MultiArray>("drone/raw_data", 10);

    _pub_visual_pose = this->create_publisher<geometry_msgs::msg::PoseStamped>("drone/visual_pose", 10);
    
    // ELRS 패킷 데이터를 수신할 구독자를 생성하고 콜백 함수를 등록합니다.
    // Create subscriber to receive ELRS packet data and register callback function.
    _sub_Packet = this->create_subscription<std_msgs::msg::UInt8MultiArray>(
        "elrs_packet", 10, std::bind(&Signal_Processor::_callback_Packet, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Signal_Processor_Node_Initialized.");
}

// CRC16-CCITT 알고리즘을 사용하여 데이터 무결성을 검증하는 함수입니다.
// Function to verify data integrity using the CRC16-CCITT algorithm.
uint16_t Signal_Processor::_calculate_crc16(const uint8_t* data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (int j = 0; j < 8; j++) {
            if (crc & 0x8000) crc = (crc << 1) ^ 0x1021;
            else crc <<= 1;
        }
    }
    return crc & 0xFFFF;
}

// 패킷 수신 콜백: 데이터 검증, 파싱, EMA 필터링을 수행합니다.
// Packet Callback: Perform data validation, parsing, and EMA filtering.
void Signal_Processor::_callback_Packet(const std_msgs::msg::UInt8MultiArray::SharedPtr msg) {
    // 1. 수신된 패킷의 크기가 정의된 프레임 길이(44바이트)와 일치하는지 확인합니다.
    // 1. Check if the received packet size matches the defined frame length (44 bytes).
    if(msg->data.size() != FRAME_LEN) {
        RCLCPP_WARN(this->get_logger(), "Invalid_Packet size: %lu", msg->data.size());
        return;
    } 

    const uint8_t* raw_packet = msg->data.data();

    // 2. 패킷 헤더(SOF)가 올바른지 확인하여 데이터 정렬을 보장합니다.
    // 2. Verify packet header (SOF) to ensure data alignment.
    if (raw_packet[0] != SOF_1 || raw_packet[1] != SOF_2) {
        RCLCPP_WARN(this->get_logger(), "Invalid SOF detected.");
        return;
    }

    // 3. 페이로드 길이 정보가 예상 값과 일치하는지 검사합니다.
    // 3. Check if the payload length information matches the expected value.
    if (raw_packet[2] != PAYLOAD_LEN) {
        RCLCPP_WARN(this->get_logger(), "Invalid LEN: %u (expected %zu)", (unsigned)raw_packet[2], PAYLOAD_LEN);
        return;
    }  

    // 4. 데이터의 뒷부분에 있는 CRC 값을 제외한 나머지 부분으로 CRC를 계산합니다.
    // 4. Calculate CRC for the data excluding the CRC value at the end.
    uint16_t calculated_crc = _calculate_crc16(raw_packet + 2, FRAME_LEN - 4);

    // 수신된 패킷의 끝부분에서 CRC 값을 추출하여 결합합니다.
    // Extract and combine the CRC value from the end of the received packet.
    uint16_t received_crc = (uint16_t)raw_packet[FRAME_LEN-2] | ((uint16_t)raw_packet[FRAME_LEN-1] << 8);

    // 계산된 CRC와 수신된 CRC가 일치하지 않으면 패킷을 버립니다.
    // Discard the packet if the calculated CRC does not match the received CRC.
    if(received_crc != calculated_crc) {
        RCLCPP_ERROR(this->get_logger(), "CRC Fail! Recv:%04X, Calc: %04X", received_crc, calculated_crc);
        return;
    }

    // 헤더 이후의 페이로드 데이터를 파싱하여 구조체로 변환합니다.
    // Parse payload data after the header and convert it into a struct.
    const uint8_t* payload = raw_packet + HEADER_LEN;
    SensorAligned sensor_data;
    if(!_parse_payload(payload, raw_packet[2], sensor_data)) return;

    // 파싱된 메타 정보를 멤버 변수에 저장하여 다른 함수에서 사용할 수 있게 합니다.
    // Store parsed meta information in member variables for use in other functions.
    uint8_t flags = sensor_data.flags;
    _Info.Timestamp = sensor_data.t_us;
    _Info.Seq  = sensor_data.seq;
    _Info.Flags = flags;

    // 클리핑 플래그에 따라 필터 게인(알파값)을 동적으로 조정합니다.
    // Dynamically adjust filter gain (alpha value) based on the clipping flag.
    bool is_clipped = (flags & FLAG_CLIPPED);
    float alpha_acc  = is_clipped ? A_ACC_CLIP  : A_ACC_NORM;
    float alpha_gyro = is_clipped ? A_GYRO_CLIP : A_GYRO_NORM;
    float alpha_flex = is_clipped ? A_FLEX_CLIP : A_FLEX_NORM;
    this->_used_alpha_flex = alpha_flex;

    // ---------------------------------------------------------
    // IMU 데이터 필터링 및 저장
    // IMU Data Filtering and Storage
    // ---------------------------------------------------------
    if (flags & FLAG_IMU_VALID) {
        for (int i=0;i<3;i++){
            // 원시 정수 데이터를 실제 물리량(가속도, 각속도)으로 스케일링합니다.
            // Scale raw integer data to actual physical quantities (acceleration, angular velocity).
            float w_acc  = sensor_data.acc[i]      * ACC_SCALE;
            float w_gyro = sensor_data.gyro[i]     * GYRO_SCALE;
            float a_acc  = sensor_data.acc_arm[i]  * ACC_SCALE;
            float a_gyro = sensor_data.gyro_arm[i] * GYRO_SCALE;

            // EMA 필터를 적용하여 노이즈를 제거하고 멤버 변수에 저장합니다.
            // Apply EMA filter to remove noise and store in member variables.
            _Wrist.acc[i]  = _Wrist_EMA_Acc[i].filter(w_acc,  alpha_acc);
            _Wrist.gyro[i] = _Wrist_EMA_Gyro[i].filter(w_gyro, alpha_gyro);
            _ARM.acc[i]    = _ARM_EMA_Acc[i].filter(a_acc, alpha_acc);
            _ARM.gyro[i]   = _ARM_EMA_Gyro[i].filter(a_gyro, alpha_gyro);
        }
    }

    // ---------------------------------------------------------
    // 지자기 데이터 필터링 및 저장
    // Magnetometer Data Filtering and Storage
    // ---------------------------------------------------------
    if (flags & FLAG_MAG_UPDATED) {
        for (int i=0;i<3;i++){
            float mag  = sensor_data.mag[i] * MAGNET_SCALE;              // raw counts 유지(현재)
            _Magnet.mag[i] = _Magnet_EMA[i].filter(mag, A_MAG_NORM);
        }

        // MAG가 갱신된 순간의 Wrist IMU 스냅샷(멀티레이트 정합)
        _Wrist_at_mag = _Wrist;

        // “새 MAG 샘플 들어옴” 래치(필터 틱에서 1회 소비)
        _mag_updated_latched.store(true, std::memory_order_release);
        }

    // ---------------------------------------------------------
    // Flex 센서 데이터 필터링 및 저장
    // Flex Sensor Data Filtering and Storage
    // ---------------------------------------------------------
    if (flags & FLAG_FLEX_VALID) {
        const float x = static_cast<float>(sensor_data.flex_adc);
        _Flex.flex_adc = _Flex_EMA.filter(x, alpha_flex);
    }

    if (flags & FLAG_BUTTON_VALID) {
        _button_on_off = sensor_data.button;
    }

    // 디버깅을 위해 시퀀스 번호와 주요 센서 값을 로그로 출력합니다.
    // Log sequence number and key sensor values for debugging.
    RCLCPP_INFO(this->get_logger(), 
        "Seq: %d | W_AccX: %.3f | Flex: %.0f | Clip: %s",
        (unsigned)_Info.Seq, _Wrist.acc[0], _Flex.flex_adc, is_clipped ? "Y" : "N"
    );
    // =========================================================================
    // [추가된 영역] PlotJuggler 2D 그래프 분석용 원시 데이터(Raw Data) 퍼블리시
    // [Added Area] Publish raw data for PlotJuggler 2D graph analysis.
    // =========================================================================

    // [변수] 다수의 센서 값을 담을 빈 Float32 배열 객체를 생성합니다.
    // [Variable] Create an empty Float32 array object to hold multiple sensor values.
    std_msgs::msg::Float32MultiArray msg_raw_array;

    // [고정] PlotJuggler에서 선을 명확히 구분하기 위해, 데이터는 항상 고정된 순서(Index)로 push_back 해야 합니다.
    // [Fixed] To clearly distinguish lines in PlotJuggler, data must always be pushed back in a fixed order (Index).
    
    // Index [0]: 통신 패킷 누락을 확인하기 위한 시퀀스 번호 (시간 대신 사용)
    msg_raw_array.data.push_back(static_cast<float>(sensor_data.seq));

    // Index [1, 2, 3]: 손목(Wrist)의 가속도 원시 데이터 (X, Y, Z) - 필터 안 거친 순수 스케일 값
    msg_raw_array.data.push_back(sensor_data.acc[0] * ACC_SCALE);
    msg_raw_array.data.push_back(sensor_data.acc[1] * ACC_SCALE);
    msg_raw_array.data.push_back(sensor_data.acc[2] * ACC_SCALE);

    // Index [4, 5, 6]: 손목(Wrist)의 각속도 원시 데이터 (X, Y, Z)
    msg_raw_array.data.push_back(sensor_data.gyro[0] * GYRO_SCALE);
    msg_raw_array.data.push_back(sensor_data.gyro[1] * GYRO_SCALE);
    msg_raw_array.data.push_back(sensor_data.gyro[2] * GYRO_SCALE);

    // Index [7, 8, 9]: 지자기(Magnet) 원시 데이터 (X, Y, Z)
    msg_raw_array.data.push_back(sensor_data.mag[0] * MAGNET_SCALE);
    msg_raw_array.data.push_back(sensor_data.mag[1] * MAGNET_SCALE);
    msg_raw_array.data.push_back(sensor_data.mag[2] * MAGNET_SCALE);

    // Index [10]: 플렉스(Flex) 센서 원시 데이터
    msg_raw_array.data.push_back(static_cast<float>(sensor_data.flex_adc));

    // [고정] 완성된 배열을 헤더에서 선언한 _pub_raw 퍼블리셔를 통해 즉각 송출합니다.
    // [Fixed] Immediately transmit the completed array via the _pub_raw publisher declared in the header.
    _pub_raw->publish(msg_raw_array);

    _Process_Filtered();
}

// [핵심] 적응형 베타 + 9축/6축 자동 전환 + 자세 계산
// [Core] Adaptive Beta + Auto 9/6-Axis Switch + Attitude Calculation
void Signal_Processor::_Filter_Angle() {
    // 1. 손목 센서 퓨전 (Madgwick Filter): 자이로, 가속도, 지자기 데이터를 사용합니다.
    // 1. Wrist Sensor Fusion (Madgwick Filter): Uses Gyro, Accel, and Magnet data.
    // 주의: Madgwick update 함수 인자 순서는 (gx, gy, gz, ax, ay, az, mx, my, mz) 입니다.
    // Note: Madgwick update function argument order is (gx, gy, gz, ax, ay, az, mx, my, mz).
    float w_ax = _Wrist.acc[0], w_ay = _Wrist.acc[1], w_az = _Wrist.acc[2];
    
    // [KOR] 손목 가속도 벡터의 노름(||a||)을 계산한다.
    // [ENG] Compute the norm (||a||) of the wrist acceleration vector.
    float w_norm = std::sqrt(w_ax*w_ax + w_ay*w_ay + w_az*w_az);

    // [KOR] 1G(정규화 기준 1.0)에서 얼마나 벗어났는지 오차로 계산한다.
    // [ENG] Compute the deviation from 1G (normalized target 1.0) as an error.
    float w_error = std::fabs(w_norm - 1.0f);

    const float wgx = _Wrist.gyro[0], wgy = _Wrist.gyro[1], wgz = _Wrist.gyro[2];
    const float w_gyro_norm = std::sqrt(wgx*wgx + wgy*wgy + wgz*wgz);

    // -----------------------------
    // 1) MAG update / Q_mag update
    // -----------------------------
    const uint32_t now_us = monotonic_us();                   // 패킷 타임(us) (너 구조에 맞게)
    const bool mag_updated = _mag_updated_latched.exchange(false, std::memory_order_acq_rel);   // 이번 틱에 새 MAG 들어왔는지

    if (mag_updated) _wrist_mag_ever_received = true;

    // Q_mag는 "연속" 품질 점수(0..1)
    float Q_mag = 0.0f;
    if (mag_updated) {
        Q_mag = _MagQ_Wrist.Quality_Update(now_us, _Magnet.mag, _Wrist_at_mag.acc, _Wrist_at_mag.gyro);
    } else {
        Q_mag = _MagQ_Wrist.onTick(now_us);
    }

    // -----------------------------
    // 2) Baseline update (SAFE segment only)
    //    - 반드시 "안전 구간" + "새 MAG 수신"일 때만 업데이트
    // -----------------------------
    if (mag_updated) {
        const float axm = _Wrist_at_mag.acc[0], aym = _Wrist_at_mag.acc[1], azm = _Wrist_at_mag.acc[2];
        const float w_norm_m  = std::sqrt(axm*axm + aym*aym + azm*azm);
        const float w_error_m = std::fabs(w_norm_m - 1.0f);

        const float wg0 = _Wrist_at_mag.gyro[0], wg1 = _Wrist_at_mag.gyro[1], wg2 = _Wrist_at_mag.gyro[2];
        const float gyro_norm = std::sqrt(wg0*wg0 + wg1*wg1 + wg2*wg2);

        const bool static_like = (gyro_norm < 0.4f);   // rad/s 기준
        const bool accel_ok    = (w_error_m  < 0.15f); // 1g 기준
        const bool q_high      = (Q_mag > 0.80f);

        if (static_like && accel_ok && q_high) {
            const float mx = _Magnet.mag[0], my = _Magnet.mag[1], mz = _Magnet.mag[2];
            const float mag_norm = std::sqrt(mx*mx + my*my + mz*mz);
            _MagQ_Wrist.Baseline_Update(mag_norm);
        }
    }

    // -----------------------------
    // 3) Hysteresis gate: 9-axis enable/disable
    // -----------------------------
    if (!_wrist_mag_ever_received) {
        _wrist_mag_fusion_enabled = false;
    } else {
        if (!_wrist_mag_fusion_enabled && (Q_mag >= QMAG_ON))  _wrist_mag_fusion_enabled = true;
        if ( _wrist_mag_fusion_enabled && (Q_mag <= QMAG_OFF)) _wrist_mag_fusion_enabled = false;
    }

    // -----------------------------
    // 4) Adaptive beta: blend by Q_mag (continuous)
    //    - Q=0 -> 6-axis beta, Q=1 -> 9-axis beta
    // -----------------------------
    const float beta6 = _Smooth_Beta_Wrist_6Axis.interpol(w_error);
    const float beta9 = _Smooth_Beta_Wrist_9Axis.interpol(w_error);

    float w_target_beta = (1.0f - Q_mag) * beta6 + Q_mag * beta9;
    float w_final_beta  = _Quaternion_Beta_EMA.filter(w_target_beta, 0.2f);

    // -----------------------------
    // 5) Madgwick update: use hysteresis result
    // -----------------------------
    uint8_t fusion_mode = 0;

    if (_wrist_mag_fusion_enabled) {
        // MAG가 이번 틱에 새로 안 들어와도, 마지막 mag 값을 사용하되 Q_mag가 TTL로 떨어지면서 자동 OFF됨
        _Madgwick_Wrist_Filter.update(
            _Wrist.gyro[0], _Wrist.gyro[1], _Wrist.gyro[2],
            _Wrist.acc[0],  _Wrist.acc[1],  _Wrist.acc[2],
            _Magnet.mag[0], _Magnet.mag[1], _Magnet.mag[2],
            w_final_beta
        );
        fusion_mode = 1;
    } else {
        _Madgwick_Wrist_Filter.update(
            _Wrist.gyro[0], _Wrist.gyro[1], _Wrist.gyro[2],
            _Wrist.acc[0],  _Wrist.acc[1],  _Wrist.acc[2],
            w_final_beta
        );
        fusion_mode = 0;
    }

    // -----------------------------
    // 6) Arm 6-axis (원코드 유지)
    // -----------------------------
    float a_ax = _ARM.acc[0], a_ay = _ARM.acc[1], a_az = _ARM.acc[2];
    float a_norm = std::sqrt(a_ax*a_ax + a_ay*a_ay + a_az*a_az);
    float a_error = std::fabs(a_norm - 1.0f);

    float a_target_beta = _Smooth_Beta_Arm_6Axis.interpol(a_error);

    // NOTE: 너 EMA_Filter가 alpha 인자 없는 오버로드면, 아래를 네 시그니처에 맞춰 통일해라.
    float a_final_beta  = _Quaternion_Beta_EMA.filter(a_target_beta, 0.2f);

    _Madgwick_ARM_Filter.update(
        _ARM.gyro[0], _ARM.gyro[1], _ARM.gyro[2],
        _ARM.acc[0],  _ARM.acc[1],  _ARM.acc[2],
        a_final_beta
    );

    // 3. 필터 내부의 쿼터니언 값을 추출합니다.
    // 3. Extract quaternion values from inside the filters.
    Quaternion_to_Euler::Quat Q_Wrist, Q_ARM;
    _Madgwick_Wrist_Filter.getQuaternion(Q_Wrist.w, Q_Wrist.x, Q_Wrist.y, Q_Wrist.z);
    _Madgwick_ARM_Filter.getQuaternion(Q_ARM.w, Q_ARM.x, Q_ARM.y, Q_ARM.z); 

    // 4. 팔(Arm) 좌표계를 기준으로 손목(Wrist)의 상대적 회전량을 계산합니다.
    // 4. Calculate relative rotation of Wrist based on Arm coordinate system.
    Quaternion_to_Euler::Quat Q_rel = _to_euler.get_Relative(Q_ARM, Q_Wrist);
    
    _Q_rel_cached = Q_rel;
    _w_e_acc_cached = w_error;
    _w_beta_cached = w_final_beta;
    _a_e_acc_cached = a_error;
    _a_beta_cached = a_final_beta;
    _w_gyro_norm_cached = w_gyro_norm;
    _w_acc_err_abs_cached = w_error;
    _mag_quality_cached = Q_mag;
    _fusion_mode_cached = fusion_mode;

    // [고정] 드론의 3D 자세를 폭스글러브로 보내기 위한 표준 메시지 객체를 생성합니다.
    // [Fixed] Create a standard message object to send the drone's 3D pose to Foxglove.
    geometry_msgs::msg::PoseStamped visual_msg;
    visual_msg.header.stamp = this->now();
    visual_msg.header.frame_id = "world";
    visual_msg.pose.orientation.w = Q_rel.w;
    visual_msg.pose.orientation.x = Q_rel.x;
    visual_msg.pose.orientation.y = Q_rel.y;
    visual_msg.pose.orientation.z = Q_rel.z;
    _pub_visual_pose->publish(visual_msg);
}

// [타이머 콜백 2] Flex 센서 값을 처리하여 스로틀 명령을 생성하는 함수입니다.
// [Timer Callback 2] Function to process Flex sensor values and generate throttle commands.
void Signal_Processor::_Filter_Flex() {
    // 1. 1차 필터링된 Flex 값을 제어 범위로 매핑합니다.
    // 1. Map the 1st-stage filtered Flex value to the control range.
    float threshold = 50.0f;
    const float flex_norm = std::clamp((_Flex.flex_adc - threshold) / (4095.0f - threshold), 0.0f, 1.0f);

    _flex_adc_f_cached = _Flex.flex_adc;
    _flex_norm_cached = flex_norm;
}

void Signal_Processor::_Publisher_Filtered() {
    hri_signal_pkg::msg::HRIFiltered msg;

    msg.header.stamp = this->now();
    msg.header.frame_id = "Wrist_Rel_Arm";

    msg.t_us = _Info.Timestamp;
    msg.seq = _Info.Seq;
    msg.flags = _Info.Flags;

    msg.quat_x = _Q_rel_cached.x;
    msg.quat_y = _Q_rel_cached.y;
    msg.quat_z = _Q_rel_cached.z;
    msg.quat_w = _Q_rel_cached.w;

    msg.w_e_acc = _w_e_acc_cached;
    msg.w_beta = _w_beta_cached;
    msg.a_e_acc = _a_e_acc_cached;
    msg.a_beta = _a_beta_cached;

    msg.w_gyro_norm = _w_gyro_norm_cached;
    msg.w_acc_err_abs = _w_acc_err_abs_cached;
    msg.mag_quality = _mag_quality_cached;

    msg.flex_adc_f = _flex_adc_f_cached;
    msg.flex_norm = _flex_norm_cached;
    msg.alpha_used = _used_alpha_flex;

    msg.button_on_off = _button_on_off;
    msg.fusion_mode = _fusion_mode_cached;

    _pub_Filtered->publish(msg);
}

void Signal_Processor::_Process_Filtered() {
    _Filter_Angle();
    _Filter_Flex();
    _Publisher_Filtered();
}