#include "hri_signal_pkg/HRI_Drone_HPP/Signal_Processor_Node.hpp"

using namespace std::chrono_literals;

// 원시 바이트 배열에서 데이터를 추출하여 정렬된 구조체에 저장하는 함수입니다.
// Function to extract data from raw byte array and store in aligned struct.
bool Signal_Processor::_parse_payload(const uint8_t* payload, size_t payload_len, SensorAligned& out) {
    // 페이로드 길이가 예상된 길이와 다른지 확인합니다.
    // Check if the payload length differs from the expected length.
    if (payload_len != 39) return false;
    
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
    out.t_us = le32(payload + off); off += 4;
    out.seq  = le16(payload + off); off += 2;
    out.flags = payload[off]; off += 1;

    // 오프셋이 정확히 39바이트만큼 이동했는지 확인 후 반환합니다.
    // Check if the offset has moved exactly 39 bytes and return.
    return off == 39;
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
    // [Angle] 드론의 자세 제어 명령(Roll, Pitch, Yaw)을 전송할 퍼블리셔를 생성합니다.
    // [Angle] Create publisher to send drone attitude control commands (Roll, Pitch, Yaw).
    _pub_Attitude = this->create_publisher<hri_signal_pkg::msg::HRI_Attitude>("drone/cmd_attitude", 10);

    // [Flex] 드론의 스로틀 값과 타임스탬프를 전송할 퍼블리셔를 생성합니다.
    // [Flex] Create publisher to send drone throttle value and timestamp.
    _pub_Flex = this->create_publisher<hri_signal_pkg::msg::HRI_Flex>("drone/cmd_flex", 10);

    // ELRS 패킷 데이터를 수신할 구독자를 생성하고 콜백 함수를 등록합니다.
    // Create subscriber to receive ELRS packet data and register callback function.
    _sub_Packet = this->create_subscription<std_msgs::msg::UInt8MultiArray>(
        "elrs_packet", 10, std::bind(&Signal_Processor::_callback_Packet, this, std::placeholders::_1));

    // 각도 계산(20ms)과 스로틀 처리(20ms)를 위한 주기적 타이머를 설정합니다.
    // Set periodic timers for angle calculation (20ms) and throttle processing (20ms).
    _timer_Attitude = this->create_wall_timer(20ms, std::bind(&Signal_Processor::_Filter_Angle, this));
    _timer_Flex = this->create_wall_timer(20ms, std::bind(&Signal_Processor::_Filter_Flex, this));

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

    // 디버깅을 위해 시퀀스 번호와 주요 센서 값을 로그로 출력합니다.
    // Log sequence number and key sensor values for debugging.
    RCLCPP_INFO(this->get_logger(), 
        "Seq: %d | W_AccX: %.3f | Flex: %.0f | Clip: %s",
        (unsigned)_Info.Seq, _Wrist.acc[0], _Flex.flex_adc, is_clipped ? "Y" : "N"
    );
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

    // 5. 상대 쿼터니언을 오일러 각(Roll, Pitch, Yaw)으로 변환합니다.
    // 5. Convert relative quaternion to Euler angles (Roll, Pitch, Yaw).
    Quaternion_to_Euler::Euler_Angles Euler_Angle = _to_euler.toEuler(Q_rel.x, Q_rel.y, Q_rel.z, Q_rel.w);

    // 6. 계산된 각도를 제어에 적합한 범위(-1.0 ~ 1.0)로 매핑합니다.
    // 6. Map calculated angles to a range suitable for control (-1.0 ~ 1.0).
    float Roll = Euler_Angle.roll;
    float Pitch = Euler_Angle.pitch;
    float Yaw = Euler_Angle.yaw;

    // 7. 결과 메시지를 생성하고 타임스탬프와 함께 발행합니다.
    // 7. Create result message and publish it along with the timestamp.
    hri_signal_pkg::msg::HRI_Attitude msg;

    msg.header.stamp = this->now();
    msg.header.frame_id = "Wrist_Rel_Arm";

    msg.t_us = _Info.Timestamp;
    msg.seq = _Info.Seq;
    msg.flags = _Info.Flags;

    msg.roll = Roll;
    msg.pitch = Pitch;
    msg.yaw = 0.0f;

    msg.W_e_acc = w_error;
    msg.W_beta = w_final_beta;

    msg.A_e_acc = a_error;
    msg.A_beta = a_final_beta;

    msg.fusion_mode = fusion_mode;

    _pub_Attitude->publish(msg);
}

// [타이머 콜백 2] Flex 센서 값을 처리하여 스로틀 명령을 생성하는 함수입니다.
// [Timer Callback 2] Function to process Flex sensor values and generate throttle commands.
void Signal_Processor::_Filter_Flex() {
    // 1. 1차 필터링된 Flex 값을 제어 범위로 매핑합니다.
    // 1. Map the 1st-stage filtered Flex value to the control range.
    float threshold = 50;
    const float flex_norm = std::clamp((_Flex.flex_adc - threshold) / (4095.0f - threshold), 0.0f, 1.0f);

    // 3. 스로틀 값과 동기화용 타임스탬프를 메시지에 담습니다.
    // 3. Put throttle value and synchronization timestamp into the message.
    hri_signal_pkg::msg::HRI_Flex msg;

    msg.header.stamp = this->now();
    msg.header.frame_id = "index_finger_flex";

    msg.t_us  = _Info.Timestamp;
    msg.seq   = _Info.Seq;
    msg.flags = _Info.Flags;

    msg.flex_adc_f = _Flex.flex_adc;
    msg.flex_norm  = flex_norm;
    msg.alpha_used = this->_used_alpha_flex;

    _pub_Flex->publish(msg);
}