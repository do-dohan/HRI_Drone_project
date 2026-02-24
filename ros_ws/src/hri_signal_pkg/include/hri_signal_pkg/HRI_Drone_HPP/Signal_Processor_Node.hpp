#ifndef SIGNAL_PROCESSOR_NODE_HPP
#define SIGNAL_PROCESSOR_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "hri_signal_pkg/msg/hri_attitude.hpp"
#include "hri_signal_pkg/msg/hri_flex.hpp"

// 사용자 정의 라이브러리 헤더 포함
// Include user-defined library headers
#include "hri_signal_pkg/HRI_Drone_HPP/Madgwick_Filter.hpp"
#include "hri_signal_pkg/HRI_Drone_HPP/EMA_Filter.hpp"
#include "hri_signal_pkg/HRI_Drone_HPP/Mapping_Utils.hpp"
#include "hri_signal_pkg/HRI_Drone_HPP/Quaternion_to_Euler.hpp"
#include "hri_signal_pkg/HRI_Drone_HPP/Smooth_Step_Utils.hpp"
#include "hri_signal_pkg/HRI_Drone_HPP/Mag_Quality.hpp"

#include <chrono>
#include <array>
#include <memory>
#include <cstring>
#include <cmath>
#include <algorithm>
// [KOR] 멀티스레드 안전한 플래그를 위해 atomic을 사용한다.
// [ENG] Use atomic for thread-safe flags.
#include <atomic>

// 비트 연산을 위한 플래그 상수를 정의하여 센서 상태를 확인합니다.
// Define flag constants for bitwise operations to check sensor status.
#define FLAG_IMU_VALID   (1 << 0)
#define FLAG_MAG_UPDATED (1 << 1)
#define FLAG_FLEX_VALID  (1 << 2)
#define FLAG_CLIPPED     (1 << 3)

class Signal_Processor : public rclcpp::Node {
public:
    // 클래스 생성자 선언입니다.
    // Constructor declaration.
    Signal_Processor();

private:
    // =========================================================
    // 상수 정의 (Constants)
    // =========================================================
    // 패킷 시작을 알리는 매직 넘버(Start of Frame)입니다.
    // Magic numbers indicating the Start of Frame (SOF).
    static constexpr uint8_t SOF_1 = 0xAA;
    static constexpr uint8_t SOF_2 = 0x55;

    // 통신 프로토콜에 따른 패킷 길이 정의입니다.
    // Packet length definitions according to the communication protocol.
    static constexpr size_t PAYLOAD_LEN = 39;
    static constexpr size_t HEADER_LEN = 3;
    static constexpr size_t CRC_LEN = 2;
    static constexpr size_t FRAME_LEN = 44;

    // [KOR] LSM6DSOX 고정 설정: Acc ±4g, Gyro ±500 dps 기준 감도(데이터시트).
    // [ENG] LSM6DSOX fixed config: Acc ±4g, Gyro ±500 dps sensitivities (datasheet).
    static constexpr float ACC_MG_PER_LSB    = 0.122f;   // ±4g  (mg/LSB)
    static constexpr float GYRO_MDPS_PER_LSB = 17.50f;   // ±500 (mdps/LSB)

    // 센서의 Raw 데이터를 물리량으로 변환하기 위한 스케일 계수입니다.
    // Scale factors to convert raw sensor data into physical quantities.
    static constexpr float ACC_SCALE  = ACC_MG_PER_LSB / 1000.0f; // g/LSB
    static constexpr float GYRO_SCALE = (GYRO_MDPS_PER_LSB / 1000.0f) * (3.14159265358979323846f / 180.0f); // rad/s/LSB
    static constexpr float MAGNET_SCALE = 1.0f;

    // 일반적인 상황에서의 지수 이동 평균(EMA) 필터 계수입니다.
    // Exponential Moving Average (EMA) filter coefficients for normal conditions.
    static constexpr float A_GYRO_NORM = 0.20f;
    static constexpr float A_ACC_NORM  = 0.15f;
    static constexpr float A_FLEX_NORM = 0.30f;
    static constexpr float A_MAG_NORM  = 0.10f;

    // 데이터 클리핑(비정상 값) 발생 시 적용할 강한 필터 계수입니다.
    // Strong filter coefficients applied when data clipping occurs.
    static constexpr float A_GYRO_CLIP = 0.03f;
    static constexpr float A_ACC_CLIP  = 0.05f;
    static constexpr float A_FLEX_CLIP = 0.10f;

    // =========================================================
    // 리틀 엔디안 디코딩 헬퍼 함수 (Little Endian Decoding Helpers)
    // =========================================================
    static inline uint16_t le16(const uint8_t* p) {
        return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
    }
    static inline int16_t le16s(const uint8_t* p) {
        return (int16_t)le16(p);
    }
    static inline uint32_t le32(const uint8_t* p) {
        return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
    }

    // =========================================================
    // 1. 내부 데이터 저장용 구조체 (Internal Data Storage Structs)
    // =========================================================
    struct Wrist_IMU_Data { float acc[3]; float gyro[3]; };
    struct ARM_IMU_Data   { float acc[3]; float gyro[3]; };
    struct Magnet_Data    { float mag[3]; };
    struct Flex_Data      { float flex_adc; };

    // 패킷 메타 정보를 저장하는 구조체입니다 (타임스탬프, 시퀀스, 플래그).
    // Struct to store packet meta information (Timestamp, Seq, Flags).
    struct Packet_Info {
        uint32_t Timestamp;
        uint16_t Seq;
        uint8_t Flags;
    };

    // 필터링된 센서 데이터를 저장할 멤버 변수들입니다.
    // Member variables to store filtered sensor data.
    Wrist_IMU_Data _Wrist;
    ARM_IMU_Data   _ARM; 
    Magnet_Data    _Magnet;
    Flex_Data      _Flex;
    Packet_Info    _Info;

    // =========================================================
    // 2. EMA 필터 객체 배열 (EMA Filter Arrays)
    // =========================================================
    std::array<EMA_Filter, 3> _Wrist_EMA_Acc;
    std::array<EMA_Filter, 3> _Wrist_EMA_Gyro;

    std::array<EMA_Filter, 3> _ARM_EMA_Acc;
    std::array<EMA_Filter, 3> _ARM_EMA_Gyro;

    std::array<EMA_Filter, 3> _Magnet_EMA;

    // Flex 센서용 필터 (입력용과 출력용 2단계 구성).
    // Filters for Flex sensor (2-stage configuration: input and output).
    EMA_Filter _Flex_EMA;      // Raw data smoothing
    float _used_alpha_flex;

    std::atomic<bool> _mag_updated_latched{false};
    Wrist_IMU_Data _Wrist_at_mag{};

    // 파싱 후 정렬된 데이터를 임시로 저장할 구조체입니다.
    // Struct to temporarily store aligned data after parsing.
    struct SensorAligned {
        int16_t acc[3];
        int16_t gyro[3];
        int16_t acc_arm[3];
        int16_t gyro_arm[3];
        int16_t mag[3];
        uint16_t flex_adc;
        uint32_t t_us;
        uint16_t seq;
        uint8_t  flags;
    };

    // --- 토픽 구독 콜백 함수 (Subscriber Callbacks) ---
    // ELRS 패킷 데이터를 수신했을 때 호출되는 콜백 함수입니다.
    // Callback function called when ELRS packet data is received.
    void _callback_Packet(const std_msgs::msg::UInt8MultiArray::SharedPtr msg);

    // 페이로드 바이트 배열을 파싱하여 구조체에 담는 함수입니다.
    // Function to parse payload byte array into the struct.
    static bool _parse_payload(const uint8_t* payload, size_t payload_len, SensorAligned& out);

    // CRC16 체크섬을 계산하는 헬퍼 함수입니다.
    // Helper function to calculate CRC16 checksum.
    uint16_t _calculate_crc16(const uint8_t* packet, size_t length);

    // [신규] 적응형 베타 계산기 (Adaptive Beta Calculators)
    // 3가지 케이스를 미리 정의하여 런타임 연산을 줄입니다.
    // Define 3 cases in advance to reduce runtime computation.
    
    // 1. 손목 9축 모드 (e: 0.03~0.20 / Beta: 0.08~0.01)
    SmoothStep _Smooth_Beta_Wrist_9Axis;
    // 2. 손목 6축 모드 (e: 0.03~0.20 / Beta: 0.06~0.008)
    SmoothStep _Smooth_Beta_Wrist_6Axis;
    // 3. 팔 6축 모드 (e: 0.03~0.30 / Beta: 0.06~0.012)
    SmoothStep _Smooth_Beta_Arm_6Axis;

    // 베타 값을 부드럽게 만들기 위한 EMA 필터 (Gain 0.2)
    // EMA filter to smooth beta values (Gain 0.2)
    EMA_Filter _Quaternion_Beta_EMA;

    // --- 알고리즘 객체 (Algorithm Objects) ---
    // 매드윅 필터: 손목(9축)과 팔(6축)의 자세 추정을 위해 2개 생성합니다.
    // Madgwick Filter: Create two for attitude estimation of Wrist (9-axis) and Arm (6-axis).
    MadgwickFilter _Madgwick_ARM_Filter;
    MadgwickFilter _Madgwick_Wrist_Filter;

    // 데이터 매핑 및 좌표 변환을 위한 유틸리티 객체들입니다.
    // Utility objects for data mapping and coordinate transformation.
    Quaternion_to_Euler _to_euler;

    // MAG 품질 평가기(손목)
    Mag_Quality _MagQ_Wrist;

    static inline uint32_t monotonic_us() {
        using clock = std::chrono::steady_clock;
        const auto us = std::chrono::duration_cast<std::chrono::microseconds>(
            clock::now().time_since_epoch()).count();
        return static_cast<uint32_t>(us);
    }

    // MAG를 한 번이라도 받은 적 있는지(초기 상태에서 9축 호출 방지)
    bool _wrist_mag_ever_received = false;

    // 현재 9축(MAG 포함) 융합을 켜고 있는지(히스테리시스 상태)
    bool _wrist_mag_fusion_enabled = false;

    // 히스테리시스 임계값(튜닝 값)
    static constexpr float QMAG_ON  = 0.25f;  // 이 이상이면 9축 ON
    static constexpr float QMAG_OFF = 0.15f;  // 이 이하이면 6축 OFF로 전환

    void _Filter_Angle();
    void _Filter_Flex();

    // ROS2 통신을 위한 퍼블리셔, 타이머, 서브스크립션 객체들입니다.
    // Publisher, Timer, and Subscription objects for ROS2 communication.
    rclcpp::Publisher<hri_signal_pkg::msg::HRI_Attitude>::SharedPtr _pub_Attitude;
    rclcpp::TimerBase::SharedPtr _timer_Attitude;
    rclcpp::Publisher<hri_signal_pkg::msg::HRI_Flex>::SharedPtr _pub_Flex;
    rclcpp::TimerBase::SharedPtr _timer_Flex;
    rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr _sub_Packet;
};
#endif