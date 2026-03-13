#ifndef INTENT_STATE_GATE_NODE_HPP
#define INTENT_STATE_GATE_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

#include <"intent_state_pkg/msg/intent_setpoint">
#include <"intent_state_pkg/msg/state_engaged">
#include <"intent_state_pkg/msg/intent_gate_debug">

#include "hri_signal_pkg/msg/hri_filtered.hpp"
//wearable/filtered custom msg
#include "px4_msgs/msg/vehicle_status.hpp"
// /fmu/out/vehicle_status subscribe msg
//arming_state, nav_state, failsafe 등 기체 전반 status
#include "px4_msgs/msg/failsafe_flags.hpp"
// /fmu/out/failsafe_flags subscribe msg
//offboard_control_signal_lost, attitude_invalid 등 failsafe 입력 flags
#include "px4_msgs/msg/estimator_status_flags.hpp"
// /fmu/out/estimator_status_flags subscribe msg
//cs_tilt_align, cs_yaw_align 등 estimator status flags
#include "px4_msgs/msg/vehicle_control_mode.hpp"
// /fmu/out/vehicle_control_mode subscribe msg
//offboard/rates/attitude 제어 체인이 active 확인
#include "px4_msgs/msg/battery_status.hpp"
// /fmu/out/battery_status subscribe msg
//battery base return mode
#include "px4_msgs/msg/vehicle_odometry.hpp"
// /fmu/out/vehicle_odometry subscribe msg
//actual attitude(q), actual angular_velocity, velocity etc...

#include "px4_msgs/msg/offboard_control_mode.hpp"
// /fmu/in/offboard_control_mode publish msg
//offboard setpoint type select
#include "px4_msgs/msg/vehicle_rates_setpoint.hpp"
// /fmu/in/vehicle_rates_setpoint publish msg
// roll/pitch/yaw body-rate setpoint, thrust_body publishing


#include "Manual_Permission_Gate.hpp"

#include <chrono>
#include <cstdint>
#include <functional>

class Intent_Gate : public rclcpp::Node {
public:
    Intent_Gate();
private:

    static constexpr uint8_t FLAG_BUTTON_VALID = (1u << 3);
    // wearable filtered flags 안에서 button_valid를 뜻하는 비트 위치다.
    // 기존 코드와 호환성을 유지하기 위해 그대로 둔다.

    // -----------------------------
    // callbacks
    // -----------------------------
    void callback_filtered(const hri_signal_pkg::msg::HRIFiltered::SharedPtr msg);
    //filtered date callback 함수

    void callback_vehicle_status(const px4_msgs::msg::VehicleStatus::SharedPtr msg);
    // vehicle_status callback 함수
    // arming_state, nav_state 등 PX4 상태를 최신값으로 캐시한다.

    void callback_failsafe_flags(const px4_msgs::msg::FailsafeFlags::SharedPtr msg);
    // failsafe_flags callback 함수
    // offboard signal lost, attitude invalid 등 실패 플래그를 캐시한다.

    void callback_estimator_status_flags(
        const px4_msgs::msg::EstimatorStatusFlags::SharedPtr msg);
    // estimator_status_flags callback 함수
    // tilt/yaw align, estimator fault 계열을 캐시한다.

    void callback_vehicle_control_mode(
        const px4_msgs::msg::VehicleControlMode::SharedPtr msg);
    // vehicle_control_mode callback 함수
    // offboard_enabled, rates_enabled 등 실제 control chain 상태를 캐시한다.

    void callback_battery_status(const px4_msgs::msg::BatteryStatus::SharedPtr msg);
    // battery_status callback 함수
    // remaining, warning, unhealthy 같은 배터리 상태를 캐시한다.

    void callback_vehicle_odometry(const px4_msgs::msg::VehicleOdometry::SharedPtr msg);
    // vehicle_odometry callback 함수
    // actual q, angular_velocity, velocity를 캐시한다.

    // -----------------------------
    // Timer / publish
    // -----------------------------
    void timer_watchdog();
    // watchdog 타이머 콜백이다.
    // 현재 단계에서는 주기적으로 offboard heartbeat 틀을 퍼블리시하는 용도로도 사용할 수 있다.
    // hard stale 판정은 아직 여기서 하지 않는다.

    void publish_engaged();
    // 현재 engaged 상태를 퍼블리시하는 함수다.
    // 지금은 임시 std_msgs/Bool이지만, 이후 custom msg로 바뀐다.

    void publish_offboard_control_mode_heartbeat();
    // /fmu/in/offboard_control_mode를 퍼블리시하는 함수다.
    // body_rate 기반 offboard control을 사용할 것임을 PX4에 알린다.

    void publish_vehicle_rates_setpoint_stub();
    // /fmu/in/vehicle_rates_setpoint를 퍼블리시하는 함수다.
    // 지금은 틀만 만들고, 실제 값은 이후 RP/Yaw/Flex 계산 결과를 연결한다.

    uint64_t now_us() const;
    // 현재 ROS 시간을 microseconds 단위 uint64로 반환하는 헬퍼다.
    // px4 input message의 timestamp 필드 채울 때 사용한다.
    
    // -----------------------------
    // Wearable subscription
    // -----------------------------
    rclcpp::Subscription<hri_signal_pkg::msg::HRIFiltered>::SharedPtr _sub_filtered;

    // -----------------------------
    // PX4 subscriptions
    // -----------------------------
    rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr _sub_vehicle_status;
    // /fmu/out/vehicle_status 구독용 subscriber다.

    rclcpp::Subscription<px4_msgs::msg::FailsafeFlags>::SharedPtr _sub_failsafe_flags;
    // /fmu/out/failsafe_flags 구독용 subscriber다.

    rclcpp::Subscription<px4_msgs::msg::EstimatorStatusFlags>::SharedPtr _sub_estimator_status_flags;
    // /fmu/out/estimator_status_flags 구독용 subscriber다.

    rclcpp::Subscription<px4_msgs::msg::VehicleControlMode>::SharedPtr _sub_vehicle_control_mode;
    // /fmu/out/vehicle_control_mode 구독용 subscriber다.

    rclcpp::Subscription<px4_msgs::msg::BatteryStatus>::SharedPtr _sub_battery_status;
    // /fmu/out/battery_status 구독용 subscriber다.

    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr _sub_vehicle_odometry;
    // /fmu/out/vehicle_odometry 구독용 subscriber다.

    // -----------------------------
    // Publishers
    // -----------------------------
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr _pub_engaged;
    // 현재 engaged/disengaged를 임시로 퍼블리시하는 publisher다.

    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr _pub_offboard_control_mode;
    // /fmu/in/offboard_control_mode 퍼블리셔다.
    // PX4가 offboard body-rate stream을 기대하게 만드는 입력이다.

    rclcpp::Publisher<px4_msgs::msg::VehicleRatesSetpoint>::SharedPtr _pub_vehicle_rates_setpoint;
    // /fmu/in/vehicle_rates_setpoint 퍼블리셔다.
    // roll/pitch/yaw rate + thrust_body를 PX4에 보낸다.

    rclcpp::TimerBase::SharedPtr _timer_watchdog;
    // watchdog 타이머다.
    // 현재 단계에서는 주기적 publish 틀과 이후 stale/hard gate 확장을 위한 기반이다.

    // -----------------------------
    // Wearable latest cache
    // -----------------------------
    bool _filtered_received_once{false};
    // wearable filtered를 최소 1회라도 받았는지 표시한다.

    rclcpp::Time _last_filtered_rx_time;
    // 가장 최근 wearable filtered 수신 시각이다.
    // 이후 hard gate의 wearable freshness 계산에 사용한다.

    bool _button_on_off{false};
    // 가장 최근 wearable filtered의 버튼 on/off 캐시다.

    bool _button_valid{false};
    // 가장 최근 wearable filtered의 버튼 valid 캐시다.

    // -----------------------------
    // PX4 latest cache + freshness basis
    // -----------------------------
    bool _vehicle_status_received_once{false};
    // vehicle_status를 최소 1회라도 받았는지 표시한다.

    bool _failsafe_flags_received_once{false};
    // failsafe_flags를 최소 1회라도 받았는지 표시한다.

    bool _estimator_status_flags_received_once{false};
    // estimator_status_flags를 최소 1회라도 받았는지 표시한다.

    bool _vehicle_control_mode_received_once{false};
    // vehicle_control_mode를 최소 1회라도 받았는지 표시한다.

    bool _battery_status_received_once{false};
    // battery_status를 최소 1회라도 받았는지 표시한다.

    bool _vehicle_odometry_received_once{false};
    // vehicle_odometry를 최소 1회라도 받았는지 표시한다.

    rclcpp::Time _last_vehicle_status_rx_time;
    // 가장 최근 vehicle_status 수신 시각이다.

    rclcpp::Time _last_failsafe_flags_rx_time;
    // 가장 최근 failsafe_flags 수신 시각이다.

    rclcpp::Time _last_estimator_status_flags_rx_time;
    // 가장 최근 estimator_status_flags 수신 시각이다.

    rclcpp::Time _last_vehicle_control_mode_rx_time;
    // 가장 최근 vehicle_control_mode 수신 시각이다.

    rclcpp::Time _last_battery_status_rx_time;
    // 가장 최근 battery_status 수신 시각이다.

    rclcpp::Time _last_vehicle_odometry_rx_time;
    // 가장 최근 vehicle_odometry 수신 시각이다.

    px4_msgs::msg::VehicleStatus _vehicle_status{};
    // 가장 최근 vehicle_status 메시지 캐시다.

    px4_msgs::msg::FailsafeFlags _failsafe_flags{};
    // 가장 최근 failsafe_flags 메시지 캐시다.

    px4_msgs::msg::EstimatorStatusFlags _estimator_status_flags{};
    // 가장 최근 estimator_status_flags 메시지 캐시다.

    px4_msgs::msg::VehicleControlMode _vehicle_control_mode{};
    // 가장 최근 vehicle_control_mode 메시지 캐시다.

    px4_msgs::msg::BatteryStatus _battery_status{};
    // 가장 최근 battery_status 메시지 캐시다.

    px4_msgs::msg::VehicleOdometry _vehicle_odometry{};
    // 가장 최근 vehicle_odometry 메시지 캐시다.

    // -----------------------------
    // Parameters / state
    // -----------------------------
    double _offboard_heartbeat_period_sec{0.05};
    // offboard control mode heartbeat 퍼블리시 주기다.
    // 0.05초면 20Hz로, PX4 offboard 유지 조건(2Hz 초과)보다 충분히 높다. :contentReference[oaicite:5]{index=5}

    bool _engaged{false};
    // 최종 engaged 상태 캐시다.
    // 지금은 기존 인터페이스 유지용으로만 둔다.
};

#endif