#include "Intent_State_Gate_Node.hpp"

using namespace std::chrono_literals;
//20ms, 100ms 같은 duration literal을 위해 사용

Intent_Gate::Intent_Gate(): Node("Intent_State_Gate_Node")
//Node base class 생성
//Node name"Intent_State_Gate_Node"
{
    this->declare_parameter<double>("offboard_heartbeat_period_sec", 0.05);
    // OffboardControlMode heartbeat를 0.05주기로 보내도록 parameter 선언

    _offboard_heartbeat_period_sec = this->get_parameter("offboard_heartbeat_period_sec").as_double();
    //parameter 변수에 저장

    // ---------------------------------
    // Wearable subscriber
    // ---------------------------------
    _sub_filtered = this->create_subscription<hri_signal_pkg::msg::HRIFiltered>(
        "wearable/filtered",
        10,
        std::bind(&Intent_Gate::callback_filtered, this, std::placeholders::_1)
    );
    //"wearable/filtered" 토픽 subscribe
    //QoS depth 10, 메시지 타입 HRIFiltered

    // ---------------------------------
    // PX4 subscribers
    // ---------------------------------
    _sub_vehicle_status = this->create_subscription<px4_msgs::msg::VehicleStatus>(
        "/fmu/out/vehicle_status",
        10,
        std::bind(&Intent_Gate::callback_vehicle_status, this, std::placeholders::_1)
    );
    // PX4 commander 상태를 받는다.
    // arming_state, nav_state, failsafe 등을 hard gate 처리에 사용

    _sub_failsafe_flags = this->create_subscription<px4_msgs::msg::FailsafeFlags>(
        "/fmu/out/failsafe_flags",
        10,
        std::bind(&Intent_Gate::callback_failsafe_flags, this, std::placeholders::_1)
    );
    // PX4 failsafe 입력 플래그를 받는다.
    // offboard_control_signal_lost, attitude_invalid 등을 hard fail 후보로 사용

    _sub_estimator_status_flags =
        this->create_subscription<px4_msgs::msg::EstimatorStatusFlags>(
            "/fmu/out/estimator_status_flags",
            10,
            std::bind(&Intent_Gate::callback_estimator_status_flags, this, std::placeholders::_1)
        );
    // EKF/estimator 상태 플래그를 받는다.
    // cs_tilt_align, cs_yaw_align, bad heading/clipping 등을 hard/soft gate에 사용 가능

    _sub_vehicle_control_mode =
        this->create_subscription<px4_msgs::msg::VehicleControlMode>(
            "/fmu/out/vehicle_control_mode",
            10,
            std::bind(&Intent_Gate::callback_vehicle_control_mode, this, std::placeholders::_1)
        );
    // PX4 control chain 활성 상태를 받는다.
    // offboard_enabled, rates_enabled 같은 실제 제어 경로 active 여부 확인

    _sub_battery_status = this->create_subscription<px4_msgs::msg::BatteryStatus>(
        "/fmu/out/battery_status",
        10,
        std::bind(&Intent_Gate::callback_battery_status, this, std::placeholders::_1)
    );
    // 배터리 상태를 받는다.
    // battery warning/remaining 기반 사용자 추적 return 사용 가능

    _sub_vehicle_odometry = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
        "/fmu/out/vehicle_odometry",
        10,
        std::bind(&Intent_Gate::callback_vehicle_odometry, this, std::placeholders::_1)
    );
    // actual q, angular_velocity, velocity를 받는다.
    // actual rate 비교와 연구/디버그 용도로 사용

    // ---------------------------------
    // Publishers
    // ---------------------------------
    _pub_engaged = this->create_publisher<intent_state_pkg::msg::state_engaged>(
        "/intent_state/engaged",
        10
    );
    //현재 engaged/disengaged state를 publish
    //gate pass 여부도 포함

    _pub_offboard_control_mode =
        this->create_publisher<px4_msgs::msg::OffboardControlMode>(
            "/fmu/in/offboard_control_mode",
            10
        );
    // PX4에 body-rate offboard control mode heartbeat를 보내는 퍼블리셔다.

    _pub_vehicle_rates_setpoint =
        this->create_publisher<px4_msgs::msg::VehicleRatesSetpoint>(
            "/fmu/in/vehicle_rates_setpoint",
            10
        );
    // PX4에 body-rate setpoint와 thrust를 보내는 퍼블리셔다. 

    // ---------------------------------
    // Timer
    // ---------------------------------
    const auto watchdog_period_ns =
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(_offboard_heartbeat_period_sec)
        );
    // parameter로 받은 초 단위 주기를 ROS wall timer가 받는 나노초 duration으로 변환

    _timer_watchdog = this->create_wall_timer(
        watchdog_period_ns,
        std::bind(&Intent_Gate::timer_watchdog, this)
    );
    // 주기 타이머를 생성한다.
    // 현재는 offboard heartbeat와 rates setpoint publish를 돌리는 데 사용

    RCLCPP_INFO(this->get_logger(), "Intent_Gate initialized. wearable initialized active...");
    //Node Initialize 완료 log 출력
}

uint64_t Intent_Gate::now_us() const
//현재 ROS time을 nanoseconds 단위 uint로 반환
{
    const int64_t now_ns = this->now().nanoseconds();
    //현재 ROS time을 nanoseconds 단위 int로 반환

    return (now_ns > 0) ? static_cast<uint64_t>(now_ns / 1000LL) : 0ULL;
    //음수 방지 보수적 처리, 양수시 microseconds로 변환하여 uint64로 반환
}

void Intent_Gate::callback_filtered(
    const hri_signal_pkg::msg::HRIFiltered::SharedPtr msg)
    {
        _filtered_recieved_once = true;
        //filtered data 1회 이상 subscribe
        //freshness 판단 시 필요

        _last_filtered_rx_time = this->now();
        //msg 수신시 현재 Ros time 기록
        //hard gate wearable freshness 계산시 판단 기준

        _button_on_off = msg->button_on_off;
        //filtered data에서 button 값 저장

        _button_valid = (msg->flags & FLAG_BUTTON_VALID) != 0;
        //flags 안에 button_valid를 통해 비트 해석
        //프레임의 버튼 값 유효한지 bool로 변환
    }

    void Intent_Gate::callback_vehicle_status(
    const px4_msgs::msg::VehicleStatus::SharedPtr msg)
{
    _vehicle_status_received_once = true;
    // vehicle_status를 최소 1회 수신했음을 기록한다.

    _last_vehicle_status_rx_time = this->now();
    // 가장 최근 vehicle_status 수신 시각을 저장한다.

    _vehicle_status = *msg;
    // 메시지 전체를 최신 캐시로 복사해 둔다.
    // 이후 hard gate에서 arming_state, nav_state 등을 읽기 쉽게 하기 위함이다.
}

void Intent_Gate::callback_failsafe_flags(
    const px4_msgs::msg::FailsafeFlags::SharedPtr msg)
{
    _failsafe_flags_received_once = true;
    // failsafe_flags를 최소 1회 수신했음을 기록한다.

    _last_failsafe_flags_rx_time = this->now();
    // 가장 최근 failsafe_flags 수신 시각을 저장한다.

    _failsafe_flags = *msg;
    // 메시지 전체를 최신 캐시로 복사한다.
    // 추후 offboard_control_signal_lost, attitude_invalid 등을 hard gate에서 읽는다.
}

void Intent_Gate::callback_estimator_status_flags(
    const px4_msgs::msg::EstimatorStatusFlags::SharedPtr msg)
{
    _estimator_status_flags_received_once = true;
    // estimator_status_flags를 최소 1회 수신했음을 기록한다.

    _last_estimator_status_flags_rx_time = this->now();
    // 가장 최근 estimator_status_flags 수신 시각을 저장한다.

    _estimator_status_flags = *msg;
    // estimator 상태 플래그 전체를 최신 캐시로 복사한다.
    // cs_tilt_align, cs_yaw_align 등을 이후 hard/soft에서 읽는다.
}

void Intent_Gate::callback_vehicle_control_mode(
    const px4_msgs::msg::VehicleControlMode::SharedPtr msg)
{
    _vehicle_control_mode_received_once = true;
    // vehicle_control_mode를 최소 1회 수신했음을 기록한다.

    _last_vehicle_control_mode_rx_time = this->now();
    // 가장 최근 vehicle_control_mode 수신 시각을 저장한다.

    _vehicle_control_mode = *msg;
    // control mode 전체를 최신 캐시로 복사한다.
    // flag_control_offboard_enabled, flag_control_rates_enabled 등을 추후 사용한다.
}

void Intent_Gate::callback_battery_status(
    const px4_msgs::msg::BatteryStatus::SharedPtr msg)
{
    _battery_status_received_once = true;
    // battery_status를 최소 1회 수신했음을 기록한다.

    _last_battery_status_rx_time = this->now();
    // 가장 최근 battery_status 수신 시각을 저장한다.

    _battery_status = *msg;
    // 배터리 상태 전체를 최신 캐시로 복사한다.
    // 이후 remaining, warning, unhealthy 등을 읽을 수 있다.
}

void Intent_Gate::callback_vehicle_odometry(
    const px4_msgs::msg::VehicleOdometry::SharedPtr msg)
{
    _vehicle_odometry_received_once = true;
    // vehicle_odometry를 최소 1회 수신했음을 기록한다.

    _last_vehicle_odometry_rx_time = this->now();
    // 가장 최근 vehicle_odometry 수신 시각을 저장한다.

    _vehicle_odometry = *msg;
    // odometry 전체를 최신 캐시로 복사한다.
    // q, angular_velocity, velocity는 이후 actual state/디버그에 사용한다.
}

void Intent_Gate::timer_watchdog()
{
    publish_offboard_control_mode_heartbeat();
    // body_rate 기반 offboard heartbeat를 주기적으로 퍼블리시한다.
    // PX4 offboard 유지에는 지속적인 offboard_control_mode / setpoint stream이 필요하다. :contentReference[oaicite:6]{index=6}

    publish_vehicle_rates_setpoint_stub();
    // 실제 RP/Yaw/Flex 계산 값이 아직 연결되지 않았으므로,
    // 지금은 안전한 0 setpoint stub만 보낸다.

    publish_engaged();
    // 현재 engaged 상태를 계속 퍼블리시한다.
    // 이후 custom state/debug/setpoint 토픽으로 대체될 수 있다.
}

void Intent_Gate::publish_engaged()
{
    std_msgs::msg::Bool msg;
    // 퍼블리시할 Bool 메시지 객체를 만든다.

    msg.data = _engaged;
    // 현재 engaged 상태를 메시지에 담는다.

    _pub_engaged->publish(msg);
    // engaged 상태를 퍼블리시한다.
}

void Intent_Gate::publish_offboard_control_mode_heartbeat()
{
    px4_msgs::msg::OffboardControlMode msg;
    // OffboardControlMode 메시지 객체를 만든다.

    msg.timestamp = now_us();
    // PX4 입력 메시지 timestamp 필드를 현재 시간(us)로 채운다.

    msg.position = false;
    // position setpoint 기반 offboard는 사용하지 않음을 명시한다.

    msg.velocity = false;
    // velocity setpoint 기반 offboard도 사용하지 않음을 명시한다.

    msg.acceleration = false;
    // acceleration 기반 offboard도 사용하지 않는다.

    msg.attitude = false;
    // attitude quaternion setpoint 기반 offboard도 사용하지 않는다.

    msg.body_rate = true;
    // 우리가 사용할 offboard 타입은 body-rate임을 명시한다.
    // 즉 roll/pitch/yaw rate setpoint를 보낼 계획이라는 뜻이다. :contentReference[oaicite:7]{index=7}

    msg.thrust_and_torque = false;
    // thrust_and_torque 직접 모드는 지금 사용하지 않는다.

    msg.direct_actuator = false;
    // direct_actuator 모드도 사용하지 않는다.

    _pub_offboard_control_mode->publish(msg);
    // PX4에 offboard control mode heartbeat를 보낸다.
}

void Intent_Gate::publish_vehicle_rates_setpoint_stub()
{
    px4_msgs::msg::VehicleRatesSetpoint msg;
    // VehicleRatesSetpoint 메시지 객체를 만든다.

    msg.timestamp = now_us();
    // PX4 입력 메시지 timestamp를 현재 시간(us)로 채운다.

    msg.roll = 0.0f;
    // 현재는 RP 계산 노드가 연결되지 않았으므로 roll rate setpoint를 0으로 둔다.

    msg.pitch = 0.0f;
    // 현재는 pitch rate setpoint도 0으로 둔다.

    msg.yaw = 0.0f;
    // 현재는 yaw rate setpoint도 0으로 둔다.
    // 현 로직 freeze 문맥에서도 yaw는 아직 핵심 경로가 아니다. :contentReference[oaicite:8]{index=8}

    msg.thrust_body[0] = 0.0f;
    // FRD body frame x축 thrust를 0으로 둔다.

    msg.thrust_body[1] = 0.0f;
    // FRD body frame y축 thrust를 0으로 둔다.

    msg.thrust_body[2] = 0.0f;
    // FRD body frame z축 thrust를 0으로 둔다.
    // 이후 Z/Flex 해석 결과를 연결하면 이 부분이 바뀔 수 있다.

    msg.reset_integral = false;
    // 현재는 PX4 내부 rate controller 적분기를 리셋하지 않는다.

    _pub_vehicle_rates_setpoint->publish(msg);
    // 0 body-rate / 0 thrust stub를 PX4에 퍼블리시한다.
}

void Intent_Gate::timer_watchdog()
{
    evaluate_gate();
    //callback이 안 들어올 동안에도 freshness timeout state 계산 필요(timer에서도 함수 호출)

    publish_engaged();
    //stale timeout 계산 후 engaged/disengaged publish
}

void Intent_Gate::evaluate_gate()
{
    bool filtered_freshness = false;
    //filtered data의 freshness 검증 변수
    //stale timeout 계산을 통해 true/false

    if (_filtered_recieved_once) {
        //filtered data를 1회 이상 받은 경우에만 계산
        //추후 Drone과 wearable 장비의 initialized 이후 시작

        const double dt = (this->now() - _last_filtered_rx_time).second();
        //filtered data 수신 이후 경과 시간 계산

        filtered_freshness = (dt <= _filtered_timeout_sec);
        //경과 시간이 timeout 이하면 fresh(true), 초과시 stale(false)
    }

    const bool button_engaged =
        _button_on_off &&
        _button_valid &&
        filtered_freshness;
        //1차 게이트(Button, stale gate)
        //체감 latency를 줄이기 위해 Button on/off시
        //즉시 engaged/disengaged

    if (button_engaged != _engaged) {
        //계산된 state가 이전 state와 다를 경우에만 상태값 갱신

        _engaged = button_engaged;
        //게이트 상태 갱신

        RCLCPP_INFO(
            this->get_logger(),
            "Gate state changed: %s (button=%d, valid=%d, fresh=%d dt=%f)",
            engaged_ ? "ENGAGED" : "DISENGAGED",
            static_cast<int>(_button_on_off),
            static_cast<int>(_button_valid),
            static_cast<int>(filtered_freshness)
            static_cast<double>(dt)
        );
        //Button on/off, Button Flag, filtered freshness, dt(경과 시간) logging
    }
}

void Intent_Gate::publish_engaged()
{
    std_msgs::msg::Bool msg;
    //msg 객체 생성

    msg.data = _engaged;
    //현재 gate 상태 저장

    _pub_engaged ->publish(msg);
}