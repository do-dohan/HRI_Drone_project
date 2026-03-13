#ifndef HARD_SAFETY_GATE.HPP
#define HARD_SAFETY_GATE.HPP

#include <cstdint>

class Hard_Safety{
public:
    struct Params{
    //Hard gate 동작 parameter struct
    //Parameter 수정을 위한 struct
    float hardgate_persist_threshold_sec{0.30f};
    //Hardgate_dynamic_event의 지속 시간에 따른 hard fail 승격 기준 시간

    bool require_yaw_align{false};
    //Yaw alignment에 따른 hard requirement로의 채택 여부 추후 yaw axis add 이후

    bool require_odometry_freshness{true};
    //Vehicle_odometry freshness를 hard requirement로의 채택 여부
    
    bool treat_battery_critical_as_fail{false};
    //Battery critical status를 hard requirement로의 채택 여부/추후 add

    bool treat_battery_unhealthy_as_fail{false};
    //Battery unhealthy status를 hard requirement로의 채택 여부/추후 add
    };

    struct Input{
        float dt_sec{0.0f};
        //evaluate 호출 간격
        //hardgate_persistence 계산에 사용

        bool wearable_freshness{false};
        //wearable input에 대해 timeout 계산값

        bool px4_vehicle_status_fresh{false};
        // vehicle_status 캐시가 fresh한지 나타낸다.

        bool px4_failsafe_flags_fresh{false};
        // failsafe_flags 캐시가 fresh한지 나타낸다.

        bool px4_estimator_status_fresh{false};
        // estimator_status_flags 캐시가 fresh한지 나타낸다.

        bool px4_control_mode_fresh{false};
        // vehicle_control_mode 캐시가 fresh한지 나타낸다.

        bool px4_odometry_fresh{false};
        // vehicle_odometry 캐시가 fresh한지 나타낸다.
        // require_odometry_fresh=true일 때만 hard requirement로 사용한다.

        bool vehicle_armed{false};
        // 현재 기체가 armed 상태인지 나타낸다.
        // 실제 비행 준비 완료 판단에 포함된다.

        bool preflight_checks_pass{false};
        // preflight checks를 통과했는지 나타낸다.
        // arm 가능 조건의 최소선으로 본다.

        bool control_offboard_enabled{false};
        // PX4 control chain에서 offboard control이 실제 활성인지 나타낸다.

        bool control_rates_enabled{false};
        // PX4 control chain에서 body-rate stabilization path가 활성인지 나타낸다.
        // 지금 구조는 VehicleRatesSetpoint 기반이므로 이 값이 중요하다.

        bool offboard_control_signal_lost{false};
        // PX4 failsafe_flags 기준으로 offboard 신호 손실이 감지됐는지 나타낸다.

        bool attitude_invalid{false};
        // attitude estimate invalid 여부다.

        bool angular_velocity_invalid{false};
        // angular velocity estimate invalid 여부다.

        bool local_velocity_invalid{false};
        // local velocity invalid 여부다.
        // 추후 z축 / actual velocity 기반 로직과 연결될 수 있다.

        bool estimator_tilt_aligned{false};
        // estimator tilt alignment 완료 여부다.

        bool estimator_yaw_aligned{false};
        // estimator yaw alignment 완료 여부다.
        // require_yaw_align=true일 때만 hard requirement로 사용한다.

        bool human_input_valid{true};
        // 인간 입력이 NaN / Inf / 범위이탈 / 스파이크 등 없이 유효한지 나타낸다.

        bool emergency_gesture{false};
        // 사용자 정의 emergency / kill gesture가 감지되었는지 나타낸다.

        bool battery_critical{false};
        // 배터리 critical 상태를 외부에서 해석해 넣는 bool이다.
        // 배터리 warning/remaining 등을 그대로 넣지 말고,
        // 노드 쪽에서 "정말 hard fail로 볼지" 해석한 뒤 전달한다.

        bool battery_unhealthy{false};
        // 배터리 unhealthy 상태를 외부에서 해석해 넣는 bool이다.

        bool rp_initialized{false};
        // RP 계산 클래스가 q_rel0 초기화 / baseline 준비를 마쳤는지 나타낸다.
        // 현재 단계에서는 hard readiness의 핵심 조건이다.

        bool require_flex_initialized{false};
        // flex_initialized를 hard readiness에 포함할지 여부다.
        // Flex 축을 실제 활성화하기 전까지는 false로 두는 편이 맞다.

        bool flex_initialized{false};
        // Flex 계산 클래스가 neutral/baseline 초기화를 마쳤는지 나타낸다.

        bool hardgate_dynamic_event{false};
        // "민감 동작 차단 + ENTRY reset/cap" 용 hardgate 이벤트 입력이다.
        // 예: 큰 팔 움직임, 특정 비정상 자세 변화, unsafe transient 등.
        // 첫 순간부터 전역 hard fail로 보지 않고 persistence를 본다.
    };

    struct Output{
        bool gate_pass{false};
        // 최종 hard safety pass 여부다.
        // true여야 permission gate 통과 후 ENGAGED 후보가 될 수 있다.

        bool safe_now{false};
        // 현재 hard fail 여부를 판단한다.

        bool link_ready{false};
        // PX4와 Wearable 사이의 링크가 ready 된 상태인지 판단한다.

        bool immediate_hard_fail{false};
        // 즉시 hard fail 항목이 하나라도 있었는지 나타낸다.
        // hardgate persistence escalation은 여기와 별도로 표시한다.

        bool hardgate_event{false};
        // 이번 tick에 hardgate raw event가 active인지 나타낸다.
        // 첫 순간부터 곧바로 전역 hard fail과 동일하지는 않다.

        bool persistent_escalation{false};
        // hardgate event가 threshold를 넘게 지속되어
        // 전역 hard fail로 승격됐는지 나타낸다.

        bool sensitive_ops_allowed{false};
        // q_rel0 commit, neutral commit 같은 민감 동작을 허용해도 되는지 나타낸다.
        // 문서상 hardgate first instant는 민감 동작을 막아야 하므로,
        // pass=true라도 hardgate_event=true면 false가 될 수 있다.

        bool entry_reset_or_cap_required{false};
        // 현재 hardgate 때문에 ENTRY dwell reset/cap 처리가 필요한지 나타낸다.

        bool ready_for_flight{false};
        // 비행 준비 완료 관점의 보조 플래그다.
        // pass=true이고 hardgate transient도 없을 때 true로 두는 편이 해석이 쉽다.

        uint32_t fail_mask{0u};
        // hard fail 이유를 bit mask로 담는다.

        uint8_t reason_primary{0u};
        // fail_mask 중 최종 판정을 대표하는 1순위 사유다.

        uint32_t readiness_mask{0u};
        // flight ready가 안 된 reason들을 담는다.

        float hardgate_persist_time_sec{0.0f};
        // 현재 hardgate가 연속 유지된 누적 시간(초)이다.
        // debug msg에 그대로 싣기 좋다.
    };

    enum FailMask : uint32_t
    // Hard gate 내부 사유를 bit mask로 정의한다.
    {
        FAIL_NONE                         = 0u,

        FAIL_WEARABLE_STALE               = 1u << 0,
        // wearable freshness timeout / stale.

        FAIL_PX4_VEHICLE_STATUS_STALE     = 1u << 1,
        // vehicle_status cache stale.

        FAIL_PX4_FAILSAFE_FLAGS_STALE     = 1u << 2,
        // failsafe_flags cache stale.

        FAIL_PX4_ESTIMATOR_STATUS_STALE   = 1u << 3,
        // estimator_status_flags cache stale.

        FAIL_PX4_CONTROL_MODE_STALE       = 1u << 4,
        // vehicle_control_mode cache stale.

        FAIL_PX4_ODOMETRY_STALE           = 1u << 5,
        // vehicle_odometry cache stale.

        FAIL_OFFBOARD_SIGNAL_LOST         = 1u << 6,
        // PX4 failsafe_flags says offboard control signal lost.

        FAIL_ATTITUDE_INVALID             = 1u << 7,
        // attitude estimate invalid.

        FAIL_ANGULAR_VELOCITY_INVALID     = 1u << 8,
        // angular velocity estimate invalid.

        FAIL_LOCAL_VELOCITY_INVALID       = 1u << 9,
        // local velocity invalid.

        FAIL_TILT_NOT_ALIGNED             = 1u << 10,
        // tilt alignment not completed.

        FAIL_YAW_NOT_ALIGNED              = 1u << 11,
        // yaw alignment not completed (only if require_yaw_align=true).

        FAIL_HUMAN_INPUT_INVALID          = 1u << 12,
        // human input invalid (NaN / Inf / spike / out-of-range).

        FAIL_EMERGENCY_GESTURE            = 1u << 13,
        // emergency / kill gesture active.

        FAIL_BATTERY_CRITICAL             = 1u << 14,
        // battery critical state.

        FAIL_BATTERY_UNHEALTHY            = 1u << 15,
        // battery unhealthy state.

        FAIL_HARDGATE_PERSIST_ESCALATION  = 1u << 16
        // hardgate event persisted too long and got escalated into global hard fail.
    };

    enum ReasonPrimary : uint8_t
    // 대표 사유 enum이다.
    // fail_mask가 여러 개 켜져도 가장 중요한 한 개를 고른다.
    {
        PRIMARY_NONE = 0,

        PRIMARY_EMERGENCY_GESTURE = 1,
        // 비상 종료가 최우선이다.

        PRIMARY_OFFBOARD_SIGNAL_LOST = 2,
        // 외부 제어 링크 상실.

        PRIMARY_OFFBOARD_MODE_DISABLED = 3,
        // PX4가 offboard chain을 실제로 안 쓰고 있음.

        PRIMARY_RATES_MODE_DISABLED = 4,
        // body-rate path가 활성화되지 않음.

        PRIMARY_VEHICLE_NOT_ARMED = 5,
        // armed 상태가 아님.

        PRIMARY_PREFLIGHT_NOT_PASSED = 6,
        // preflight checks 미통과.

        PRIMARY_ATTITUDE_INVALID = 7,
        // attitude invalid.

        PRIMARY_ANGULAR_VELOCITY_INVALID = 8,
        // angular velocity invalid.

        PRIMARY_LOCAL_VELOCITY_INVALID = 9,
        // local velocity invalid.

        PRIMARY_TILT_NOT_ALIGNED = 10,
        // tilt alignment 미완료.

        PRIMARY_YAW_NOT_ALIGNED = 11,
        // yaw alignment 미완료.

        PRIMARY_WEARABLE_STALE = 12,
        // wearable stale.

        PRIMARY_PX4_TOPIC_STALE = 13,
        // PX4 관련 토픽 stale.

        PRIMARY_HUMAN_INPUT_INVALID = 14,
        // human input invalid.

        PRIMARY_BATTERY_CRITICAL = 15,
        // battery critical.

        PRIMARY_BATTERY_UNHEALTHY = 16,
        // battery unhealthy.

        PRIMARY_RP_NOT_INITIALIZED = 17,
        // RP init 미완료.

        PRIMARY_FLEX_NOT_INITIALIZED = 18,
        // Flex init 미완료.

        PRIMARY_HARDGATE_PERSIST_ESCALATION = 19,
        // hardgate persist escalated.

        PRIMARY_QUALIFIED_PASS = 100
        // hard safety를 모두 통과함.
    };

    enum ReadinessMask : uint32_t
{
    READY_NONE                     = 0u,

    READY_WEARABLE_LINK_NOT_READY  = 1u << 0,
    // Wearable 입력 link가 prepare 되지 않음

    READY_PX4_LINK_NOT_READY     = 1u << 1,
    // PX4 입력 link가 prepare 되지 않음

    READY_PREFLIGHT_NOT_PASSED     = 1u << 2,
    // PX4 preflight checks pass 되지 않음

//    READY_OFFBOARD_NOT_ENABLED     = 1u << 4,
    // PX4 control chain에서 offboard path가 active 되지 않음

//    READY_RATES_NOT_ENABLED        = 1u << 5,
    // PX4 control chain에서 body-rate path가 active 되지 않음

    READY_RP_NOT_INITIALIZED       = 1u << 6,
    // RP baseline initialize가 finish 되지 않음

    READY_FLEX_NOT_INITIALIZED     = 1u << 7
    // Flex baseline initialize가 finish 되지 않음

    };

    explicit Hard_Safety_Gate(const Params& param = Params{});
    //설정값을 받아 Hard_Gate 생성

    void set_params(const Params& param);
    //runtime 중 setting값 config 가능

    const Params& param() const;
    //현재 설정값 참조 return

    void reset();
    //internal persistence status reset
    //experiment reset시 호출

    Output evaluate(const Input& input);
    //현재 input 기준 hard safety gate evaluate
    
private:
    void update_hardgate_persistence(const Input& input);
    //hardgate_raw_event persistence를 누적/리셋

    uint32_t build_immediate_fail_mask(const Input& input) const;
    //persistence escalation을 제외한 immediate hard fail bit mask 생성

    uint8_t pick_primary_reason(uint32_t fail_mask) const;
    //fail_mask에서 primary reason을 고른다.

    uint32_t build_readiness_mask(const Input& input) const;
    //readiness에서의 readiness fail bit mask 생성

    Params _param{};
    //현재 hard gate의 setting 값

    float _hardgate_persist_time_sec{0.0f};
    //hardgate_raw_event의 연속 누적 시간 측정
};

#endif