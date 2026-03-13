#include "Hard_Safety_Gate.hpp"

#include <cmath>

Hard_Safety::Hard_Safety(const Params& param): _param(param)
// 생성자에서 parameter를 private member로 초기화
{
    _hardgate_persist_time_sec = 0.0f;
    // persistence 누적 시간은 0부터 시작한다.
}

void Hard_Safety::set_param(const Params& param)
{
    _param = param;
    //새로운 parameter 값으로 교체
}

const Hard_Safety::Params& Hard_Safety::param() const
{
    return _param;
    //현재 parameter 값을 const reference로 반환
}

void Hard_Safety::reset()
{
    _hardgate_persist_time_sec = 0.0f;
    //hardgate persistence의 누적 시간 초기화
}

void Hard_Safety::update_hardgate_persistence(const Input& input)
{
    float dt_sec = input.dt_sec;
    //input dt를 local 변수로 복사

    if (!std::isfinite(dt_sec) || dt_sec < 0.0f) {
        dt_sec = 0.0f;
    }

    if (input.hardgate_dynamic_event) {
        _hardgate_persist_time_sec += dt_sec;
        //hardgate가 active할 경우 누적 time을 더한다.
    } else {
        _hardgate_persist_time_sec = 0.0f;
        //hardgate off시 persistence를 0으로 reset
    }
}

uint32_t Hard_Safety::build_immediate_fail_mask(const Input& input) const
{
    uint32_t fail_mask = FAIL_NONE;
    // 즉시 hard fail bit mask를 0부터 시작한다.

    if (!input.wearable_fresh) {
        fail_mask |= FAIL_WEARABLE_STALE;
        // wearable 입력 stale이면 즉시 hard fail이다.
    }

    if (!input.px4_vehicle_status_fresh) {
        fail_mask |= FAIL_PX4_VEHICLE_STATUS_STALE;
        // vehicle_status stale이면 PX4 상태를 신뢰할 수 없다.
    }

    if (!input.px4_failsafe_flags_fresh) {
        fail_mask |= FAIL_PX4_FAILSAFE_FLAGS_STALE;
        // failsafe_flags stale이면 중요한 failsafe 입력을 신뢰할 수 없다.
    }

    if (!input.px4_estimator_status_fresh) {
        fail_mask |= FAIL_PX4_ESTIMATOR_STATUS_STALE;
        // estimator_status_flags stale이면 EKF health를 신뢰할 수 없다.
    }

    if (!input.px4_control_mode_fresh) {
        fail_mask |= FAIL_PX4_CONTROL_MODE_STALE;
        // vehicle_control_mode stale이면 실제 control chainput 상태를 신뢰할 수 없다.
    }

    if (_param.require_odometry_fresh && !input.px4_odometry_fresh) {
        fail_mask |= FAIL_PX4_ODOMETRY_STALE;
        // 설정상 odometry freshness를 요구하는데 stale이면 hard fail이다.
    }

    if (input.offboard_control_signal_lost) {
        fail_mask |= FAIL_OFFBOARD_SIGNAL_LOST;
        // PX4 failsafe_flags에서 offboard signal loss가 감지되면 즉시 hard fail이다.
    }

    if (input.attitude_invalid) {
        fail_mask |= FAIL_ATTITUDE_INVALID;
        // attitude estimate invalid면 제어 허가를 유지하면 안 된다.
    }

    if (input.angular_velocity_invalid) {
        fail_mask |= FAIL_ANGULAR_VELOCITY_INVALID;
        // angular velocity estimate invalid도 hard fail이다.
    }

    if (input.local_velocity_invalid) {
        fail_mask |= FAIL_LOCAL_VELOCITY_INVALID;
        // local velocity invalid는 z축/feedback 연동 시 문제를 일으킬 수 있으므로
        // 보수적으로 hard fail로 둔다.
    }

    if (!input.estimator_tilt_aligned) {
        fail_mask |= FAIL_TILT_NOT_ALIGNED;
        // tilt alignment가 완료되지 않으면 자세 추정 신뢰도가 부족하다.
    }

    if (_param.require_yaw_align && !input.estimator_yaw_aligned) {
        fail_mask |= FAIL_YAW_NOT_ALIGNED;
        // yaw alignment를 요구하는 설정에서 미완료면 hard fail이다.
    }

    if (!input.human_input_valid) {
        fail_mask |= FAIL_HUMAN_INPUT_INVALID;
        // human input이 NaN/Inf/범위이탈/스파이크 등으로 invalid면 hard fail이다.
    }

    if (input.emergency_gesture) {
        fail_mask |= FAIL_EMERGENCY_GESTURE;
        // emergency / kill gesture는 최우선 hard fail이다.
    }

    if (_param.treat_battery_critical_as_fail && input.battery_critical) {
        fail_mask |= FAIL_BATTERY_CRITICAL;
        // 설정상 배터리 critical을 hard fail로 취급하면 bit를 세운다.
    }

    if (_param.treat_battery_unhealthy_as_fail && input.battery_unhealthy) {
        fail_mask |= FAIL_BATTERY_UNHEALTHY;
        // 설정상 배터리 unhealthy도 hard fail로 취급하면 bit를 세운다.
    }

    return fail_mask;
    // persistence escalation을 제외한 즉시 hard fail mask를 반환한다.
}

uint8_t Hard_Safety::pick_primary_reason(uint32_t fail_mask) const
{
    // 아래 순서는 "대표 사유 우선순위"다.
    // 동시에 여러 bit가 켜져 있어도 가장 중요한 하나를 선택한다.

    if (fail_mask & FAIL_EMERGENCY_GESTURE) {
        return PRIMARY_EMERGENCY_GESTURE;
        // emergency gesture가 최우선이다.
    }

    if (fail_mask & FAIL_OFFBOARD_SIGNAL_LOST) {
        return PRIMARY_OFFBOARD_SIGNAL_LOST;
        // offboard 링크 상실은 즉시 대표 사유가 된다.
    }

    if (fail_mask & FAIL_ATTITUDE_INVALID) {
        return PRIMARY_ATTITUDE_INVALID;
        // attitude invalid는 강한 estimator fail이다.
    }

    if (fail_mask & FAIL_ANGULAR_VELOCITY_INVALID) {
        return PRIMARY_ANGULAR_VELOCITY_INVALID;
        // angular velocity invalid도 강한 estimator fail이다.
    }

    if (fail_mask & FAIL_LOCAL_VELOCITY_INVALID) {
        return PRIMARY_LOCAL_VELOCITY_INVALID;
        // local velocity invalid가 있으면 대표 사유로 반영한다.
    }

    if (fail_mask & FAIL_TILT_NOT_ALIGNED) {
        return PRIMARY_TILT_NOT_ALIGNED;
        // tilt alignment 미완료를 대표 사유로 반영한다.
    }

    if (fail_mask & FAIL_YAW_NOT_ALIGNED) {
        return PRIMARY_YAW_NOT_ALIGNED;
        // yaw alignment 미완료를 대표 사유로 반영한다.
    }

    if (fail_mask & FAIL_WEARABLE_STALE) {
        return PRIMARY_WEARABLE_STALE;
        // wearable freshness fail은 대표 사유 후보다.
    }

    if (fail_mask & (FAIL_PX4_VEHICLE_STATUS_STALE |
                     FAIL_PX4_FAILSAFE_FLAGS_STALE |
                     FAIL_PX4_ESTIMATOR_STATUS_STALE |
                     FAIL_PX4_CONTROL_MODE_STALE |
                     FAIL_PX4_ODOMETRY_STALE)) {
        return PRIMARY_PX4_TOPIC_STALE;
        // PX4 관련 stale들은 하나로 묶어 대표 사유로 반환한다.
    }

    if (fail_mask & FAIL_HUMAN_INPUT_INVALID) {
        return PRIMARY_HUMAN_INPUT_INVALID;
        // human input invalid를 대표 사유로 반영한다.
    }

    if (fail_mask & FAIL_BATTERY_CRITICAL) {
        return PRIMARY_BATTERY_CRITICAL;
        // battery critical을 대표 사유로 반영한다.
    }

    if (fail_mask & FAIL_BATTERY_UNHEALTHY) {
        return PRIMARY_BATTERY_UNHEALTHY;
        // battery unhealthy를 대표 사유로 반영한다.
    }

    if (fail_mask & FAIL_HARDGATE_PERSIST_ESCALATION) {
        return PRIMARY_HARDGATE_PERSIST_ESCALATION;
        // hardgate persistence escalation을 대표 사유로 반영한다.
    }

    return PRIMARY_QUALIFIED_PASS;
    // fail_mask가 비어 있으면 hard safety를 통과한 것이다.
}

uint32_t Hard_Safety::build_readiness_mask(const Input& input) const
{
    uint32_t readiness_mask = READY_NONE;
    // readiness mask를 0부터 시작한다.
    // flight ready가 finish되었는가를 판단한다.

    if (!input.wearable_fresh) {
        readiness_mask |= READY_WEARABLE_LINK_NOT_READY;
        // wearable 링크가 stable하지 않음을 의미한다.
        // readiness에서 gate node의 LINK_WAIT 판단에서 사용한다.
    }

    if (!input.px4_vehicle_status_fresh ||
        !input.px4_failsafe_flags_fresh ||
        !input.px4_estimator_status_fresh ||
        !input.px4_control_mode_fresh ||
        (_param.require_odometry_fresh && !input.px4_odometry_fresh))
    {
        readiness_mask |= READY_PX4_LINK_NOT_READY;
        // PX4 primary status가 ready되지 않음을 의미한다.
        // gate node에서 위 bit를 이용해 LINK_WAIT 상태 판단에 사용한다.
    }

    if (!input.preflight_checks_pass) {
        readiness_mask |= READY_PREFLIGHT_NOT_PASSED;
        // preflight checks가 pass 되지 않으면 비행 시작 준비가 끝나지 않았다고 판단한다.
    }

//    if (!input.control_offboard_enabled) {
//        readiness_mask |= READY_OFFBOARD_NOT_ENABLED;
//        // PX4 offboard path가 아직 활성화되지 않았다면
//        // gate node에서 intent를 PX4에 전달할 준비가 끝난 것이 아니다.
//    }

//    if (!input.control_rates_enabled) {
//        readiness_mask |= READY_RATES_NOT_ENABLED;
//        // body-rate path가 아직 활성화되지 않았다면
//        // VehicleRatesSetpoint 기반 제어를 받을 준비가 안 된 것이다.
//    }

    if (!input.rp_initialized) {
        readiness_mask |= READY_RP_NOT_INITIALIZED;
        // RP baseline이 initialize 되지 않으면 제스처 기반 RP 제어를 시작할 수 없다.
    }

    if (input.require_flex_initialized && !input.flex_initialized) {
        readiness_mask |= READY_FLEX_NOT_INITIALIZED;
        // Flex baseline이 initialize 되지 않으면 제스처 기반 Flex 제어를 시작할 수 없다.
    }

    return readiness_mask;
    // readiness mask 반환.
}

Hard_Safety::Output Hard_Safety::evaluate(const Input& input)
{
    Output out{};
    //return할 Output struct를 기본값으로 만든다.

    const bool hardgate_event = input.hardgate_dynamic_event;
    //현재 tick에서 hardgate dynamic event가 들어왔는지를 local 변수로 저장
    
    update_hardgate_persistence(input);
    //hardgate_persistence 시간을 갱신한다.

    const bool persistent_escalation =
        hardgate_event &&
        (_hardgate_persist_time_sec >= _param.hardgate_persist_threshold_sec);
        //hardgate_dynamic event가 threshold 이상으로 continuse하게 유지될 경우 hard fail

    const uint32_t immediate_fail_mask = build_immediate_fail_mask(input);
    //immediate한 hard fail list를 계산한다.

    const bool immediate_hard_fail = (immediate_fail_mask != FAIL_NONE);
    //immediate hard fail가 존재할 경우 true로 처리한다.

    const uint32_t final_fail_mask =
        immediate_fail_mask |
        (persistent_escalation ? FAIL_HARDGATE_PERSIST_ESCALATION : 0u);
        //final fail mask를 추가한다.
    
    const bool final_hard_fail = (final_fail_mask != FAIL_NONE);
    //final_fail 값이 존재할 경우 true

    const uint32_t readiness_mask = build_readiness_mask(input);
    // readiness mask를 계산한다.
    // 이 값은 "위험"이 아니라 "아직 비행 시작 준비가 안 됨"을 의미한다.

    const bool readiness_ok = (readiness_mask == READY_NONE);
    // readiness bit가 하나도 없을 때만 true다.

    const bool link_ready =
        ((readiness_mask & (READY_WEARABLE_LINK_NOT_READY |
                            READY_PX4_LINK_NOT_READY)) == 0u);
    // 링크 준비 완료 여부를 계산한다.
    // wearable 링크와 PX4 핵심 상태 링크가 모두 준비되었을 때만 true다.
    // gate node는 이 값을 보고 LINK_WAIT 상태를 판단할 수 있다.

    out.gate_pass = !final_hard_fail;
    //fail none, 즉, hard fail bit가 없을 경우 true

    out.immediate_hard_fail = immediate_hard_fail;
    // immediate한 hard safety fail 여부 저장 
    // persistence escalation과 구분하여 debug에 사용

    out.hardgate_event = hardgate_event;
    // 현재 tick에서의 dynamic Hardgate status 저장
    // sensitive operation 차단과 ENTRY reset/cap에 사용

    out.persistent_escalation = persistent_escalation;
    // Hardgate가 threshold를 넘어 hard fail로 승격됐는지 저장
    // true면 이번 tick은 DISENGAGED에 사용

    out.sensitive_ops_allowed = (!final_hard_fail && !hardgate_event);
    // baseline update 등 민감한 동작 허용 여부에 사용
    // hard fail이 있으면 당연히 금지해야 하고,
    // hardgate 발생시 sensitive한 operation은 막아야 하므로 false

    out.entry_reset_or_cap_required = hardgate_event;
    // Hardgate가 active할 때 ENTRY dwell reset/cap 규칙을 적용 필요를 보임
    // first instant에서도 이 플래그는 바로 true가 될 수 있어야 한다.

    out.readiness_mask = readiness_mask;
    // 비행 시작 준비 미완료 사유 bit mask를 저장

    out.link_ready = link_ready;
    // wearable/PX4 링크 준비 완료 여부를 gate node에 전달
    // gate node는 이 flag를 보고 LINK_WAIT 상태를 쉽게 판단할 수 있다.

    out.ready_for_flight = (!final_hard_fail && readiness_ok && !hardgate_event);
    // 최종 비행 준비 완료 여부다.
    // 조건은
    // 1) hard fail이 없고
    // 2) readiness 항목이 모두 만족되고
    // 3) 현재 hardgate transient도 없을 때만 true다.
    // 즉 "이제 사용자의 arm 승인 요청을 받아도 되는 상태"를 의미한다.

    out.fail_mask = final_fail_mask;
    // final hard fail mask를 저장
    // immediate fail들과 hardgate persistence escalation이 모두 반영된 최종 결과

    out.reason_primary = pick_primary_reason(final_fail_mask);
    // 최종 fail mask에서 대표 사유를 하나 저장
    // 여러 bit가 동시에 켜져 있어도
    // 디버그/로그/상태 표시용으로 가장 우선순위가 높은 원인 하나를 선택한다.

    out.hardgate_persist_time_sec = _hardgate_persist_time_sec;
    // 현재 Hardgate 연속 유지 시간을 저장
    // 디버그 메시지와 실험 로그에서
    // "얼마나 오래 유지돼서 escalation 됐는지"를 복원할 때 필요하다.

    return out;
    // 현재 tick에서의 Hard safety 평가 결과를 return
}