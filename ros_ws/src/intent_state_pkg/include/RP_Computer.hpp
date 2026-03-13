#ifndef RP_COMPUTER_HPP
#define RP_COMPUTER_HPP

#include <cstdint>

class RP_Computer
//Roll/Pitch intent computer class
//input-q_rel/flags -> q_rel0 baseline compute-> output-Roll/Pitchrate setpoint
{
public:
    struct Params
    {
        float baseline_outer_bound_from_init_rad{0.10f};
        //initial baseline에서 baseline update시 acceptable한 swing 범위
        //기준 baseline에서 큰 오차는 허용 안 함

        float baseline_update_band_rad{0.03f};
        //baseline update시 움직임 허용 범위

        float baseline_commit_hold_sec{0.30f};
        //baseline update commit 허용 시간

        float baseline_ema_tau_sec{6.0f};
        //baseline update EMA time const
        //값이 클수록 baseline update slowly

        float baseline_ema_max_step_rad_s{0.0070f};
        //baseline EMA의 maximum radian
        //0.007rad/s = 0.40 deg/s => 1 deg/ 2.5 sec

        float deadzone_rad{0.05f};
        //swing 기반 입력에서 deadzone 처리할 swing

        float max_swing_rad{0.40f};
        //full_scale 입력으로 해석할 최대 swing

        float max_rate_rad_s{1.20f};
        //Roll/Pitch raw rate setpoint 최대값

        float slew_rate_rad_s2{8.00f};
        //raw rate 출력의 slew limit에 사용할 값
    };

    struct Input
    {   
        bool sensitive_ops_allowed{false};
        // hard gate first instant에서 false가 될 수 있는 민감 동작 허용 플래그
        // baseline initialize/update에서 이 값이 true시에만 허용

        bool hardgate_event{false};
        // hardgate first instant 발생 여부
        // first instant는 global hard fail은 아니어도 baseline update는 금지

        bool persistent_escalation{false};
        // hardgate persistence 누적으로 hard fail로 승격된 상태
        // baseline 관련 동작 금지

        bool motion_still_enough{false};
        // gyro_norm, arm/wrist motion 등에서 충분히 정적인가를 Gate Node에서 판단한 flag
        // baseline update hold/EMA 시작 전제조건

        bool accel_quality_ok{false};
        // 가속도 norm error나 정적성 측면에서 baseline update 가능한지 flag

        bool mag_quality_ok{true};
        // 자력계를 baseline gating에 사용시 quality flag

        bool q_rel_valid{false};
        //q_rel 입력의 유효성 판단

        float q_rel_w{1.0f};
        float q_rel_x{0.0f};
        float q_rel_y{0.0f};
        float q_rel_z{0.0f};
        //arm_wrist 상대자세 쿼터니안 성분

        float dt_sec{0.0f};
        //이전 update 호출 이후 경과 시간

        bool init_request{false};
        //Gate_Node에서 주는 baseline initializing flag

        bool init_collect_active{false};
        //현재 tick이 INITIALIZING COLLECTING 구간 안에 있는지 여부

        bool init_finalize_request{false};
        //collect time 중 collecting 이후 baseline 계산/확정 시도 flag

        bool baseline_update_allowed{false};
        //baseline update의 허용 여부
        //hard gate의 sensitive_ops_allowed와 현재 운영 상태로 판단

        bool output_enabled{false};
        //현재 RP raw 출력의 active 가능 여부
        //ENGAGED_ENTRY or ENGAGED_STEADY 상 true

        bool reset_request{false};
        //RP 내부 상태 reset 필요시 true
    };

    struct Output
    {
        bool rp_initialized{false};
        //q_rel0 baseline의 intializing 여부

        bool init_done_this_tick{false};
        //현재 tick에서 initial baseline latch 여부

        bool baseline_updated_this_tick{false};
        //현재 tick에서 baseline update 여부

        bool valid_output{false};
        //Roll/Pitch raw output이 신뢰 가능한지 여부

        float roll_rate_sp_raw{0.0f};
        float pitch_rate_sp_raw{0.0f};
        //gate/shaping 적용 전 raw RP의 rate setpoint

        float swing_lateral{0.0f};
        float swing_vertical{0.0f};
        //q_delta에서 추출한 x,y축 편차

        float q_delta_w{1.0f};
        float q_delta_x{0.0f};
        float q_delta_y{0.0f};
        float q_delta_z{0.0f};
        // q_delta 쿼터니언의 성분
    };

    explicit RP_Computer(const Params& param = Params{});
    //RP_Computer 생성 및 parameter initialize
    //default parameter 사용
    //외부 parameter 생성

    void reset();
    //내부 상태 강제 초기화

    Output update(const Input& input);

private:
    void reset_runtime_state();
    //내부 runtime 상태만 reset

    bool try_initialize_baseline(const Input& input, Output& output);
    //init_request가 input 되면 q_rel0를 초기 baseline으로 latch
    //initialize finish 이후 initialized 상태를 세우고 output.init_done_this_tick을 true

    bool try_update_baseline(const Input& input, Output& output);
    //baseline update 조건 충족시 q_rel0 update
    //baseline update시 output.baseline_updated_this_tick을 true

    void compute_q_delta(const Input& input, Output& output) const;
    //현재 q_rel과 q_rel0를 이용해 q_delta compute

    void compute_swing_from_q_delta(Output& output) const;
    //q_delta를 swing 기반 편차로 compute

    void compute_raw_rate_from_swing(const Input& input, Output& output);
    //swing를 deadzone/gain/slew를 거쳐 Roll/Pitch 변환

    Params _param{};

    bool _initialized{false};
    //q_rel0 baseline의 latch 여부

    bool _init_collecting{false};
    //initialize collecting 중인지 여부

    float _init_elapsed_sec{0.0f};
    //initialize window 누적 time

    uint32_t _init_sample_count{0};
    //유효 sample 개수

    float _init_q_sum_w{0.0f};
    float _init_q_sum_x{0.0f};
    float _init_q_sum_y{0.0f};
    float _init_q_sum_z{0.0f};
    // quaternion 평균용 누적합

    float _init_swing_lat_sum{0.0f};
    float _init_swing_ver_sum{0.0f};
    float _init_swing_lat_sq_sum{0.0f};
    float _init_swing_ver_sq_sum{0.0f};
    //초기 표준편차 추정용 누적합

    float _q_rel_init_w{1.0f};
    float _q_rel_init_x{0.0f};
    float _q_rel_init_y{0.0f};
    float _q_rel_init_z{0.0f};
    //initial baseline의 q_rel 성분이다.

    float _q_rel0_w{1.0f};
    float _q_rel0_x{0.0f};
    float _q_rel0_y{0.0f};
    float _q_rel0_z{0.0f};
    // baseline 상대자세 q_rel0의 성분이다.

    float _roll_rate_prev{0.0f};
    float _pitch_rate_prev{0.0f};
    // 이전 프레임 raw Roll/Pitch rate 출력이다.

    float _baseline_hold_time_sec{0.0f};
    //baseline update 조건의 연속 유지 시간
};

#endif