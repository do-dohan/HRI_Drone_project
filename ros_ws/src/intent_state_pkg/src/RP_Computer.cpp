#inclide "RP_Computer.hpp"

#include <algorithm>
#include <cmath>

namespace
{
constexpr float k_Quat_Norm_Eps = 1.0e-6f;
//quaternion norm이 k_Quat_Norm_Eps보다 작을 경우 invalid 처리

constexpr float k_Angle_Eps = 1.0e-6f;
//angle의 특이구간 보호용 epsilon

inline bool is_finite_float(const float v)
{
    return std::isfinite(v);
    //float이 NaN/Inf가 아닌지 check
}

inline float clamp_f(const float v, const float lo, const float hi){
    return std::max(lo, std::min(v, hi));
    //float clamp helper
}

inline float sign_f_no_zero(const float v)
{
    return (v >= 0.0f) ? 1.0f : -1.0f;
    //zero를 positive 쪽으로 보내는 sign helper
}

bool quat_components_finite(const float w, const float x, const float y, const float z)
{
    return is_finite_float(w) && is_finite_float(x) && is_finite_float(y) && is_finite_float(z)   
    //quaternion의 네 성분의 finite 여부 확인
}

bool nomalize_quat(const float w, const float x, const float y, const float z,
                   float& norm_w, float& norm_x, float& norm_y, float& norm_z)
{
    //quaternion unit quaternion nomalize helper

    if (!quat_components_finite(w, x, y, z)) {
        return false;
        //quaternion 성분 중 하나라도 NaN/Inf라면 false
    }

    const float n2 = w*w + x*x + y*y + z*z;
    //norm 제곱 계산

    if (!is_finite_float(n2) || n2 < k_Quat_Norm_Eps) {
        return false;
        //norm이 너무 작거나 수치적으로 깨지면 false
    }

    const float_inv_n = 1.0f / std::sqrt(n2);
    //norm의 역수를 생성

    norm_w *= inv_n;
    norm_x *= inv_n;
    norm_y *= inv_n;
    norm_z *= inv_n;
    //quaternion 성분들 nomalize

    return quat_components_finite(norm_w, norm_x, norm_y, norm_z);
}

void quat_conjugate(const float w, const float x, const float y, const float z,
                    float& out_w, float& out_x, float& out_y, float& out_z)
{
    //quaternion conjugate를 compute
    out_w = w;
    out_x = -x;
    out_y = -y;
    out_z = -z;
}

void quat_inverse_unit(const float w, const float x, const float y, const float z,
                    float& out_w, float& out_x, float& out_y, float& out_z)
{
    //unit quaternion의 inverse는 conjugate와 동일
    quat_conjugate(w, x, y, z, out_w, out_x, out_y, out_z);
}

void quat_multiply(const float aw, const float ax, const float ay, const float az,
                   const float bw, const float bx, const float by, const float bz,
                   float& out_w, float& out_x, float& out_y, float& out_z)
{
    //Hamilton product
    out_w = aw*bw - ax*bx - ay*by - az*bz;
    out_x = aw*bx + ax*bw + ay*bz - az*by;
    out_y = aw*by - ax*bz + ay*bw + az*bx;
    out_z = aw*bz + ax*by - ay*bx + az*bw;
}

void quat_rotate_vec_unit(const float qw, const float qx, const float qy, const float qz,
                          const float vx, const float vy, const float vz,
                          float& out_x, float& out_y, float out_z)
{
    //unit quaternion으로 3D vector rotate
    //v_out = q * [0,v] * q^{-1} 를 구현한다.

    float iw = 0.0f;
    float ix = 0.0f;
    float iy = 0.0f;
    float iz = 0.0f;
    //중간 quaternion 성분 저장 변수

    quat_multiply(qw, qx, qy, qz,
                  0.0f, vx, vy, vz,
                  iw, ix, iy, iz);
    //q * [0,v] 를 먼저 계산한다.

    float cw = 0.0f;
    float cx = 0.0f;
    float cy = 0.0f;
    float cz = 0.0f;
    //q^{-1} 성분 저장 변수

    quat_inverse_unit(qw, qx, qy, qz, cw, cx, cy, cz);
    //unit quaternion inverse 생성

    float rw = 0.0f;
    //final scalar 성분

    quat_multiply(iw, ix, iy, iz,
                  cw, cx, cy, cz,
                  rw, out_x, out_y, out_z);
    //(q * [0,v]) * q^{-1}를 계산해 rotate vector return
}

bool extract_twist_about_local_x(const float qw, const float qx, const float qy, const float qz,
                                 const float ax, const float ay, const float az,
                                 float& twist_w, float& twist_x, float& twist_y, float& twist_z)
{
    const float vx = qx;
    const float vy = qy;
    const float vz = qz;
    //quaternion vector part

    const float dot = vx*ax + vy*ay + vz*az;
    //projection of v onto axis a

    twist_w = qw;
    twist_x = dot * ax;
    twist_y = dot * ay;
    twist_z = dot * az;
    //x_axis twist extraction

    if (!normalize_quat(twist_w, twist_x, twist_y, twist_z)){
        //특이구간 normalizing 실패시 identity twist로 fallback
        twist_w =  1.0f;
        twist_x =  0.0f;
        twist_y =  0.0f;
        twist_z =  0.0f;
        return false;
    }
    return true;
}

bool extract_swing_quat_from_total(const float qw, const float qx, const float qy, const float qz,
                                   float& swing_w, float& swing_x, float& swing_y, float& swing_z)
{
    //total rotate q_total은 swing * twist
    //q_swing = q_total ⊗ q_twist^{-1}로 swing 성분만 return

    float qn_w = qw;
    float qn_x = qx;
    float qn_y = qy;
    float qn_z = qz;
    //Input quaternion을 저장

    if(!normalize_quat(qn_w, qn_x, qn_y, qn_z)){
        return false;
    }

    float twist_w = 1.0f;
    float twist_x = 1.0f;
    float twist_y = 1.0f;
    float twist_z = 1.0f;
    //twist quaternion 성분

    float rx = 1.0f;
    float ry = 0.0f;
    float rz = 0.0f;
    //reference axis의 성분

    extract_twist_about_local_x(qn_w, qn_x, qn_y, qn_z,
                                rx, ry, rz,
                                twist_w, twist_x, twist_y, twist_z);
    //local x_aixs 기준 twist extraction

    float twist_inv_w = 1.0f;
    float twist_inv_x = 0.0f;
    float twist_inv_y = 0.0f;
    float twist_inv_z = 0.0f;
    //twist inverse의 성분

    quat_inverse_unit(twist_w, twist_x, twist_y, twist_z,
                      twist_inv_w, twist_inv_x, twist_inv_y, twist_inv_z);
    //q_twist^{-1} 생성

    quat_multiply(qn_w, qn_x, qn_y, qn_z,
                  twist_inv_w, twist_inv_x, twist_inv_y, twist_inv_z,
                  swing_w, swing_x, swing_y, swing_z);
    //q_swing = q_total ⊗ q_twist^{-1} 를 compute

    if (!normalize_quat(swing_w, swing_x, swing_y, swing_z)){
        return false;
    }

    if (swing_w < 0.0f) {
        swing_w = -swing_w;
        swing_x = -swing_x;
        swing_y = -swing_y;
        swing_z = -swing_z;
    }

    return true;
}

bool extract_swing_feature_from_quat(const float qw, const float qx, const float qy, const float qz,
                                     float& lateral_rad, float& vertical_rad, float& swing_mag_rad,
                                     float& swing_qw, float& swing_qx, float swing_qy, float swing_qz)
{
    //quaternion에서 Roll/Pitch용 2D swing feature을 extraction
    if (!extract_swing_quat_from_total(qw, qx, qy, qz,
                                       swing_qw, swing_qx, swing_qy, swing_qz)) {
        //swing extraction fail시 feature = 0;
        lateral_rad = 0.0f;
        vertical_rad = 0.0f;
        swing_mag_rad = 0.0f;
        return false;
    }

    float rx = 1.0f;
    float ry = 0.0f;
    float rz = 0.0f;
    //reference axis의 성분

    quat_rotate_vec_unit(swing_qw, swing_qx, swing_qy, swing_qz,
                         1.0f, 0.0f, 0.0f,
                         rx, ry, rz);
    //swing quaternion으로 local x axis rotate

    lateral_rad = std::atan2(ry, rx);
    //x axis가 local y 방향으로 얼마나 기울었는지 lateral로 define

    vertical_rad = std::atan2(-rz, rx);
    //x axis가 local z 방향으로 얼마나 기울었는지 vertical로 define

    swing_mag_rad = std::sqrt(lateral_rad*lateral_rad + vertical_rad*vertical_rad);
    //2D Swing feature norm을 생성

    if (!is_finite_float(lateral_rad) ||
        !is_finite_float(vertical_rad) ||
        !is_finite_float(swing_mag_rad)) {
        //수치가 깨지면 0과 false 처리
        lateral_rad = 0.0f;
        vertical_rad = 0.0f;
        swing_mag_rad = 0.0f;
        return false;
    }

    return true;
}

bool scaled_quat_step_from_identity(const float target_w, const float target_x, const float target_y, const float target_z,
                                    const float dt_sec, const float taw_sec, const float max_step_rad_s,
                                    float& step_w, float& step_x, float& step_y, float& step_z)
{
    //identity에서 target quaternion 방향으로 alpha 비율만큼 scale
    //baseline의 scale에 max_step_rad를 적용하여 느리게 이동
    float q_w = target_w;
    float q_x = target_x;
    float q_y = target_y;
    float q_z = target_z;
    //target quaternion을 저장

    if (!normalize_quat(q_w, q_x, q_y, q_z)) {
        // target이 invalid면 identity step을 반환한다.
        step_w = 1.0f;
        step_x = 0.0f;
        step_y = 0.0f;
        step_z = 0.0f;
        return false;
    }

    if (q_w < 0.0f) {
        q_w = -q_w;
        q_x = -q_x;
        q_y = -q_y;
        q_z = -q_z;
    }

    const float half_angle = std::acos(clamp_f(q_w, -1.0f, 1.0f)); 
    //target quaternion의 half angle을 복원

    const float full_angle = 2.0f * half_angle;
    //target quaternion의 full angle을 저장

    if (full_angle < k_Angle_Eps) {
        //full angle이 k_Angle_Eps보다 작을 경우 identity step
        step_w = 1.0f;
        step_x = 0.0f;
        step_y = 0.0f;
        step_z = 0.0f;
        return;
    }

    const float sin_half = std::sin(half_angle);
    //axis 복원에 필요한 sin(half_angle) 저장

    if (std::fabs(sin_half) < k_Angle_Eps) {
        //특이구간이면 identity step으로 fallback한다.
        step_w = 1.0f;
        step_x = 0.0f;
        step_y = 0.0f;
        step_z = 0.0f;
        return true;
    }

    const float axis_x = q_x / sin_half;
    const float axis_y = q_y / sin_half;
    const float axis_z = q_z / sin_half;
    // 회전축을 복원한다.

    const float tau = std::max(tau_sec, 1.0e-3f);
    //tau가 0이 되지 않도록 protect

    const float dt = std::max(dt_sec, 0.0f);
    //dt를 0 이상으로 protect

    const float alpha = 1.0f - std::exp(-dt / tau);
    //EMA time constant 기반으로 이번 tick에서의 alpha compute

    float step_angle = alpha * full_angle;
    //EMA 비율만큼 목표 step 각도를 생성

    const float max_step_angle = std::max(max_step_rad_s, 0.0f) * dt;
    //최대 baseline 적용 속도를 tick 단위 각도로 변환

    step_angle = std::min(step_angle, max_step_angle);
    //EMA 요청 각도와 max_step_angle 사이 더 작은 값 적용

    if (step_angle < k_Angle_Eps) {
        // 최종 step 각도가 너무 작으면 사실상 무동작으로 본다.
        step_w = 1.0f;
        step_x = 0.0f;
        step_y = 0.0f;
        step_z = 0.0f;
        return true;
    }

    const float step_half = 0.5f * step_angle;
    //step quaternion의 half angle 생성

    const float step_sin_half = std::sin(step_half);
    //step quaternion의 vector part 배율

    step_w = std::cos(step_half);
    step_x = axis_x * step_sin_half;
    step_y = axis_y * step_sin_half;
    step_z = axis_z * step_sin_half;
    // step quaternion의 성분 생성

    return nomalize_quat(step_w, step_x, step_y, step_z);
    //step quaternion의 unit quaternion 성공 여부 반환
    }

float apply_deadzone_and_scale(const float value_rad, const float deadzone_rad, 
                               const float max_swing_rad, const float max_rate_rad_s)
{
    //deadzone 밖에서 0부터 시작하는 linear scale
    const float av = std::fabs(value_rad);
}
}

RP_Computer::RP_Computer(const Params& param): _param(param)

void RP_Computer::reset_runtime_state()
{
    //외부에서 강제 reset시 호출
    reset_runtime_state();
}

bool RP_Computer::try_initialize_baseline(const Input& input, Output& output)
{
    //초기 baseline latch try
    if (!input.init_request) {
        //init_request가 false일 경우 false return
        return false;
    }

    if (!input.q_rel_valid) {
        //입력 quaternion이 invalid할 경우 false return
        return false;
    }

    float q_w = input.q_rel_w;
    float q_x = input.q_rel_x;
    float q_y = input.q_rel_y;
    float q_z = input.q_rel_z;
    // 입력 q_rel의 성분들을 복사

    if (!normalize_quat(q_w, q_x, q_y, q_z)) {
        // q_rel의 normalization이 되지 않으면 false return
        return false;
    }

    float init_lat = 0.0f;
    float init_ver = 0.0f;
    float init_mag = 0.0f;
    //init acceptance 검사용 변수

    float swing_qw = 1.0f;
    float swing_qx = 0.0f;
    float swing_qy = 0.0f;
    float swing_qz = 0.0f;
    // 임시 swing quaternion의 성분

    if (!extract_swing_feature_from_quat(q_w, q_x, q_y, q_z,
                                         init_lat, init_ver, init_mag,
                                         swing_qw, swing_qx, swing_qy, swing_qz)){
        return false;
        //현재 q_rel에서 swing extraction이 불가한 경우 initial baseline 거부
    }

    _q_rel_init_w = q_w;
    _q_rel_init_x = q_x;
    _q_rel_init_y = q_y;
    _q_rel_init_z = q_z;
    //현재 q_rel를 baseline으로 저장

    _q_rel0_w = _q_rel_init_w;
    _q_rel0_x = _q_rel_init_x;
    _q_rel0_y = _q_rel_init_y;
    _q_rel0_z = _q_rel_init_z;
    // 현재 q_rel을 baseline z로 저장한다.

    _initialized = true;
    //initial baseline latch 성공 여부를 true로 저장

    _baseline_hold_time_sec = 0.0f;
    //update hold timer를 0으로 초기화

    output.rp_initialized = true;
    //Output에 initial baseline 존재 반영

    output.init_done_this_tick = true;
    //현재 tick에 init 완료를 표시

    return true;
}

void RP_Computer::compute_q_delta(const Input& input, Output& output) const
{
    // q_delta = q_rel0^{-1} ⊗ q_relfmf compute

    if (!_initialized || !input.q_rel_valid) {
        //baseline이 없거나 input이 invalid할 경우 identity 사용
        output.q_delta_w = 1.0f;
        output.q_delta_x = 0.0f;
        output.q_delta_y = 0.0f;
        output.q_delta_z = 0.0f;
        return;
    }

    float q_rel_w = input.q_rel_w;
    float q_rel_x = input.q_rel_x;
    float q_rel_y = input.q_rel_y;
    float q_rel_z = input.q_rel_z;
    //q_rel의 성분을 복사

    if (!normalize_quat(q_rel_w, q_rel_x, q_rel_y, q_rel_z)) {
        //q_rel nomalization fail 시 identity fallback
        output.q_delta_w = 1.0f;
        output.q_delta_x = 0.0f;
        output.q_delta_y = 0.0f;
        output.q_delta_z = 0.0f;
        return;
    }

    float q0_w = _q_rel0_w;
    float q0_x = _q_rel0_x;
    float q0_y = _q_rel0_y;
    float q0_z = _q_rel0_z;
    // baseline의 성분을 복사

    if (!normalize_quat(q0_w, q0_x, q0_y, q0_z)) {
        //baseline이 깨졌으면 identity fallback
        output.q_delta_w = 1.0f;
        output.q_delta_x = 0.0f;
        output.q_delta_y = 0.0f;
        output.q_delta_z = 0.0f;
        return;
    }

    float q0_inv_w = 1.0f;
    float q0_inv_x = 0.0f;
    float q0_inv_y = 0.0f;
    float q0_inv_z = 0.0f;
    //baseline inverse 성분을 저장

    quat_inverse_unit(q0_w, q0_x, q0_y, q0_z,
                      q0_inv_w, q0_inv_x, q0_inv_y, q0_inv_z);
    //q_rel0^{-1}를 생성

    quat_multiply(q0_inv_w, q0_inv_x, q0_inv_y, q0_inv_z,
                  q_rel_w, q_rel_x, q_rel_y, q_rel_z,
                  output.q_delta_w,
                  output.q_delta_x,
                  output.q_delta_y,
                  output.q_delta_z);
    //q_delta를 compute

    if (!normalize_quat(output.q_delta_w,
                        output.q_delta_x,
                        output.q_delta_y,
                        output.q_delta_z)) {
        //delta quaternion이 깨지면 identity fallback
        output.q_delta_w = 1.0f;
        output.q_delta_x = 0.0f;
        output.q_delta_y = 0.0f;
        output.q_delta_z = 0.0f;
        return;
    }

    if (output.q_delta_w < 0.0f) {
        //shortest-path 해석을 위해 w>=0으로 통일
        output.q_delta_w = -output.q_delta_w;
        output.q_delta_x = -output.q_delta_x;
        output.q_delta_y = -output.q_delta_y;
        output.q_delta_z = -output.q_delta_z;
    }
}

void RP_Computer::compute_swing_from_q_delta(Output& output) const
{
    //q_delta에서 swing-only 2D feature extraction

    float lateral = 0.0f;
    float vertical = 0.0f;
    //lateral/vertical 임시 변수 생성

    float swing_mag = 0.0f;
    // swing magnitude 임시 변수 생성

    float swing_qw = 1.0f;
    float swing_qx = 0.0f;
    float swing_qy = 0.0f;
    float swing_qz = 0.0f;
    // swing quaternion 임시 변수 생성

    if (!extract_swing_feature_from_quat(output.q_delta_w,
                                         output.q_delta_x,
                                         output.q_delta_y,
                                         output.q_delta_z,
                                         lateral,
                                         vertical,
                                         swing_mag,
                                         swing_qw,
                                         swing_qx,
                                         swing_qy,
                                         swing_qz)) {
        //extraction fail시 feature를 0
        output.swing_lateral = 0.0f;
        output.swing_vertical = 0.0f;
        return;
    }

    output.swing_lateral = lateral;
    output.swing_vertical = vertical;
    //lateral/vertical feature를 저장
}

bool RP_Computer::try_update_baseline(const Input& input, Output& output)
{
    //baseline update 후보 검사 후 hold time 만족시 EMA를 통하여 baseline update
    
}