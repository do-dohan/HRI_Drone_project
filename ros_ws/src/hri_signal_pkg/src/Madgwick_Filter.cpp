#include "hri_signal_pkg/HRI_Drone_HPP/Madgwick_Filter.hpp"

// [KOR] 생성자: 샘플 주파수로부터 dt를 계산하고 쿼터니언을 단위 상태로 초기화한다.
// [ENG] Constructor: computes dt from sample frequency and initializes quaternion to identity.
MadgwickFilter::MadgwickFilter(float freq_hz)
// [KOR] 멤버 초기화: dt=0으로 시작하고 q=(1,0,0,0)로 초기화한다.
// [ENG] Member init: start with dt=0 and quaternion q=(1,0,0,0).
: _dt(0.0f), _q0(1.0f), _q1(0.0f), _q2(0.0f), _q3(0.0f)
// [KOR] 생성자 바디 시작.
// [ENG] Begin constructor body.
{
    // [KOR] 주파수가 유효하면 dt=1/f로 설정한다.
    // [ENG] If frequency is valid, set dt=1/f.
    if (freq_hz > 1e-6f) _dt = 1.0f / freq_hz;
    // [KOR] 주파수가 비정상이면 안전한 기본 dt(100Hz)를 사용한다.
    // [ENG] If frequency is invalid, use safe fallback dt (100Hz).
    else _dt = 0.01f;
    // [KOR] 생성자 종료.
    // [ENG] End constructor.
}

// [KOR] 6축 업데이트: (gyro+accel)로 qDot을 계산하고, 가속도 기반 경사하강을 적용한다.
// [ENG] 6-axis update: computes qDot from (gyro+accel) and applies accel-based gradient descent.
void MadgwickFilter::update(float gx, float gy, float gz,
// [KOR] 6축 입력: 가속도(ax,ay,az)와 베타(beta)를 받는다.
// [ENG] 6-axis inputs: acceleration (ax,ay,az) and beta.
                            float ax, float ay, float az,
// [KOR] 베타는 보정(경사하강) 강도를 의미한다.
// [ENG] Beta controls correction (gradient descent) strength.
                            float beta)
// [KOR] 함수 바디 시작.
// [ENG] Begin function body.
{
    // [KOR] 경사 벡터 s0~s3를 선언한다.
    // [ENG] Declare gradient vector components s0..s3.
    float s0, s1, s2, s3;
    // [KOR] 쿼터니언 미분 qDot0~qDot3를 선언한다.
    // [ENG] Declare quaternion derivative qDot0..qDot3.
    float qDot0, qDot1, qDot2, qDot3;
    // [KOR] 중간 계산에 쓰는 2q/4q/8q 계수를 선언한다.
    // [ENG] Declare 2q/4q/8q auxiliary coefficients.
    float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2, _8q1, _8q2;
    // [KOR] 쿼터니언 제곱항 q0^2..q3^2를 선언한다.
    // [ENG] Declare quaternion squared terms q0^2..q3^2.
    float q0q0, q1q1, q2q2, q3q3;

    // [KOR] 자이로 기반 쿼터니언 미분(회전 운동학) qDot을 계산한다.
    // [ENG] Compute gyro-based quaternion derivative (rotation kinematics) qDot.
    qDot0 = 0.5f * (-_q1 * gx - _q2 * gy - _q3 * gz);
    // [KOR] qDot1을 계산한다.
    // [ENG] Compute qDot1.
    qDot1 = 0.5f * ( _q0 * gx + _q2 * gz - _q3 * gy);
    // [KOR] qDot2를 계산한다.
    // [ENG] Compute qDot2.
    qDot2 = 0.5f * ( _q0 * gy - _q1 * gz + _q3 * gx);
    // [KOR] qDot3를 계산한다.
    // [ENG] Compute qDot3.
    qDot3 = 0.5f * ( _q0 * gz + _q1 * gy - _q2 * gx);

    // [KOR] 가속도 노름을 계산해 유효성을 확인한다(0이면 정규화 불가).
    // [ENG] Compute accel norm to validate (cannot normalize if 0).
    float Ac_norm = std::sqrt(ax*ax + ay*ay + az*az);
    // [KOR] 가속도 노름이 너무 작으면 보정 없이 적분만 수행한다.
    // [ENG] If accel norm is too small, integrate only (no correction).
    if (!(Ac_norm > 1e-6f)) {
        // [KOR] q = q + qDot*dt 적분을 수행한다.
        // [ENG] Integrate q = q + qDot*dt.
        _q0 += qDot0 * _dt; _q1 += qDot1 * _dt; _q2 += qDot2 * _dt; _q3 += qDot3 * _dt;
        // [KOR] 쿼터니언을 단위화한다.
        // [ENG] Normalize quaternion.
        _normalize();
        // [KOR] 조기 종료한다.
        // [ENG] Early return.
        return;
    }

    // [KOR] 가속도 벡터를 단위화하기 위한 역노름을 계산한다.
    // [ENG] Compute inverse norm for accel normalization.
    float invAc = 1.0f / Ac_norm;
    // [KOR] 가속도(ax,ay,az)를 단위화한다.
    // [ENG] Normalize accel (ax,ay,az).
    ax *= invAc; ay *= invAc; az *= invAc;

    // [KOR] 보정 항 계산을 위한 보조 변수(2q,4q,8q)를 계산한다.
    // [ENG] Compute auxiliary variables (2q,4q,8q) for correction.
    _2q0 = 2.0f * _q0; _2q1 = 2.0f * _q1; _2q2 = 2.0f * _q2; _2q3 = 2.0f * _q3;
    // [KOR] 4q 계수를 계산한다.
    // [ENG] Compute 4q coefficients.
    _4q0 = 4.0f * _q0; _4q1 = 4.0f * _q1; _4q2 = 4.0f * _q2;
    // [KOR] 8q 계수를 계산한다.
    // [ENG] Compute 8q coefficients.
    _8q1 = 8.0f * _q1; _8q2 = 8.0f * _q2;
    // [KOR] 쿼터니언 제곱항을 계산한다.
    // [ENG] Compute quaternion squared terms.
    q0q0 = _q0*_q0; q1q1 = _q1*_q1; q2q2 = _q2*_q2; q3q3 = _q3*_q3;

    // [KOR] 가속도 기반 목적함수의 기울기(gradient) s0를 계산한다.
    // [ENG] Compute gradient component s0 for accel objective.
    s0 = _4q0*q2q2 + _2q2*ax + _4q0*q1q1 - _2q1*ay;
    // [KOR] 기울기 s1을 계산한다.
    // [ENG] Compute gradient component s1.
    s1 = _4q1*q3q3 - _2q3*ax + 4.0f*q0q0*_q1 - _2q0*ay - _4q1 + _8q1*q1q1 + _8q1*q2q2 + _4q1*az;
    // [KOR] 기울기 s2를 계산한다.
    // [ENG] Compute gradient component s2.
    s2 = 4.0f*q0q0*_q2 + _2q0*ax + _4q2*q3q3 - _2q3*ay - _4q2 + _8q2*q1q1 + _8q2*q2q2 + _4q2*az;
    // [KOR] 기울기 s3을 계산한다.
    // [ENG] Compute gradient component s3.
    s3 = 4.0f*q1q1*_q3 - _2q1*ax + 4.0f*q2q2*_q3 - _2q2*ay;

    // [KOR] 기울기 벡터 노름을 계산한다(정규화용).
    // [ENG] Compute gradient norm for normalization.
    float S_norm = std::sqrt(s0*s0 + s1*s1 + s2*s2 + s3*s3);
    // [KOR] 노름이 충분히 크면 기울기를 정규화하고 beta로 보정항을 적용한다.
    // [ENG] If norm is large enough, normalize gradient and apply beta correction.
    if (S_norm > 1e-9f) {
        // [KOR] 기울기 역노름을 계산한다.
        // [ENG] Compute inverse gradient norm.
        float invS = 1.0f / S_norm;
        // [KOR] 기울기 성분을 단위화한다.
        // [ENG] Normalize gradient components.
        s0 *= invS; s1 *= invS; s2 *= invS; s3 *= invS;
        // [KOR] qDot에 -beta*s를 적용해 보정(gradient descent)을 수행한다.
        // [ENG] Apply correction to qDot via -beta*s (gradient descent).
        qDot0 -= beta*s0; qDot1 -= beta*s1; qDot2 -= beta*s2; qDot3 -= beta*s3;
    }

    // [KOR] 보정된 qDot을 dt로 적분해 쿼터니언을 업데이트한다.
    // [ENG] Integrate corrected qDot with dt to update quaternion.
    _q0 += qDot0 * _dt; _q1 += qDot1 * _dt; _q2 += qDot2 * _dt; _q3 += qDot3 * _dt;
    // [KOR] 수치 드리프트를 막기 위해 쿼터니언을 단위화한다.
    // [ENG] Normalize quaternion to prevent numerical drift.
    _normalize();
    // [KOR] 6축 update 종료.
    // [ENG] End of 6-axis update.
}

// [KOR] 9축 업데이트: (gyro+accel+mag)로 기울기 s를 구성하고 보정항을 적용한다.
// [ENG] 9-axis update: builds gradient s from (gyro+accel+mag) and applies correction.
void MadgwickFilter::update(float gx, float gy, float gz,
// [KOR] 9축 입력: 가속도(ax,ay,az) 추가.
// [ENG] 9-axis inputs: acceleration (ax,ay,az).
                            float ax, float ay, float az,
// [KOR] 9축 입력: 자기장(mx,my,mz) 추가.
// [ENG] 9-axis inputs: magnetometer (mx,my,mz).
                            float mx, float my, float mz,
// [KOR] 9축 입력: 베타(beta) 추가.
// [ENG] 9-axis input: beta.
                            float beta)
// [KOR] 함수 바디 시작.
// [ENG] Begin function body.
{
    // [KOR] gradient 누적용 s0~s3를 0으로 초기화한다(가속/자기 기여를 더할 것).
    // [ENG] Initialize s0..s3 to 0 to accumulate accel/mag contributions.
    float s0 = 0.0f, s1 = 0.0f, s2 = 0.0f, s3 = 0.0f;
    // [KOR] 쿼터니언 미분 qDot0~qDot3를 선언한다.
    // [ENG] Declare quaternion derivative qDot0..qDot3.
    float qDot0, qDot1, qDot2, qDot3;

    // [KOR] 보조 변수(2q, 곱항 등)를 선언한다.
    // [ENG] Declare auxiliary variables (2q, product terms, etc.).
    float _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3;
    // [KOR] 쿼터니언 곱/제곱 항을 선언한다.
    // [ENG] Declare quaternion product/squared terms.
    float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
    // [KOR] 자기장 보정 계산에 필요한 중간 변수들을 선언한다.
    // [ENG] Declare intermediate variables needed for mag correction.
    float _2q0mx, _2q0my, _2q0mz, _2q1mx;
    // [KOR] 지구 자기장 수평/수직 성분(2bx,2bz 등) 계산용 변수 선언.
    // [ENG] Declare variables for earth field components (2bx,2bz, etc.).
    float _2bx, _2bz, _4bx, _4bz;
    // [KOR] h와 b 계산용 변수를 선언한다.
    // [ENG] Declare variables for h and b computation.
    float hx, hy, bx, bz;

    // [KOR] 2q 계수를 계산한다.
    // [ENG] Compute 2q coefficients.
    _2q0 = 2.0f*_q0; _2q1 = 2.0f*_q1; _2q2 = 2.0f*_q2; _2q3 = 2.0f*_q3;
    // [KOR] 일부 곱항(2*q0*q2, 2*q2*q3)을 계산한다.
    // [ENG] Compute some product terms (2*q0*q2, 2*q2*q3).
    _2q0q2 = 2.0f*_q0*_q2; _2q2q3 = 2.0f*_q2*_q3;

    // [KOR] 쿼터니언 곱/제곱 항을 계산한다.
    // [ENG] Compute quaternion product/squared terms.
    q0q0 = _q0*_q0; q0q1 = _q0*_q1; q0q2 = _q0*_q2; q0q3 = _q0*_q3;
    // [KOR] q1 관련 항을 계산한다.
    // [ENG] Compute q1-related terms.
    q1q1 = _q1*_q1; q1q2 = _q1*_q2; q1q3 = _q1*_q3;
    // [KOR] q2/q3 관련 항을 계산한다.
    // [ENG] Compute q2/q3-related terms.
    q2q2 = _q2*_q2; q2q3 = _q2*_q3; q3q3 = _q3*_q3;

    // [KOR] 자이로 기반 qDot을 계산한다.
    // [ENG] Compute gyro-based qDot.
    qDot0 = 0.5f * (-_q1 * gx - _q2 * gy - _q3 * gz);
    // [KOR] qDot1 계산.
// [ENG] Compute qDot1.
    qDot1 = 0.5f * ( _q0 * gx + _q2 * gz - _q3 * gy);
    // [KOR] qDot2 계산.
// [ENG] Compute qDot2.
    qDot2 = 0.5f * ( _q0 * gy - _q1 * gz + _q3 * gx);
    // [KOR] qDot3 계산.
// [ENG] Compute qDot3.
    qDot3 = 0.5f * ( _q0 * gz + _q1 * gy - _q2 * gx);

    // [KOR] 가속도 노름을 계산해 유효할 때만 가속 기여를 추가한다.
    // [ENG] Compute accel norm and add accel contribution only if valid.
    float Ac_norm = std::sqrt(ax*ax + ay*ay + az*az);
    // [KOR] 가속도가 유효하면 정규화 후 accel 기반 gradient를 누적한다.
    // [ENG] If accel valid, normalize and accumulate accel-based gradient.
    if (Ac_norm > 1e-6f) {
        // [KOR] 가속도 역노름을 계산한다.
        // [ENG] Compute inverse accel norm.
        float invAc = 1.0f / Ac_norm;
        // [KOR] 가속도 벡터를 단위화한다.
        // [ENG] Normalize accel vector.
        ax *= invAc; ay *= invAc; az *= invAc;

        // [KOR] accel 목적함수의 gradient를 s0~s3에 더한다.
        // [ENG] Add accel objective gradient into s0..s3.
        s0 += -_2q2*(2.0f*q1q3 - _2q0q2 - ax) + _2q1*(2.0f*q0q1 + _2q2q3 - ay);
        // [KOR] s1에 accel 기여를 더한다.
        // [ENG] Add accel contribution to s1.
        s1 +=  _2q3*(2.0f*q1q3 - _2q0q2 - ax) + _2q0*(2.0f*q0q1 + _2q2q3 - ay) - 4.0f*_q1*(1.0f - 2.0f*q1q1 - 2.0f*q2q2 - az);
        // [KOR] s2에 accel 기여를 더한다.
        // [ENG] Add accel contribution to s2.
        s2 += -_2q0*(2.0f*q1q3 - _2q0q2 - ax) + _2q3*(2.0f*q0q1 + _2q2q3 - ay) - 4.0f*_q2*(1.0f - 2.0f*q1q1 - 2.0f*q2q2 - az);
        // [KOR] s3에 accel 기여를 더한다.
        // [ENG] Add accel contribution to s3.
        s3 +=  _2q1*(2.0f*q1q3 - _2q0q2 - ax) + _2q2*(2.0f*q0q1 + _2q2q3 - ay);
    }

    // [KOR] 자기장 노름을 계산해 유효할 때만 mag 기여를 추가한다(스케일 독립 임계).
    // [ENG] Compute mag norm and add mag contribution only if valid (scale-robust threshold).
    float M_norm = std::sqrt(mx*mx + my*my + mz*mz);
    // [KOR] 자기장 노름이 아주 작지 않으면 정규화해서 사용한다.
    // [ENG] If mag norm is not tiny, normalize and use it.
    if (M_norm > 1e-9f) {
        // [KOR] 자기장 역노름을 계산한다.
        // [ENG] Compute inverse mag norm.
        float invM = 1.0f / M_norm;
        // [KOR] 자기장을 단위화한다.
        // [ENG] Normalize magnetometer vector.
        mx *= invM; my *= invM; mz *= invM;

        // [KOR] 자계 보정 계산에 필요한 2*q0*mx 등의 중간항을 계산한다.
        // [ENG] Compute intermediate terms like 2*q0*mx for mag correction.
        _2q0mx = 2.0f*_q0*mx; _2q0my = 2.0f*_q0*my; _2q0mz = 2.0f*_q0*mz; _2q1mx = 2.0f*_q1*mx;

        // [KOR] 지구 자기장 방향을 body->earth로 회전한 h의 x 성분을 계산한다.
        // [ENG] Compute hx, the x component of rotated earth field estimate.
        hx = mx*q0q0 - _2q0my*_q3 + _2q0mz*_q2 + mx*q1q1 + _2q1*my*_q2 + _2q1*mz*_q3 - mx*q2q2 - mx*q3q3;
        // [KOR] h의 y 성분을 계산한다.
        // [ENG] Compute hy, the y component of rotated earth field estimate.
        hy = _2q0mx*_q3 + my*q0q0 - _2q0mz*_q1 + _2q1*mx*_q2 - my*q1q1 + my*q2q2 + 2.0f*_q2*mz*_q3 - my*q3q3;

        // [KOR] bx는 수평 성분의 크기(sqrt(hx^2+hy^2))로 계산한다.
        // [ENG] Compute bx as horizontal magnitude sqrt(hx^2+hy^2).
        bx = std::sqrt(hx*hx + hy*hy);
        // [KOR] bz는 수직 성분을 계산한다.
        // [ENG] Compute bz as vertical component.
        bz = -_2q0mx*_q2 + _2q0my*_q1 + mz*q0q0 + _2q1*mx*_q3 - mz*q1q1 + 2.0f*_q2*my*_q3 - mz*q2q2 + mz*q3q3;

        // [KOR] 2bx/2bz 및 4bx/4bz를 계산한다.
        // [ENG] Compute 2bx/2bz and 4bx/4bz.
        _2bx = 2.0f*bx; _2bz = 2.0f*bz; _4bx = 4.0f*bx; _4bz = 4.0f*bz;

        // [KOR] mag 목적함수의 gradient를 s0에 누적한다.
        // [ENG] Accumulate mag objective gradient into s0.
        s0 += -_2bz*_q2*(_2bx*(0.5f - q2q2 - q3q3) + _2bz*(q1q3 - q0q2) - mx)
            + (_2bx*_q1 + _2bz*_q0)*(_2bx*(q1q2 - q0q3) + _2bz*(q0q1 + q2q3) - my)
            + _2bx*_q2*(_2bx*(q0q2 + q1q3) + _2bz*(0.5f - q1q1 - q2q2) - mz);

        // [KOR] mag 목적함수의 gradient를 s1에 누적한다.
        // [ENG] Accumulate mag objective gradient into s1.
        s1 +=  _2bz*_q3*(_2bx*(0.5f - q2q2 - q3q3) + _2bz*(q1q3 - q0q2) - mx)
            + (_2bx*_q2 + _2bz*_q0)*(_2bx*(q1q2 - q0q3) + _2bz*(q0q1 + q2q3) - my)
            + (_2bx*_q3 - _4bz*_q1)*(_2bx*(q0q2 + q1q3) + _2bz*(0.5f - q1q1 - q2q2) - mz);

        // [KOR] mag 목적함수의 gradient를 s2에 누적한다.
        // [ENG] Accumulate mag objective gradient into s2.
        s2 += (-_4bx*_q2 - _2bz*_q0)*(_2bx*(0.5f - q2q2 - q3q3) + _2bz*(q1q3 - q0q2) - mx)
            + (_2bx*_q1 + _2bz*_q3)*(_2bx*(q1q2 - q0q3) + _2bz*(q0q1 + q2q3) - my)
            + (_2bx*_q0 - _4bz*_q2)*(_2bx*(q0q2 + q1q3) + _2bz*(0.5f - q1q1 - q2q2) - mz);

        // [KOR] mag 목적함수의 gradient를 s3에 누적한다.
        // [ENG] Accumulate mag objective gradient into s3.
        s3 += (-_4bx*_q3 + _2bz*_q1)*(_2bx*(0.5f - q2q2 - q3q3) + _2bz*(q1q3 - q0q2) - mx)
            + (-_2bx*_q0 + _2bz*_q2)*(_2bx*(q1q2 - q0q3) + _2bz*(q0q1 + q2q3) - my)
            + _2bx*_q1*(_2bx*(q0q2 + q1q3) + _2bz*(0.5f - q1q1 - q2q2) - mz);
    }

    // [KOR] 최종 gradient 노름을 계산한다.
    // [ENG] Compute final gradient norm.
    float S_norm = std::sqrt(s0*s0 + s1*s1 + s2*s2 + s3*s3);
    // [KOR] 노름이 유효하면 정규화 후 beta로 qDot에 보정항을 적용한다.
    // [ENG] If norm valid, normalize and apply beta correction to qDot.
    if (S_norm > 1e-9f) {
        // [KOR] 역노름을 계산한다.
        // [ENG] Compute inverse norm.
        float invS = 1.0f / S_norm;
        // [KOR] gradient를 단위화한다.
        // [ENG] Normalize gradient.
        s0 *= invS; s1 *= invS; s2 *= invS; s3 *= invS;
        // [KOR] qDot에 -beta*s를 적용한다.
        // [ENG] Apply -beta*s to qDot.
        qDot0 -= beta*s0; qDot1 -= beta*s1; qDot2 -= beta*s2; qDot3 -= beta*s3;
    }

    // [KOR] q = q + qDot*dt로 적분한다.
    // [ENG] Integrate q = q + qDot*dt.
    _q0 += qDot0 * _dt; _q1 += qDot1 * _dt; _q2 += qDot2 * _dt; _q3 += qDot3 * _dt;
    // [KOR] 쿼터니언을 단위화한다.
    // [ENG] Normalize quaternion.
    _normalize();
    // [KOR] 9축 update 종료.
    // [ENG] End of 9-axis update.
}

// [KOR] 내부 정규화 함수: q의 노름으로 나눠 단위 쿼터니언을 유지한다.
// [ENG] Internal normalize: divides by norm to keep unit quaternion.
void MadgwickFilter::_normalize()
// [KOR] 함수 바디 시작.
// [ENG] Begin function body.
{
    // [KOR] 쿼터니언 노름을 계산한다.
    // [ENG] Compute quaternion norm.
    float n = std::sqrt(_q0*_q0 + _q1*_q1 + _q2*_q2 + _q3*_q3);
    // [KOR] 노름이 유효하면 역노름으로 단위화한다.
    // [ENG] If norm valid, normalize using inverse norm.
    if (n > 1e-12f) {
        // [KOR] 역노름을 계산한다.
        // [ENG] Compute inverse norm.
        float inv = 1.0f / n;
        // [KOR] 각 성분에 역노름을 곱한다.
        // [ENG] Multiply each component by inverse norm.
        _q0 *= inv; _q1 *= inv; _q2 *= inv; _q3 *= inv;
    }
    // [KOR] normalize 종료.
    // [ENG] End normalize.
}

// [KOR] 현재 쿼터니언을 외부로 반환한다(복사).
// [ENG] Returns current quaternion to caller (copy-out).
void MadgwickFilter::getQuaternion(float& w, float& x, float& y, float& z) const
// [KOR] 함수 바디 시작.
// [ENG] Begin function body.
{
    // [KOR] w 성분에 q0를 대입한다.
    // [ENG] Assign q0 to w.
    w = _q0;
    // [KOR] x 성분에 q1를 대입한다.
    // [ENG] Assign q1 to x.
    x = _q1;
    // [KOR] y 성분에 q2를 대입한다.
    // [ENG] Assign q2 to y.
    y = _q2;
    // [KOR] z 성분에 q3를 대입한다.
    // [ENG] Assign q3 to z.
    z = _q3;
    // [KOR] getQuaternion 종료.
    // [ENG] End getQuaternion.
}