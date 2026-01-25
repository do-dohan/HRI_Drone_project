#include "HRI_Drone_HPP/Madgwick_Filter.hpp"

// 생성자: 초기 자세를 설정하고 샘플링 주기를 계산함
// Constructor: Sets initial orientation and calculates sampling interval
MadgwickFilter::MadgwickFilter(float freq, float b) {
    this->_invSampleFreq = 1.0f / freq; // 샘플링 주파수의 역수 (dt) 저장 / Store inverse of frequency
    this->_beta = b;                    // 알고리즘 게인 설정 / Set algorithm gain
    this->_q0 = 1.0f;                   // 초기 쿼터니언 값 설정 (정지 상태)
    this->_q1 = 0.0f;                   // Initial quaternion values (stationary state)
    this->_q2 = 0.0f;
    this->_q3 = 0.0f;
}

// [6축 모드 구현] 지자기 관련 연산을 제거한 경량화 버전
// [6-Axis Implementation] Lightweight version with magnetometer calculations removed
void MadgwickFilter::update(float gx, float gy, float gz, float ax, float ay, float az) {
    float s0, s1, s2, s3;
    float qDot0, qDot1, qDot2, qDot3;
    float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2, _8q1, _8q2;
    float q0q0, q1q1, q2q2, q3q3;

    // 자이로스코프를 이용한 변화율 예측
    qDot0 = 0.5f * (-this->_q1 * gx - this->_q2 * gy - this->_q3 * gz);
    qDot1 = 0.5f * (this->_q0 * gx + this->_q2 * gz - this->_q3 * gy);
    qDot2 = 0.5f * (this->_q0 * gy - this->_q1 * gz + this->_q3 * gx);
    qDot3 = 0.5f * (this->_q0 * gz + this->_q1 * gy - this->_q2 * gx);

    // 가속도 데이터 유효성 검사
    float Ac_norm = std::sqrt(ax * ax + ay * ay + az * az);
    if (!(Ac_norm > 0.1f && Ac_norm < 10.0f)) {
        // 가속도 신뢰 불가 시 자이로 적분만 수행
        this->_q0 += qDot0 * this->_invSampleFreq;
        this->_q1 += qDot1 * this->_invSampleFreq;
        this->_q2 += qDot2 * this->_invSampleFreq;
        this->_q3 += qDot3 * this->_invSampleFreq;
        _normalize();
        return;
    }

    // 가속도 정규화
    float invAc_norm = 1.0f / Ac_norm;
    ax *= invAc_norm; ay *= invAc_norm; az *= invAc_norm;

    // 보조 변수 계산 (6축 전용 최적화)
    _2q0 = 2.0f * this->_q0; _2q1 = 2.0f * this->_q1; _2q2 = 2.0f * this->_q2; _2q3 = 2.0f * this->_q3;
    _4q0 = 4.0f * this->_q0; _4q1 = 4.0f * this->_q1; _4q2 = 4.0f * this->_q2; _8q1 = 8.0f * this->_q1; _8q2 = 8.0f * this->_q2;
    q0q0 = this->_q0 * this->_q0; q1q1 = this->_q1 * this->_q1; q2q2 = this->_q2 * this->_q2; q3q3 = this->_q3 * this->_q3;

    // 경사 하강법 (Gradient Descent) - 중력 방향 오차 보정만 수행
    s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
    s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * this->_q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
    s2 = 4.0f * q0q0 * this->_q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
    s3 = 4.0f * q1q1 * this->_q3 - _2q1 * ax + 4.0f * q2q2 * this->_q3 - _2q2 * ay;

    // 오차 벡터 정규화
    float S_norm = std::sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
    if (S_norm > 0.0001f) {
        float invS = 1.0f / S_norm;
        s0 *= invS; s1 *= invS; s2 *= invS; s3 *= invS;
        // 피드백 적용
        qDot0 -= this->_beta * s0;
        qDot1 -= this->_beta * s1;
        qDot2 -= this->_beta * s2;
        qDot3 -= this->_beta * s3;
    }

    // 적분 및 정규화
    this->_q0 += qDot0 * this->_invSampleFreq;
    this->_q1 += qDot1 * this->_invSampleFreq;
    this->_q2 += qDot2 * this->_invSampleFreq;
    this->_q3 += qDot3 * this->_invSampleFreq;
    _normalize();
}

// [9축 모드 구현] 기존의 지자기 포함 로직 (무거운 연산)
// [9-Axis Implementation] Original logic including magnetometer (Heavy computation)
void MadgwickFilter::update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz) {
    // 경사 하강법을 위한 오차 벡터 초기화
    // Initialize error vector for Gradient Descent
    float s0 = 0.0f, s1 = 0.0f, s2 = 0.0f, s3 = 0.0f;
    float qDot0, qDot1, qDot2, qDot3; // 쿼터니언 변화율 / Quaternion rate of change
    
    // 계산 효율을 위한 중간 변수들 (Common Sub-expression Elimination)
    // Intermediate variables for computational efficiency
    float _2q0mx, _2q0my, _2q0mz, _2q1mx;
    float _2bx, _2bz, _4bx, _4bz;
    float _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3;
    float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
    float hx, hy, bx, bz;

    // 멤버 변수를 활용한 상수 계산 (언더바 변수 적용)
    // Constant calculations using member variables
    _2q0 = 2.0f * this->_q0; _2q1 = 2.0f * this->_q1; _2q2 = 2.0f * this->_q2; _2q3 = 2.0f * this->_q3;
    _2q0q2 = 2.0f * this->_q0 * this->_q2; _2q2q3 = 2.0f * this->_q2 * this->_q3;
    q0q0 = this->_q0 * this->_q0; q0q1 = this->_q0 * this->_q1; q0q2 = this->_q0 * this->_q2; q0q3 = this->_q0 * this->_q3;
    q1q1 = this->_q1 * this->_q1; q1q2 = this->_q1 * this->_q1; q1q3 = this->_q1 * this->_q3;
    q2q2 = this->_q2 * this->_q2; q2q3 = this->_q2 * this->_q3; q3q3 = this->_q3 * this->_q3;

    // 단계 1: 자이로스코프 데이터를 이용한 자세 변화 예측
    // Step 1: Predict orientation rate from gyroscope data
    qDot0 = 0.5f * (-this->_q1 * gx - this->_q2 * gy - this->_q3 * gz);
    qDot1 = 0.5f * (this->_q0 * gx + this->_q2 * gz - this->_q3 * gy);
    qDot2 = 0.5f * (this->_q0 * gy - this->_q1 * gz + this->_q3 * gx);
    qDot3 = 0.5f * (this->_q0 * gz + this->_q1 * gy - this->_q2 * gx);

    // 단계 2: 가속도계를 이용한 데이터 보정
    // Step 2: Data correction using accelerometer
    float Ac_norm = std::sqrt(ax * ax + ay * ay + az * az);
    if (Ac_norm > 0.1f && Ac_norm < 10.0f) {
        float invAc_norm = 1.0f / Ac_norm;
        ax *= invAc_norm; ay *= invAc_norm; az *= invAc_norm; // 정규화 / Normalization
        
        // 경사 하강법 단계별 오차 누적
        // Gradient Descent error accumulation
        s0 += -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - ay);
        s1 += _2q3 * (2.0f * q1q3 - _2q0q2 - ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * this->_q1 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az);
        s2 += -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * this->_q2 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az);
        s3 += _2q1 * (2.0f * q1q3 - _2q0q2 - ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - ay);
    }

    // 단계 3: 지자기 센서를 이용한 방위각 보정
    // Step 3: Heading correction using geomagnetic field
    float M_norm = std::sqrt(mx * mx + my * my + mz * mz);
    if (M_norm > 0.01f && M_norm < 10.0f) {
        float invM_norm = 1.0f / M_norm;
        mx *= invM_norm; my *= invM_norm; mz *= invM_norm; // 정규화 / Normalization
        
        _2q0mx = 2.0f * this->_q0 * mx; _2q0my = 2.0f * this->_q0 * my; _2q0mz = 2.0f * this->_q0 * mz; _2q1mx = 2.0f * this->_q1 * mx;

        // 지구 자기장 벡터 보정
        // Earth's magnetic field vector correction
        hx = mx * q0q0 - _2q0my * this->_q3 + _2q0mz * this->_q2 + mx * q1q1 + _2q1 * my * this->_q2 + _2q1 * mz * this->_q3 - mx * q2q2 - mx * q3q3;
        hy = _2q0mx * this->_q3 + my * q0q0 - _2q0mz * this->_q1 + _2q1 * mx * this->_q2 - my * q1q1 + my * q2q2 + 2.0f * this->_q2 * mz * this->_q3 - my * q3q3;
        bx = std::sqrt(hx * hx + hy * hy);
        bz = -_2q0mx * this->_q2 + _2q0my * this->_q1 + mz * q0q0 + _2q1 * mx * this->_q3 - mz * q1q1 + 2.0f * this->_q2 * my * this->_q3 - mz * q2q2 + mz * q3q3;
        _2bx = 2.0f * bx; _2bz = 2.0f * bz; _4bx = 4.0f * bx; _4bz = 4.0f * bz;

        // 지자기 데이터를 포함한 경사 하강법 적용
        // Apply Gradient Descent including geomagnetic data
        s0 += -_2bz * this->_q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * this->_q1 + _2bz * this->_q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * this->_q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        s1 += _2bz * this->_q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * this->_q2 + _2bz * this->_q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * this->_q3 - _4bz * this->_q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        s2 += (-_4bx * this->_q2 - _2bz * this->_q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * this->_q1 + _2bz * this->_q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * this->_q0 - _4bz * this->_q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        s3 += (-_4bx * this->_q3 + _2bz * this->_q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * this->_q0 + _2bz * this->_q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * this->_q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
    }

    // 오차 벡터 정규화 및 최종 자세 업데이트
    // Error vector normalization and final orientation update
    float S_norm = std::sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
    if (S_norm > 0.0001f) {
        float InvSnorm = 1.0f / S_norm;
        s0 *= InvSnorm; s1 *= InvSnorm; s2 *= InvSnorm; s3 *= InvSnorm;
        qDot0 -= this->_beta * s0; qDot1 -= this->_beta * s1; qDot2 -= this->_beta * s2; qDot3 -= this->_beta * s3;
    }

    // 최종 적분 (계산된 변화율을 현재 자세에 더함)
    // Final Integration (Add calculated rate of change to current orientation)
    this->_q0 += qDot0 * this->_invSampleFreq;
    this->_q1 += qDot1 * this->_invSampleFreq;
    this->_q2 += qDot2 * this->_invSampleFreq;
    this->_q3 += qDot3 * this->_invSampleFreq;
    _normalize(); // 결과값 정규화 / Result normalization
}

void MadgwickFilter::_normalize() {
    float norm = std::sqrt(this->_q0 * this->_q0 + this->_q1 * this->_q1 + this->_q2 * this->_q2 + this->_q3 * this->_q3);
    if (norm > 0.0001f) {
        float Invnorm = 1.0f / norm;
        this->_q0 *= Invnorm; this->_q1 *= Invnorm; this->_q2 *= Invnorm; this->_q3 *= Invnorm;
    }
}

void MadgwickFilter::getQuatarian(float& w, float& x, float& y, float& z) {
    w = this->_q0; x = this->_q1; y = this->_q2; z = this->_q3;
}