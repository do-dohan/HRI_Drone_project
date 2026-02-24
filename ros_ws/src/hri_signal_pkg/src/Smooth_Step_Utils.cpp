#include "hri_signal_pkg/HRI_Drone_HPP/Smooth_Step_Utils.hpp"

// 생성자 구현: 초기화 리스트를 사용하여 임계값 _e0와 _e1을 설정합니다.
// Constructor implementation: Initialize thresholds _e0 and _e1 using initialization list.
SmoothStep::SmoothStep(float e0, float e1, float out0, float out1) : _e0(e0), _e1(e1), _out0(out0), _out1(out1) {
    // 안전장치: 실수로 뒤집혀도 의미 유지
    if (_e1 < _e0) std::swap(_e0, _e1);
}

// 입력된 오차를 기반으로 SmoothStep 공식을 적용하여 계수를 계산하는 함수입니다.
// Function to calculate the coefficient by applying the SmoothStep formula based on the input error.
float SmoothStep::compute(float error) const {
    // error는 보통 |a_norm - 1|이지만, 혹시 음수로 들어와도 안전하게
    error = std::fabs(error);

    // e0==e1이면 step처럼 처리
    const float denom = (_e1 - _e0);
    if (denom <= 1e-12f) {
        return (error >= _e1) ? 1.0f : 0.0f;
    }

    // 계산된 값을 0.0과 1.0 사이로 클램핑(제한)하여 x를 구합니다.
    // Clamp the calculated value between 0.0 and 1.0 to obtain x.
    const float x = std::clamp((error - _e0) / denom, 0.0f, 1.0f);

    // 2. 3차 다항식 적용: s = x^2 * (3 - 2x) 공식을 사용하여 부드러운 곡선을 만듭니다.
    // 2. Apply cubic polynomial: Create a smooth curve using the formula s = x^2 * (3 - 2x).
    return x * x * (3.0f - 2.0f * x);
}

float SmoothStep::interpol(float error) const { //out1 = (s = 1)에서의 값, out0 = (s = 0)에서의 값
    const float s = compute(error); 
    return _out0 + (_out1 - _out0) * s;
}