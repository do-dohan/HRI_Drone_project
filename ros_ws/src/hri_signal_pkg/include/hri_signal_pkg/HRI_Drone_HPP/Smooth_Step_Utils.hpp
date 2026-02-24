#ifndef SMOOTH_STEP_UTILS_HPP
#define SMOOTH_STEP_UTILS_HPP

#include <algorithm> // std::clamp
#include <cmath>     // std::fabs
#include <utility>   // std::swap

class SmoothStep {
public:
    // 생성자: 정지 상태로 간주할 오차 하한값(e0)과 움직임 상태로 간주할 오차 상한값(e1)을 설정합니다.
    // Constructor: Set the lower error threshold (e0) for static state and upper threshold (e1) for motion state.
    SmoothStep(float e0, float e1, float out0, float out1);

    // 오차(error)를 입력받아 0.0(정지) ~ 1.0(움직임) 사이의 보간 계수 s를 반환합니다.
    // Inputs error and returns interpolation coefficient s between 0.0 (static) and 1.0 (motion).
    float compute(float error) const;

    float interpol(float error) const;

private:
    float _e0; // 정지 기준 (Beta High) / Static threshold
    float _e1; // 움직임 기준 (Beta Low) / Motion threshold
    float _out0;
    float _out1;
};

#endif // SMOOTH_STEP_UTILS_HPP