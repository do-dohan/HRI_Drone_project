#include "hri_signal_pkg/HRI_Drone_HPP/EMA_Filter.hpp"

#include <algorithm>  // std::clamp
#include <cmath>      // std::fabs (optional)

// 생성자: 필터 가중치를 설정하고 초기화 상태를 거짓으로 시작함
// Constructor: Sets the filter weight and initializes the state to false
// 생성자: alpha=0, 출력=0, 미초기화 상태로 시작
EMA_Filter::EMA_Filter()
: _alpha(0.0f), _y_output(0.0f), _is_initialized(false) {
}

void EMA_Filter::initialize(float init_y) {
    // 이미 초기화돼 있으면 그대로 둠
    if (_is_initialized) return;

    _y_output = init_y;
    _is_initialized = true;

    // 내부 alpha는 update_alpha()를 쓸 때 의미가 있으니 기본값 세팅
    _alpha = 0.0f;
}

float EMA_Filter::filter(float input, float alpha) {
    // 최초 호출이면 입력으로 상태를 초기화(초기 과도응답 방지)
    if (!_is_initialized) {
        initialize(input);
        return _y_output;
    }

    // alpha 안전 클램프
    alpha = std::clamp(alpha, 0.0f, 1.0f);

    // EMA: y = alpha*x + (1-alpha)*y_prev
    _y_output = alpha * input + (1.0f - alpha) * _y_output;
    return _y_output;
}

void EMA_Filter::update_alpha(float input) {
    // 초기화 전이면 입력으로 초기화하고 최소 alpha로 둠
    if (!_is_initialized) {
        initialize(input);
        _alpha = 0.05f;
        return;
    }

    // 변화량(오차)
    float delta = input - _y_output;

    // 알파 업데이트 파라미터(너 기존 값 유지)
    const float alpha_gain = 0.000055f;
    const float alpha_max  = 0.55f;
    const float alpha_min  = 0.05f;

    // delta^2 는 XOR라서 버그. (delta*delta)로 제곱.
    float a = alpha_min + alpha_gain * (delta * delta);

    // 범위 제한
    a = std::clamp(a, alpha_min, alpha_max);

    // 내부 alpha 저장
    _alpha = a;
}