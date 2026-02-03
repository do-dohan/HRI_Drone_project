#include "hri_signal_pkg/EMA_Filter.hpp"

// 생성자: 필터 가중치를 설정하고 초기화 상태를 거짓으로 시작함
// Constructor: Sets the filter weight and initializes the state to false
EMA_Filter::EMA_Filter(float a) : _alpha(a), _y_output(0.0f), _is_initialized(false) {}

void EMA_Filter::initialize(float init_y) {
    // 필터가 아직 초기화되지 않았다면 첫 번째 입력값으로 필터 상태를 채움
    // If the filter is not yet initialized, set the initial state with the first input
    if (!_is_initialized) {
        this->_y_output = init_y;
        this->_is_initialized = true;
    }
}

float EMA_Filter::filter(float input) {
    // 지수 이동 평균 수식 적용: y = alpha * x + (1 - alpha) * y_prev
    // Apply EMA formula: y = alpha * x + (1 - alpha) * y_prev
    this->_y_output = this->_alpha * input + (1.0f - this->_alpha) * this->_y_output;
    return _y_output; // 필터링된 결과값 반환 / Return the filtered result
}

void EMA_Filter::update_alpha(float input) {
    // 현재 입력값과 이전 출력값 사이의 오차(변화량)를 계산
    // Calculate the error (delta) between current input and previous output
    float delta = input - this->_y_output;
    
    // 알파 업데이트를 위한 게인값 및 제한값 설정
    // Constants for alpha update gain and limits
    const float alpha_gain = 0.000055f; // 변화에 반응하는 속도 조절 / Sensitivity to changes
    const float alpha_max = 0.55f;      // 최대 반응성 제한 / Maximum responsiveness limit
    const float alpha_min = 0.05f;      // 최소 노이즈 제거 성능 유지 / Minimum noise reduction limit

    // 오차의 제곱에 비례하여 alpha를 동적으로 조정 (급격한 변화 시 반응성 증가)
    // Dynamically adjust alpha proportional to error squared (Increases responsiveness during sharp changes)
    this->_alpha = (delta * delta * alpha_gain) + alpha_min;

    // alpha 값이 설정된 최대치를 넘지 않도록 제한
    // Constrain the alpha value within the maximum limit
    if (this->_alpha > alpha_max) {
        this->_alpha = alpha_max;
    }
}