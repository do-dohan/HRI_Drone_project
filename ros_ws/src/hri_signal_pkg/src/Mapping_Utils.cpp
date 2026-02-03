#include "hri_signal_pkg/Mapping_Utils.hpp"

// IMU_Mapping 생성자: 기울기 입력(-30~30도)을 제어 출력(-1~1)으로 매핑 설정
// IMU_Mapping Constructor: Configures mapping from tilt input (-30 to 30) to control output (-1 to 1)
IMU_Mapping::IMU_Mapping():
    Base_Mapping(-30.0f, 30.0f, -1.0f, 1.0f) {}
    
float IMU_Mapping::map(float input) {
    float abs_in = std::abs(input);           // 입력의 절대값 계산 / Calculate absolute value of input
    float sign = (input > 0) ? 1.0f : -1.0f;  // 원래의 방향(부호) 저장 / Store the original direction (sign)
    float output = 0.0f;
    
    // 입력을 설정된 최대 범위 내로 제한
    // Clamp the input within the configured maximum range
    abs_in = clamp(abs_in, 0.0f, this->_in_max);
    
    if (abs_in < 5.0f) {
        // [사역대 및 지수 처리] 5도 미만의 미세 움직임은 제곱근을 사용하여 매우 부드럽게 반응
        // [Deadzone & Expo] Fine movements below 5 degrees react very smoothly using quadratic mapping
        output = 0.1f * (abs_in * abs_in);
    }
    else {
        // 5도 이상의 움직임은 선형적으로 반응 (연속성을 위해 2.5 오프셋 보정)
        // Movements above 5 degrees react linearly (Compensate with 2.5 offset for continuity)
        output = abs_in - 2.5f;
    }

    // 결과값을 정규화하고 원래 방향을 곱하여 최종 출력 계산
    // Normalize the result and multiply by the original sign for final output
    float final_mapped = output / (this->_in_max - 2.5f);
    return clamp(final_mapped, this->_out_min, this->_out_max) * sign;
}

// Flex_Mapping 생성자: ADC 입력(0~1024)을 출력(0~1)으로 매핑 설정
// Flex_Mapping Constructor: Configures mapping from ADC input (0 to 1024) to output (0 to 1)
Flex_Mapping::Flex_Mapping():
    Base_Mapping(0.0f, 1024.0f, 0.0f, 1.0f), _threshold(150.0f) {}

void Flex_Mapping::set_initial_threshold(float initial_flex_value){
    // 초기 센서값에 30의 여유분을 더해 동작 기준점(문턱값) 설정
    // Set the activation threshold by adding a margin of 30 to the initial sensor value
    this->_threshold = initial_flex_value + 30.0f;
}

float Flex_Mapping::map(float input) {
    // 입력값을 ADC 가용 범위 내로 제한
    // Clamp the input within the available ADC range
    float clamped_in = clamp(input, this->_in_min, this->_in_max);
    
    // 문턱값이 안전 범위를 벗어나지 않도록 다시 한번 제한
    // Re-limit the threshold to ensure it stays within a safe range
    float safe_threshold = clamp(this->_threshold, 0, 300);
    
    if (clamped_in < safe_threshold) {
        return 0.0f; // 문턱값보다 작으면 움직임이 없는 것으로 간주 / Ignore if input is below threshold
    }
    
    // 실제 동작이 일어나는 유효 범위 계산
    // Calculate the effective range where the action occurs
    float effective_range = this->_in_max - safe_threshold;
    if (effective_range < 0.001f) return 1.0f; // 나누기 0 방지 / Prevent division by zero
    
    // 유효 범위 내에서 정규화 후 제곱 처리하여 정밀한 스로틀 제어 유도
    // Normalize within the range and apply square for precise throttle control
    float normalized = (clamped_in - safe_threshold) / effective_range;
    
    return normalized * normalized;
}