#include "hri_signal_pkg/HRI_Drone_HPP/Mapping_Utils.hpp"

IMU_Mapping::IMU_Mapping()
: Base_Mapping(-30.0f, 30.0f, -1.0f, 1.0f) {}

float IMU_Mapping::map(float input) {
    // 부호: 0일 때 -1로 떨어지면 안 됨 (버그 수정)
    const float sign = (input >= 0.0f) ? 1.0f : -1.0f;

    // 절대값 기반으로 크기만 매핑
    float abs_in = std::fabs(input);

    // 입력 최대 범위로 클램프
    abs_in = clamp_f(abs_in, 0.0f, _in_max);

    float output = 0.0f;

    // 0~5deg: 아주 부드러운(quad) 반응
    if (abs_in < 5.0f) {
        output = 0.1f * (abs_in * abs_in);
    }
    // 5~30deg: 선형 반응 (연속성 맞추려고 2.5 offset)
    else {
        output = abs_in - 2.5f;
    }

    // 0..(in_max-2.5) 범위를 0..1로 정규화
    float final_mapped = output / (_in_max - 2.5f);

    // 출력 범위(-1..1)로 제한 후 부호 적용
    final_mapped = clamp_f(final_mapped, 0.0f, 1.0f);
    return clamp_f(final_mapped * sign, _out_min, _out_max);
}

Flex_Mapping::Flex_Mapping()
: Base_Mapping(0.0f, 4095.0f, 0.0f, 1.0f), _threshold(150.0f) {}

void Flex_Mapping::set_initial_threshold(float initial_flex_value) {
    // 초기값 + margin
    _threshold = initial_flex_value + 30.0f;

    // 문턱값은 ADC 범위 안으로 제한 (기존 0..300은 치명 버그)
    _threshold = clamp_f(_threshold, _in_min, _in_max - 1.0f);
}

float Flex_Mapping::map(float input) {
    // 입력 클램프
    float x = clamp_f(input, _in_min, _in_max);

    // 문턱값도 ADC 범위 안으로 클램프
    float th = clamp_f(_threshold, _in_min, _in_max - 1.0f);

    // 문턱값 이하: 0
    if (x <= th) return 0.0f;

    // 유효 범위(0 나눗셈 방지)
    float range = _in_max - th;
    if (range < 1e-6f) return 1.0f;

    // 0..1 정규화 후 clamp
    float u = (x - th) / range;
    u = clamp_f(u, 0.0f, 1.0f);

    // 제곱 expo
    float y = u * u;

    // 출력 범위(0..1) clamp
    return clamp_f(y, _out_min, _out_max);
}