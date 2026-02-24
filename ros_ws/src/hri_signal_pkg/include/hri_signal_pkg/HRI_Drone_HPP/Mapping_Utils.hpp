#ifndef MAPPING_UTILS_HPP
#define MAPPING_UTILS_HPP

#include <algorithm>
#include <cmath>

// 데이터 매핑을 위한 기본 클래스 (추상 클래스)
class Base_Mapping {
protected:
    float _in_min, _in_max, _out_min, _out_max;

public:
    Base_Mapping(float i_min, float i_max, float o_min, float o_max)
    : _in_min(i_min), _in_max(i_max), _out_min(o_min), _out_max(o_max) {}

    virtual ~Base_Mapping() = default;

    // 파생 클래스에서 구현
    virtual float map(float input) = 0;

protected:
    // float clamp 유틸
    static inline float clamp_f(float v, float lo, float hi) {
        return (v < lo) ? lo : (v > hi ? hi : v);
    }
};

// IMU(각도) -> 제어값 매핑
class IMU_Mapping : public Base_Mapping {
public:
    IMU_Mapping();
    float map(float input) override;
};

// Flex(ADC) -> 제어값 매핑
class Flex_Mapping : public Base_Mapping {
private:
    float _threshold; // 동작 문턱값(ADC)

public:
    Flex_Mapping();
    float map(float input) override;

    void set_initial_threshold(float initial_flex_value);
};

#endif