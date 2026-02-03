#ifndef MAPPING_UTILS_HPP
#define MAPPING_UTILS_HPP

#include <algorithm>
#include <cmath>

/**
 * @brief 데이터 매핑을 위한 기본 클래스 (추상 클래스)
 * @details Base class for data mapping (Abstract class)
 */
class Base_Mapping {
protected:
    // 입력 및 출력의 최소/최대 범위를 저장하는 멤버 변수들
    // Member variables storing the minimum and maximum ranges for input and output
    float _in_min, _in_max, _out_min, _out_max;
    
public:
    /**
     * @brief 생성자: 매핑에 필요한 입출력 범위를 초기화함
     * @param i_min 입력 최소, i_max 입력 최대, o_min 출력 최소, o_max 출력 최대
     */
    Base_Mapping(float i_min, float i_max, float o_min, float o_max) :
    _in_min(i_min), _in_max(i_max), _out_min(o_min), _out_max(o_max) {}

    /**
     * @brief 순수 가상 함수: 상속받은 클래스에서 각자 고유의 매핑 로직을 구현함
     * @details Pure virtual function: Derived classes implement their specific mapping logic
     */
    virtual float map(float input) = 0;

    /**
     * @brief 입력값을 지정된 최소/최대 범위 내로 제한하는 함수
     * @details Function to limit the input value within the specified min/max range
     */
    float clamp(float val, float min_val, float max_val) {
        return (val < min_val) ? min_val : (val > max_val ? max_val : val);
    }
};

/**
 * @brief IMU(자세 센서) 데이터를 제어 값으로 변환하는 클래스
 * @details Class to convert IMU (Orientation sensor) data into control values
 */
class IMU_Mapping : public Base_Mapping {
public:
    IMU_Mapping();
    float map(float input) override;
};

/**
 * @brief 굴곡 센서(Flex Sensor) 데이터를 제어 값으로 변환하는 클래스
 * @details Class to convert Flex sensor data into control values
 */
class Flex_Mapping : public Base_Mapping {
private:
    float _threshold; // 동작 여부를 결정하는 문턱값
    // Threshold value to determine the activation
    
public:
    Flex_Mapping();
    float map(float input) override;

    /**
     * @brief 초기 문턱값 설정: 사용자의 초기 상태를 기준으로 기준점을 잡음
     * @param initial_flex_value 초기 센서 입력값
     */
    void set_initial_threshold(float initial_flex_value);
};

#endif