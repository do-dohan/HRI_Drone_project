#ifndef MADGWICK_FILTER_HPP
#define MADGWICK_FILTER_HPP

#include <cmath>

class MadgwickFilter {
private:
    float _dt; 
    float _q0, _q1, _q2, _q3; // 쿼터니언

    void _normalize();      // 내부 정규화 함수

public:
    explicit MadgwickFilter(float freq);
    
    // [6축 모드] 가속도 + 자이로 (빠른 주기로 호출)
    // [6-Axis Mode] Accel + Gyro (Call at high frequency)
    void update(float gx, float gy, float gz, float ax, float ay, float az, float beta);

    // [9축 모드] 가속도 + 자이로 + 지자기 (데이터가 있을 때 호출)
    // [9-Axis Mode] Accel + Gyro + Mag (Call when mag data is available)
    void update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float beta);

    void getQuaternion(float& w, float& x, float& y, float& z);
};

#endif