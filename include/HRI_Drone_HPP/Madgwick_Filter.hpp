#ifndef MADGWICK_FILTER_H
#define MADGWICK_FILTER_H

#include <cmath>

class MadgwickFilter {
private:
    float _beta;            // 알고리즘 게인
    float _q0, _q1, _q2, _q3; // 쿼터니언
    float _invSampleFreq;   // 샘플링 주기의 역수 (1/dt)

    void _normalize();      // 내부 정규화 함수

public:
    MadgwickFilter(float freq, float b);
    
    // [6축 모드] 가속도 + 자이로 (빠른 주기로 호출)
    // [6-Axis Mode] Accel + Gyro (Call at high frequency)
    void update(float gx, float gy, float gz, float ax, float ay, float az);

    // [9축 모드] 가속도 + 자이로 + 지자기 (데이터가 있을 때 호출)
    // [9-Axis Mode] Accel + Gyro + Mag (Call when mag data is available)
    void update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);

    void getQuatarian(float& w, float& x, float& y, float& z);
};

#endif