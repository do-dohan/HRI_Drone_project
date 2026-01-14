#ifndef MADGWICK_FILTER_H
#define MADGWICK_FILTER_H

#include <iostream>
#include <cmath>

class MadgwickFilter {
private:
    float beta;
    float q0, q1, q2, q3;
    float invSampleFreq;
    void normalize();
    
public:
    MadgwickFilter(float freq, float b);
    
    void update(float gx, float gy, float gz,  
                float ax, float ay, float az,
                float mx, float my, float mz);
    void getQuatarian(float& w, float& x, float& y, float& z);
};

#endif