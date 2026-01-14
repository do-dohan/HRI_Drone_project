#include "HRI_Drone_HPP/Madgwick_Filter.hpp"

MadgwickFilter::MadgwickFilter(float freq, float b){
        this->invSampleFreq = 1.0f / freq;
        beta = b;
        q0 = 1.0f;
        q1 = 0.0f;
        q2 = 0.0f;
        q3 = 0.0f;
    }
    
void MadgwickFilter::update(float gx, float gy, float gz,  
                float ax, float ay, float az,
                float mx, float my, float mz){
        float s0 = 0.0f, s1 = 0.0f, s2 = 0.0f, s3 = 0.0f;
        //Initialize_Gradient_Vector
        float qDot0, qDot1, qDot2, qDot3;
        //Quatarian_Dif Vari
        float _2q0mx, _2q0my, _2q0mz, _2q1mx;
        float _2bx, _2bz, _4bx, _4bz;
        float _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3;
        float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
        float hx, hy, bx, bz;
        //Pre_computed constant
        
        //Common_Sub-expression_Elimination
        _2q0 = 2.0f * q0; _2q1 = 2.0f * q1; _2q2 = 2.0f * q2; _2q3 = 2.0f * q3;
        _2q0q2 = 2.0f * q0 * q2; _2q2q3 = 2.0f * q2 * q3;
        _2q2 = 2.0f * q2; _2q3 = 2.0f * q3; _2q0q2 = 2.0f * q0 * q2; _2q2q3 = 2.0f * q2 * q3;
        q0q0 = q0 * q0; q0q1 = q0 * q1; q0q2 = q0 * q2; q0q3 = q0 * q3;
        q1q1 = q1 * q1; q1q2 = q1 * q2; q1q3 = q1 * q3; q2q2 = q2 * q2; q2q3 = q2 * q3; q3q3 = q3 * q3;
        
        //step1 Gyroscope_prediction
        qDot0 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
        qDot1 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
        qDot2 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
        qDot3 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);
        
        //step2 Data_correction_using_accelerometer
        //Accelerometer_Correction_(Gravity Vector Alignment)
        float Ac_norm = std::sqrt(ax * ax + ay * ay + az * az);
        float M_norm = std::sqrt(mx * mx + my * my + mz * mz);
        //Validation_using_Ac_norm/0.1f and 10.0f After experimental tuning
        if (Ac_norm > 0.1f && Ac_norm < 10.0f) {
            //Accerlation_Nomalize
            float invAc_norm = 1.0f / Ac_norm;
            ax *= invAc_norm;
            ay *= invAc_norm;
            az *= invAc_norm;
            
            //Gradient_Descent
            s0 += -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - ay);
            s1 += _2q3 * (2.0f * q1q3 - _2q0q2 - ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - ay) - 
            4.0f * q1 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az);
            s2 += -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - ay) - 
            4.0f * q2 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az);
            s3 += _2q1 * (2.0f * q1q3 - _2q0q2 - ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - ay);
        }
            
        //step3 Data_correction_using_Geomagnetic Field   
        //Direction_Calculation
        //Validation_using_M_norm/0.1f and 10.0f After experimental tuning
        if (M_norm > 0.01f && M_norm < 10.0f){
            
            //Geomagnetic_Nomalize
            float invM_norm = 1.0f / M_norm;
            mx *= invM_norm;
            my *= invM_norm;
            mz *= invM_norm;
            
            //Common_Sub-expression_Elimination
            _2q0mx = 2.0f * q0 * mx; _2q0my = 2.0f * q0 * my; _2q0mz = 2.0f * q0 * mz;
            _2q1mx = 2.0f * q1 * mx; _2q0 = 2.0f * q0; _2q1 = 2.0f * q1;

            //Earth's_magnetic_field_vector_correction
            hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1 + _2q1 * my * q2 + _2q1 * mz * q3 - mx * q2q2 - mx * q3q3;
            hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1 * mx * q2 - my * q1q1 + my * q2q2 + _2q2 * mz * q3 - my * q3q3;
            bx = std::sqrt(hx * hx + hy * hy); // 수평 성분
            bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1 * mx * q3 - mz * q1q1 + _2q2 * my * q3 - mz * q2q2 + mz * q3q3; // 수직 성분
            _2bx = 2.0f * bx; _2bz = 2.0f * bz; _4bx = 4.0f * bx; _4bz = 4.0f * bz;

            //Gradient_Descent
            s0 += -_2bz * q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + 
            (_2bx * q1 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + 
            _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
            s1 += _2bz * q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + 
            (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + 
            (_2bx * q3 - _4bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
            s2 += (-_4bx * q2 - _2bz * q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * 
            (q1q3 - q0q2) - mx) + (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + 
            _2bz * (q0q1 + q2q3) - my) + (_2bx * q0 - _4bz * q2) * (_2bx * (q0q2 + q1q3) + 
            _2bz * (0.5f - q1q1 - q2q2) - mz);
            s3 += (-_4bx * q3 + _2bz * q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * 
            (q1q3 - q0q2) - mx) + (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + 
            _2bz * (q0q1 + q2q3) - my) + _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * 
            (0.5f - q1q1 - q2q2) - mz);
        }
            
        //Error_Vector_Normalization
        float S_norm = std::sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
        std::cout << S_norm << std::endl;
        if (S_norm > 0.0001f){
            float InvSnorm = 1.0f / S_norm;
            s0 *= InvSnorm;
            s1 *= InvSnorm;
            s2 *= InvSnorm;
            s3 *= InvSnorm;
                
            //Gyro_Prediction_Error_Correction
            qDot0 -= beta * s0;
            qDot1 -= beta * s1;
            qDot2 -= beta * s2;
            qDot3 -= beta * s3;
        }
    //Final_Integral
    q0 += qDot0 * invSampleFreq;
    q1 += qDot1 * invSampleFreq;
    q2 += qDot2 * invSampleFreq;
    q3 += qDot3 * invSampleFreq;
    normalize();                
    }
    
void MadgwickFilter::normalize(){
        float norm = std::sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
        if (norm > 0.0001f){
            float Invnorm = 1.0f / norm;
            q0 *= Invnorm;
            q1 *= Invnorm;
            q2 *= Invnorm;
            q3 *= Invnorm;
        }
    }
    
void MadgwickFilter::getQuatarian(float& w, float& x, float& y, float& z){
        w = q0; x = q1; y = q2; z=q3;
}

