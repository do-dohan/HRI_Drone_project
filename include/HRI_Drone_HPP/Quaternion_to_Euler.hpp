#ifndef QUATERNION_TO_EULER_UTILS_HPP
#define QUATERNION_TO_EULER_UTILS_HPP

#include <cmath>

/**
 * @brief 쿼터니언을 오일러 각도로 변환하는 유틸리티 클래스
 * @details Utility class to convert Quaternions to Euler angles (Roll, Pitch, Yaw)
 */
class Quaternion_to_Euler {
private:
    // 라디안을 도(Degree) 단위로 변환하기 위한 상수
    // Constant for converting Radians to Degrees
    const float _Rad_to_Deg = 180.0f / M_PI;

public:
    /**
     * @brief 쿼터니언(4원수) 구조체
     * @details Quaternion structure (x, y, z, w)
     */
    struct Quat { float x, y, z, w; };

    /**
     * @brief 오일러 각도 구조체
     * @details Euler angles structure (roll, pitch, yaw)
     */
    struct Euler_Angles { float roll, pitch, yaw; };

    /**
     * @brief 쿼터니언의 켤레(Conjugate)를 계산함
     * @param q 원본 쿼터니언
     * @return 켤레 쿼터니언 (-x, -y, -z, w)
     */
    Quat conjugate(Quat q) {
        return {-q.x, -q.y, -q.z, q.w};
    }

    /**
     * @brief 두 쿼터니언의 해밀턴 곱(Hamilton Product)을 계산함
     * @details Calculates the Hamilton Product of two quaternions
     */
    Quat Hamilton_Product(Quat a, Quat b);

    /**
     * @brief 기준 자세 대비 목표 자세의 상대적 쿼터니언을 계산함
     * @details Calculates the relative quaternion of target orientation from reference
     */
    Quat get_Relative(Quat q_ref, Quat q_target);

    // 생성자 / Constructor
    Quaternion_to_Euler() {}

    /**
     * @brief 쿼터니언 데이터를 Roll, Pitch, Yaw 각도로 변환함
     * @return Euler_Angles (단위: Degree)
     */
    Euler_Angles toEuler(float x, float y, float z, float w);
};

#endif