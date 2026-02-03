#include "hri_signal_pkg/Quaternion_to_Euler_Utils.hpp"

/**
 * @brief 두 쿼터니언의 해밀턴 곱을 계산함
 * @details Calculates the Hamilton Product of two quaternions (Combined Rotation)
 */
Quaternion_to_Euler::Quat Quaternion_to_Euler::Hamilton_Product(Quat a, Quat b) {
    // 쿼터니언 곱셈 수식 적용 (결합법칙 성립, 교환법칙 불성립)
    // Apply quaternion multiplication formula (Associative, but non-commutative)
    return {
        a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
        a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
        a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
        a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z
    };
}

/**
 * @brief 기준 자세에 대한 현재 자세의 상대적 회전량을 계산함
 * @details Calculates relative rotation from reference to target orientation
 */
Quaternion_to_Euler::Quat Quaternion_to_Euler::get_Relative(Quat q_ref, Quat q_target) {
    // 기준 쿼터니언의 역회전(켤레)을 구함
    // Get the inverse rotation (conjugate) of the reference quaternion
    Quat q_ref_inv = conjugate(q_ref);
    
    // 역회전과 목표 회전을 곱하여 상대적 차이를 구함
    // Multiply inverse and target to find the relative difference
    return Hamilton_Product(q_ref_inv, q_target);
}

/**
 * @brief 쿼터니언을 오일러 각도(Roll, Pitch, Yaw)로 변환함
 * @details Converts Quaternion to Euler angles in Degrees
 */
Quaternion_to_Euler::Euler_Angles Quaternion_to_Euler::toEuler(float x, float y, float z, float w) {
    Euler_Angles angles;

    // [Roll 계산] X축 주위의 회전
    // [Roll calculation] Rotation around X-axis
    float sinr_cosp = 2 * (w * x + y * z);
    float cosr_cosp = 1 - 2 * (x * x + y * y);
    angles.roll = std::atan2(sinr_cosp, cosr_cosp) * this->_Rad_to_Deg;

    // [Pitch 계산] Y축 주위의 회전 (짐벌 락 방지 로직 포함)
    // [Pitch calculation] Rotation around Y-axis (Includes Gimbal Lock protection)
    float sinp = 2 * (w * y - z * x);
    if (std::abs(sinp) >= 1)
        // 90도 직립 상태 처리 / Handle 90-degree upright case
        angles.pitch = std::copysign(M_PI / 2, sinp) * this->_Rad_to_Deg;
    else
        // 일반적인 경우 / Normal case
        angles.pitch = std::asin(sinp) * this->_Rad_to_Deg;

    // [Yaw 계산] Z축 주위의 회전 (방위각)
    // [Yaw calculation] Rotation around Z-axis (Heading)
    float siny_cosp = 2 * (w * z + x * y);
    float cosy_cosp = 1 - 2 * (y * y + z * z);
    angles.yaw = std::atan2(siny_cosp, cosy_cosp) * this->_Rad_to_Deg;

    return angles;
}