#include "hri_signal_pkg/HRI_Drone_HPP/Quaternion_to_Euler.hpp"

// [KOR] PI 상수(표준 M_PI 의존 제거).
// [ENG] PI constant (removes dependency on non-standard M_PI).
static constexpr float kPi = 3.14159265358979323846f;

// [KOR] rad -> deg 변환 상수.
// [ENG] rad -> deg conversion constant.
static constexpr float kRadToDeg = 180.0f / kPi;

// [KOR] conjugate 구현 시작.
// [ENG] Begin conjugate implementation.
Quaternion_to_Euler::Quat Quaternion_to_Euler::conjugate(const Quat& q) const {
// [KOR] 벡터부(x,y,z)를 부호 반전하고 스칼라부(w)는 유지한다.
// [ENG] Negate vector part (x,y,z) and keep scalar part (w).
    return Quat{-q.x, -q.y, -q.z, q.w};
// [KOR] conjugate 구현 종료.
// [ENG] End conjugate implementation.
}

// [KOR] Hamilton_Product 구현 시작.
// [ENG] Begin Hamilton_Product implementation.
Quaternion_to_Euler::Quat Quaternion_to_Euler::Hamilton_Product(const Quat& a, const Quat& b) const {
// [KOR] 쿼터니언 곱은 결합법칙은 성립하지만 교환법칙은 성립하지 않는다.
// [ENG] Quaternion multiplication is associative but not commutative.
    return Quat{
        // [KOR] x 성분 계산.
        // [ENG] Compute x component.
        a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
        // [KOR] y 성분 계산.
        // [ENG] Compute y component.
        a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
        // [KOR] z 성분 계산.
        // [ENG] Compute z component.
        a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
        // [KOR] w 성분 계산.
        // [ENG] Compute w component.
        a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z
    };
// [KOR] Hamilton_Product 구현 종료.
// [ENG] End Hamilton_Product implementation.
}

// [KOR] get_Relative 구현 시작.
// [ENG] Begin get_Relative implementation.
Quaternion_to_Euler::Quat Quaternion_to_Euler::get_Relative(const Quat& q_ref, const Quat& q_target) const {
// [KOR] 기준 쿼터니언의 켤레를 구한다(단위쿼터니언이면 역과 동일).
// [ENG] Compute conjugate of reference (equals inverse if unit quaternion).
    Quat q_ref_inv = conjugate(q_ref);
// [KOR] q_rel = q_ref_inv ⊗ q_target로 상대 회전을 계산한다.
// [ENG] Compute relative rotation as q_rel = q_ref_inv ⊗ q_target.
    return Hamilton_Product(q_ref_inv, q_target);
// [KOR] get_Relative 구현 종료.
// [ENG] End get_Relative implementation.
}

// [KOR] toEuler(Quat) 오버로드 구현 시작.
// [ENG] Begin toEuler(Quat) overload.
Quaternion_to_Euler::Euler_Angles Quaternion_to_Euler::toEuler(const Quat& q) const {
// [KOR] 내부 기본 함수로 위임한다.
// [ENG] Delegate to the base toEuler function.
    return toEuler(q.x, q.y, q.z, q.w);
// [KOR] toEuler(Quat) 오버로드 구현 종료.
// [ENG] End toEuler(Quat) overload.
}

// [KOR] toEuler(x,y,z,w) 구현 시작.
// [ENG] Begin toEuler(x,y,z,w) implementation.
Quaternion_to_Euler::Euler_Angles Quaternion_to_Euler::toEuler(float x, float y, float z, float w) const {
// [KOR] 결과 오일러 각도 구조체를 선언한다.
// [ENG] Declare result Euler angles struct.
    Euler_Angles angles{};
// [KOR] roll 계산을 위한 sinr_cosp를 계산한다.
// [ENG] Compute sinr_cosp for roll.
    float sinr_cosp = 2.0f * (w * x + y * z);
// [KOR] roll 계산을 위한 cosr_cosp를 계산한다.
// [ENG] Compute cosr_cosp for roll.
    float cosr_cosp = 1.0f - 2.0f * (x * x + y * y);
// [KOR] roll = atan2(sin,cos) 를 degree로 변환한다.
// [ENG] roll = atan2(sin,cos) converted to degrees.
    angles.roll = std::atan2(sinr_cosp, cosr_cosp) * kRadToDeg;

// [KOR] pitch 계산을 위한 sinp를 계산한다.
// [ENG] Compute sinp for pitch.
    float sinp = 2.0f * (w * y - z * x);
// [KOR] sinp가 [-1,1]을 살짝 벗어나는 수치오차를 대비해 분기한다.
// [ENG] Branch to handle numeric drift where sinp slightly exceeds [-1,1].
    if (std::fabs(sinp) >= 1.0f) {
// [KOR] 짐벌락 근처에서는 ±90deg로 고정한다.
// [ENG] Near gimbal lock, clamp to ±90deg.
        angles.pitch = std::copysign(kPi * 0.5f, sinp) * kRadToDeg;
// [KOR] else 분기 시작(일반 영역).
// [ENG] Else branch begins (normal region).
    } else {
// [KOR] 일반 영역에서는 asin(sinp)로 pitch를 계산한다.
// [ENG] In normal region, compute pitch via asin(sinp).
        angles.pitch = std::asin(sinp) * kRadToDeg;
// [KOR] else 분기 종료.
// [ENG] End else branch.
    }

// [KOR] yaw 계산을 위한 siny_cosp를 계산한다.
// [ENG] Compute siny_cosp for yaw.
    float siny_cosp = 2.0f * (w * z + x * y);
// [KOR] yaw 계산을 위한 cosy_cosp를 계산한다.
// [ENG] Compute cosy_cosp for yaw.
    float cosy_cosp = 1.0f - 2.0f * (y * y + z * z);
// [KOR] yaw = atan2(sin,cos) 를 degree로 변환한다.
// [ENG] yaw = atan2(sin,cos) converted to degrees.
    angles.yaw = std::atan2(siny_cosp, cosy_cosp) * kRadToDeg;

// [KOR] 계산된 오일러 각도를 반환한다.
// [ENG] Return computed Euler angles.
    return angles;
// [KOR] toEuler 구현 종료.
// [ENG] End toEuler implementation.
}