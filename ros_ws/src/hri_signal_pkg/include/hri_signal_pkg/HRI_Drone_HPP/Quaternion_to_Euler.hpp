#ifndef QUATERNION_TO_EULER_HPP
#define QUATERNION_TO_EULER_HPP

#include <cmath>

// [KOR] 쿼터니언을 오일러 각도로 변환하는 유틸리티 클래스다.
// [ENG] Utility class that converts quaternions to Euler angles.
class Quaternion_to_Euler {
public:
    // [KOR] 쿼터니언 구조체(x,y,z,w)로 저장한다.
    // [ENG] Quaternion struct stored as (x,y,z,w).
    struct Quat { float x, y, z, w; };

    // [KOR] 오일러 각도 구조체(roll,pitch,yaw)이며 단위는 degree로 반환한다.
    // [ENG] Euler angles struct (roll,pitch,yaw) returned in degrees.
    struct Euler_Angles { float roll, pitch, yaw; };

public:
    // [KOR] 생성자(상태 없음).
    // [ENG] Constructor (stateless).
    Quaternion_to_Euler() = default;

    // [KOR] 켤레(conjugate) 계산: (-x,-y,-z,w).
    // [ENG] Conjugate: (-x,-y,-z,w).
    Quat conjugate(const Quat& q) const;

    // [KOR] 해밀턴 곱(Hamilton Product) 계산: a ⊗ b.
    // [ENG] Hamilton product: a ⊗ b.
    Quat Hamilton_Product(const Quat& a, const Quat& b) const;

    // [KOR] 상대 쿼터니언 계산: q_rel = conj(q_ref) ⊗ q_target (단, q_ref가 단위쿼터니언일 때 역과 동일).
    // [ENG] Relative quaternion: q_rel = conj(q_ref) ⊗ q_target (equals inverse only if q_ref is unit quaternion).
    Quat get_Relative(const Quat& q_ref, const Quat& q_target) const;

    // [KOR] (x,y,z,w) 쿼터니언을 Euler(roll,pitch,yaw)로 변환한다.
    // [ENG] Converts quaternion (x,y,z,w) to Euler (roll,pitch,yaw).
    Euler_Angles toEuler(float x, float y, float z, float w) const;

    // [KOR] Quat 타입 입력을 바로 Euler로 변환한다.
    // [ENG] Converts Quat directly to Euler.
    Euler_Angles toEuler(const Quat& q) const;
};

// [KOR] 헤더 가드 종료.
// [ENG] Header guard end.
#endif