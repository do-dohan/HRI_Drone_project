#ifndef MANUAL_PERMISSION_GATE_HPP
#define MANUAL_PERMISSION_GATE_HPP

#include <cstdint>

class Manual_Permission {
public:
    struct Button_Input{
        bool button_on_off{false};
        //현재 Button의 on/off state(true=눌림/flase=해제)
        bool button_valid{false};
        //flags 해석을 통해 Button 비트의 유효성 평가
        bool data_received{false};
        //filtered data 최초 수신 여부 확인
    };

    struct Button_Output{
        bool gate_pass{false};
        //Manual permission gate의 통과 여부
        uint32_t reason_bits{0u};
        //Manual permission gate 내부 fail/pass 사유 bit mask
        uint8_t reason_primary{0u};
        //최종 판정 대표 사유
    };

    enum Reason_Bits : uint32_t
    {
        REASON_NONE = 0u,
        //정상 통과 상태
        REASON_MANUAL_OFF = 1u << 0,
        //Button off state
        REASON_MANUAL_INVALID = 1u << 1,
        //Button flags distrust
        REASON_INITIAL_NO_FRAME = 1u << 2
        //아직 Filtered data 미수신
    };

    enum Reason_Primary : uint8_t
    {
        PRIMARY_NONE = 0,
        //Evaluate 이전 state
        PRIMARY_MANUAL_OFF = 1,
        //Button off state
        PRIMARY_MANUAL_INVALID = 2,
        //Button flags distrust
        PRIMARY_INITIAL_NO_FRAME = 3,
        //아직 Filtered data 미수신
        PRIMARY_QUALIFIED_PASS = 10
        //Manual permission gate 정상 pass
    };

    Button_Output evaluate(const Button_Input& input) const;
};

#endif