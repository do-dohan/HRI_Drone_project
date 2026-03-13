#include "Manual_Permission_Gate.hpp"

Manual_Permission::Button_Output
//반환 타입 명시
Manual_Permission::evaluate(const Button_Input& input) const
//evaluate fuction define
{
    Button_Output output{};
    //Button_Output struct 생성
    if (!input.data_received) {
        //아직 filtered date 미수신시
        output.gate_pass = false;
        //Manual permission gate 통과 불가
        output.reason_bits = REASON_INITIAL_NO_FRAME;
        //fail 사유 초기 no frame
        output.reason_primary = PRIMARY_INITIAL_NO_FRAME;
        //primary 사유 초기 no frame
        return output;
    }

    if(!input.button_valid) {
        //아직 filtered date 미수신시
        output.gate_pass = false;
        //Manual permission gate 통과 불가
        output.reason_bits = REASON_MANUAL_INVALID;
        //fail 사유 Button invaild
        output.reason_primary = PRIMARY_MANUAL_INVALID;
        //primary 사유 Button invaild
        return output;
    }

    if(!input.button_on_off) {
        //아직 filtered date 미수신시
        output.gate_pass = false;
        //Manual permission gate 통과 불가
        output.reason_bits = REASON_MANUAL_OFF;
        //fail 사유 Button off
        output.reason_primary = PRIMARY_MANUAL_OFF;
        //primary 사유 Button off
        return output;
    }

    output.gate_pass = true;
    output.reason_bits = REASON_NONE;
    output.reason_primary = PRIMARY_QUALIFIED_PASS;

    return output;
}