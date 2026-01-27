#ifndef FLEX_DRIVER_HPP
#define FLEX_DRIVER_HPP

#include <cstdint>
#include "esp_timer.h"
#include "driver/adc.h"

/**
 * @brief 굴곡 센서(Flex Sensor) 드라이버 클래스
 * @details Flex Sensor Driver Class for reading finger bending data
 */
class Flex_Driver {
public:
    struct Flex_Data{
        int16_t raw_flex;
        uint32_t timestamp;
    };
    /**
     * @brief 생성자: 센서 연결 핀을 설정함
     * @param pin 사용할 ADC 핀 번호
     */
    Flex_Driver(uint8_t pin);

    /**
     * @brief 하드웨어 초기화: ADC 채널 및 감쇠(Attenuation) 설정
     * @return 성공 여부 (현재는 항상 true 반환)
     */
    bool begin();

    /**
     * @brief 데이터 업데이트: 현재 ADC 값과 시간을 읽어 저장함
     */
    void update();

    /**
     * @brief 데이터 획득: 외부에서 최신 굴곡 데이터와 시간을 가져옴
     * @param raw_flex 읽어온 원시 값을 저장할 참조 변수
     * @param flex_time 읽어온 시간을 저장할 참조 변수
     */
    inline Flex_Data get_data() const { return _data; }

private:
    uint8_t _pin_number;        // 센서가 연결된 ADC 핀 번호
    // ADC pin number where the sensor is connected
    
    bool _is_initialized;      // 센서 초기화 완료 여부
    // Flag indicating if the sensor is successfully initialized
    
    Flex_Data _data;
};

#endif