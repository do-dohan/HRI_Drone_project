#include "Flex_Driver.hpp"
#include "esp_timer.h"
#include "driver/adc.h"

// 생성자: 멤버 변수들을 초기화함 (_ 접두사 적용)
// Constructor: Initializes member variables (Applying '_' prefix)
Flex_Driver::Flex_Driver(uint8_t pin) 
    : _pin_number(pin), _current_raw_flex(0), _current_flex_time(0) {
}

bool Flex_Driver::begin() {
    // ADC 해상도를 12비트(0~4095)로 설정함
    // Set ADC resolution to 12-bit (range 0 to 4095)
    adc1_config_width(ADC_WIDTH_BIT_12);

    // ADC 입력 감쇠율을 11dB로 설정 (최대 약 3.3V까지 측정 가능)
    // Set ADC attenuation to 11dB (Allows measurement up to approx. 3.3V)
    adc1_config_channel_atten(ADC1_CHANNEL_6, ADC_ATTEN_DB_11);
    
     return true; // 설정 성공 / Configuration successful
}

void Flex_Driver::update() {
    // 현재 시스템 부팅 후 경과 시간(us)을 저장함
    // Store the elapsed time (microseconds) since system boot
    this->_current_flex_time = (uint32_t)esp_timer_get_time();

    // 지정된 ADC 채널에서 아날로그 원시 값을 읽어옴
    // Read the raw analog value from the specified ADC channel
    this->_current_raw_flex = (int16_t)adc1_get_raw(ADC1_CHANNEL_6);    
}

void Flex_Driver::get_flex_data(int16_t &raw_flex, uint32_t &flex_time) {
    // 외부 참조 변수에 현재 읽어온 데이터와 시간을 복사함
    // Copy the current raw data and timestamp to external reference variables
    raw_flex = this->_current_raw_flex;
    flex_time = this->_current_flex_time;
}