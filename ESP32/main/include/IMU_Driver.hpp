#ifndef IMU_DRIVER_HPP
#define IMU_DRIVER_HPP

#include <cstdint>
#include "esp_err.h"

class IMU_Driver {
private:
    uint8_t _sda_pin;          // I2C 데이터 핀 번호
    // I2C Serial Data pin number
    
    uint8_t _scl_pin;          // I2C 클럭 핀 번호
    // I2C Serial Clock pin number
    
    uint8_t _device_address;   // 센서의 I2C 슬레이브 주소
    // I2C slave address of the sensor
    
    int16_t _ax, _ay, _az;     // 가속도 원시 데이터 (X, Y, Z)
    // Raw accelerometer data for X, Y, and Z axes
    
    int16_t _gx, _gy, _gz;     // 자이로 원시 데이터 (X, Y, Z)
    // Raw gyroscope data for X, Y, and Z axes
    
    bool _is_initialized;      // 센서 초기화 완료 여부
    // Flag indicating if the sensor is successfully initialized
    
    static bool _is_i2c_installed; // I2C 드라이버 설치 여부 (공유 리소스)
    // Static flag to check if the I2C driver is installed

    esp_err_t _err;            // 최신 에러 상태 저장 변수
    // Variable to store the latest error status
    
public:
    explicit IMU_Driver(uint8_t sda, uint8_t scl, uint8_t addr);
    bool begin();
    void update();
    void get_accel_raw(int16_t &x, int16_t &y, int16_t &z);
    void get_gyro_raw(int16_t &x, int16_t &y, int16_t &z);    
    
    // 에러 상태를 확인할 수 있는 게터 함수 추가
    // Getter function to check the error status
    esp_err_t get_last_error() const { return _err; }
};

#endif