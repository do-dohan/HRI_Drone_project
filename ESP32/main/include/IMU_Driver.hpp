#ifndef IMU_DRIVER_HPP
#define IMU_DRIVER_HPP

#include <cstdint>
#include "esp_err.h"
#include "esp_timer.h" // 타임스탬프를 위해 반드시 포함해야 함 / Must include for timestamps.

class IMU_Driver {
public:
    // 데이터를 한 번에 묶어서 관리할 구조체
    // Structure to bundle data for easier management
    struct IMU_Data {
        int16_t ax, ay, az;
        int16_t gx, gy, gz;
        uint32_t timestamp; // 데이터 읽은 시점 (us)
    };

    explicit IMU_Driver(uint8_t sda, uint8_t scl, uint8_t addr);
    bool begin();
    void update(); // 센서 읽기 + 타임스탬프 기록
    
    // 인라인 함수로 오버헤드 없이 데이터 반환
    // Return data without overhead using inline functions
    inline IMU_Data get_data() const { return _data; }

    // 에러 상태를 확인할 수 있는 게터 함수 추가
    // Getter function to check the error status
    esp_err_t get_last_error() const { return _err; }

private:
    uint8_t _sda_pin;          // I2C 데이터 핀 번호
    // I2C Serial Data pin number
    
    uint8_t _scl_pin;          // I2C 클럭 핀 번호
    // I2C Serial Clock pin number
    
    uint8_t _device_address;   // 센서의 I2C 슬레이브 주소
    // I2C slave address of the sensor
    
    bool _is_initialized;      // 센서 초기화 완료 여부
    // Flag indicating if the sensor is successfully initialized
    
    static bool _is_i2c_installed; // I2C 드라이버 설치 여부 (공유 리소스)
    // Static flag to check if the I2C driver is installed

    esp_err_t _err;            // 최신 에러 상태 저장 변수
    // Variable to store the latest error status

    IMU_Data _data; // 내부 저장소 / Internal storage
    // 가속도 원시 데이터 (X, Y, Z)
    // Raw accelerometer data for X, Y, and Z axes
    // 자이로 원시 데이터 (X, Y, Z)
    // Raw gyroscope data for X, Y, and Z axes
    //timestamp 데이터 읽은 시점
};

#endif