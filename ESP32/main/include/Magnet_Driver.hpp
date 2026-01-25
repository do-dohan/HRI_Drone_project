#ifndef MAGNET_DRIVER_HPP
#define MAGNET_DRIVER_HPP

#include <cmath>
#include "esp_err.h" // ESP-IDF 에러 타입 정의 / ESP-IDF error type definitions

class Magnet_Driver {
public:
    /**
     * @brief 3축 자기장 데이터를 담는 구조체
     * @details Structure to hold 3-axis magnetic field data
     */
    struct Magnet_Vector { float x, y, z; };

    /**
     * @brief 생성자: I2C 주소를 설정함
     * @param addr 센서의 I2C 슬레이브 주소 (기본값 0x1E)
     * @details Constructor: Sets the I2C slave address (Default 0x1E)
     */
    explicit Magnet_Driver(uint8_t addr = 0x1E);

    /**
     * @brief I2C 드라이버 설치 및 센서 초기화
     * @param sda_pin SDA 핀 번호 / SDA pin number
     * @param scl_pin SCL 핀 번호 / SCL pin number
     * @return 초기화 성공 여부 / Initialization success status
     */
    bool begin(int sda_pin, int scl_pin);

    /**
     * @brief 보정값 설정: Hard Iron 및 Soft Iron 보정 계수 적용
     * @param hard_iron 오프셋 보정 배열 (Hard Iron bias)
     * @param soft_iron 왜곡 보정 행렬 (Soft Iron matrix)
     * @details Set calibration values: Apply Hard Iron and Soft Iron coefficients
     */
    void set_calibration(float hard_iron[3], float soft_iron[3][3]);

    /**
     * @brief 데이터 읽기 및 보정 수행
     * @return 보정된 3축 자기장 벡터
     * @details Read data and apply calibration. Returns calibrated 3-axis vector
     */
    Magnet_Vector read();

    /**
     * @brief 최신 통신 에러 상태를 반환함
     * @return esp_err_t 타입의 에러 코드
     * @details Returns the latest communication error status
     */
    esp_err_t get_last_error() const { return this->_last_err; }

private:
    uint8_t _addr;              // 센서의 I2C 슬레이브 주소
                                // I2C slave address of the sensor
    
    esp_err_t _last_err;        // 최신 에러 상태 저장 변수
                                // Variable to store the latest error status

    float _hard_iron[3];        // 하드 아이언(편향) 보정값
                                // Hard iron (bias) calibration values
    
    float _soft_iron[3][3];     // 소프트 아이언(왜곡) 보정 행렬
                                // Soft iron (distortion) calibration matrix

    /**
     * @brief 레지스터 쓰기 내부 함수
     * @param reg 대상 레지스터 주소 / Target register address
     * @param val 기록할 값 / Value to write
     * @return 통신 결과 (esp_err_t) / Communication result
     */
    esp_err_t _write_reg(uint8_t reg, uint8_t val);
};

#endif