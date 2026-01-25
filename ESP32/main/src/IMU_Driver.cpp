#include "IMU_Driver.hpp"
#include "driver/i2c.h"
#include "esp_timer.h"

// I2C 포트 번호 정의 (ESP32 기본 포트 0번 사용)
// Define I2C port number (Using default port 0 for ESP32)
#define I2C_MASTER_NUM I2C_NUM_0

// 정적 변수 초기화
// Initialize static member variable
bool IMU_Driver::_is_i2c_installed = false;

IMU_Driver::IMU_Driver(uint8_t sda, uint8_t scl, uint8_t addr):
    _sda_pin(sda), _scl_pin(scl), _device_address(addr), _is_initialized(false), _err(ESP_OK) {
    // 모든 축 데이터 초기화
    // Initialize all axes data to zero
    _ax = _ay = _az = 0;
    _gx = _gy = _gz = 0;
}

bool IMU_Driver::begin() {
    // I2C 드라이버가 설치되지 않았다면 설정 진행
    // If I2C driver is not installed, proceed with configuration
    if (_is_i2c_installed == false) {
        i2c_config_t conf = {};
        conf.mode = I2C_MODE_MASTER;
        conf.sda_io_num = (gpio_num_t)_sda_pin;
        conf.scl_io_num = (gpio_num_t)_scl_pin;
        conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
        conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
        conf.master.clk_speed = 400000;
    
        // 설정 적용 및 드라이버 설치 후 에러 확인
        // Apply config and check error after installing driver
        this->_err = i2c_param_config(I2C_MASTER_NUM, &conf);
        if (this->_err != ESP_OK) return false;

        this->_err = i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
        if (this->_err == ESP_OK) {
            _is_i2c_installed = true;
        } else {
            return false;
        }
    }
    
    // 센서 깨우기 명령 전송 (0x6B 레지스터에 0x01 쓰기)
    // Send wake-up command (Write 0x01 to register 0x6B)
    uint8_t wake_cmd[2] = {0x6B, 0x01};
    this->_err = i2c_master_write_to_device(I2C_MASTER_NUM, _device_address, wake_cmd,
                                               2, 1000 / portTICK_PERIOD_MS);
    
    if (this->_err == ESP_OK) {
        _is_initialized = true;
    }
    return _is_initialized;
}

void IMU_Driver::update() {
    // 초기화되지 않았다면 실행 중단
    // Abort execution if not initialized
    if (!_is_initialized) return;
    
    uint8_t start_reg = 0x3B; // 가속도 데이터 시작 주소
    uint8_t buffer[14];       // 데이터를 담을 버퍼
    
    // 센서로부터 가속도/자이로 데이터 14바이트를 한 번에 읽음
    // Read 14 bytes of accel/gyro data from the sensor at once
    this->_err = i2c_master_write_read_device(I2C_MASTER_NUM, _device_address, &start_reg,
                                 1, buffer, 14, 100 / portTICK_PERIOD_MS);
    
    // 읽기 실패 시 데이터 업데이트를 건너뜀 (Fail-safe)
    // If reading fails, skip data update (Fail-safe)
    if (this->_err != ESP_OK) return;

    // 비트 연산을 통해 8비트 데이터 2개를 16비트 정수로 합침
    // Combine two 8-bit data into a 16-bit integer using bitwise operations
    this->_ax = (int16_t) ((buffer[0] << 8) | buffer[1]);
    this->_ay = (int16_t) ((buffer[2] << 8) | buffer[3]);
    this->_az = (int16_t) ((buffer[4] << 8) | buffer[5]);
    
    this->_gx = (int16_t) ((buffer[8] << 8) | buffer[9]);
    this->_gy = (int16_t) ((buffer[10] << 8) | buffer[11]);
    this->_gz = (int16_t) ((buffer[12] << 8) | buffer[13]);
}

void IMU_Driver::get_accel_raw(int16_t &x, int16_t &y, int16_t &z) {
    // 저장된 가속도 데이터를 참조로 반환
    // Return stored accelerometer data via reference
    x = this->_ax;
    y = this->_ay;
    z = this->_az;
}

void IMU_Driver::get_gyro_raw(int16_t &x, int16_t &y, int16_t &z) {
    // 저장된 자이로 데이터를 참조로 반환
    // Return stored gyroscope data via reference
    x = this->_gx;
    y = this->_gy;
    z = this->_gz;
}