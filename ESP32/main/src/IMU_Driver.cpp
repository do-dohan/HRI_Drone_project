#include "IMU_Driver.hpp"

// I2C 포트 번호 정의 (ESP32 기본 포트 0번 사용)
// Define I2C port number (Using default port 0 for ESP32)
#define I2C_MASTER_NUM I2C_NUM_0

// 정적 변수 초기화
// Initialize static member variable
bool IMU_Driver::_is_i2c_installed = false;

IMU_Driver::IMU_Driver(uint8_t sda, uint8_t scl, uint8_t addr):
    _sda_pin(sda), _scl_pin(scl), _device_address(addr), _isMPU9250(false), _is_initialized(false), _err(ESP_OK) {
    // 모든 축 데이터 초기화
    // Initialize all axes data to zero
    _data.ax = _data.ay = _data.az = 0;
    _data.gx = _data.gy = _data.gz = 0;
    _data.timestamp = 0;
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

    // 1. 장치 식별 확인
        // Check device identity
        uint8_t deviceID = readRegister(WHO_AM_I_REG);

        if(deviceID == 0x68) {
            //MPU 6050
            this->_isMPU9250 = false;
        }
        else if (deviceID == 0x71) {
            //MPU 9250
            this->_isMPU9250 = true;

            // [중요] 지자계 센서(AK8963) 접근을 위한 Bypass Mode 활성화
            // [CRITICAL] Enable Bypass Mode to access AK8963 Magnetometer
            writeRegister(INT_PIN_CFG, 0x02);
        }
        else {
            return false;
        }
    
    // 센서 깨우기 명령 전송 (Sleep Mode 해제)
    // Send wake-up command (Disable Sleep Mode)
    writeRegister(PWR_MGMT_1, 0x01);
    
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

    // 데이터 읽기 직전에 타임스탬프를 찍습니다.
    // Take a timestamp right before reading data.
    this->_data.timestamp = (uint32_t)esp_timer_get_time();
    
    // 센서로부터 가속도/자이로 데이터 14바이트를 한 번에 읽음
    // Read 14 bytes of accel/gyro data from the sensor at once
    this->_err = i2c_master_write_read_device(I2C_MASTER_NUM, _device_address, &start_reg,
                                 1, buffer, 14, 100 / portTICK_PERIOD_MS);
    
    // 읽기 실패 시 데이터 업데이트를 건너뜀 (Fail-safe)
    // If reading fails, skip data update (Fail-safe)
    if (this->_err != ESP_OK) return;

    // 비트 연산을 통해 8비트 데이터 2개를 16비트 정수로 합침
    // Combine two 8-bit data into a 16-bit integer using bitwise operations
    this->_data.ax = (int16_t) ((buffer[0] << 8) | buffer[1]);
    this->_data.ay = (int16_t) ((buffer[2] << 8) | buffer[3]);
    this->_data.az = (int16_t) ((buffer[4] << 8) | buffer[5]);
    
    this->_data.gx = (int16_t) ((buffer[8] << 8) | buffer[9]);
    this->_data.gy = (int16_t) ((buffer[10] << 8) | buffer[11]);
    this->_data.gz = (int16_t) ((buffer[12] << 8) | buffer[13]);
}

/*
i2c_master_write_read_device(
    I2C_MASTER_NUM,   // 1. 누가 (ESP32 포트 번호)
    _device_address,  // 2. 누구에게 (센서 주소, 예: 0x71)
    &reg,             // 3. 무엇을 (읽고 싶은 레지스터 주소, 예: 0x75)
    1,                // 4. 레지스터 주소 길이 (1 byte)
    &value,           // 5. 어디에 담을까 (결과를 저장할 변수 주소)
    1,                // 6. 얼마나 읽을까 (1 byte)
    timeout           // 7. 기다릴 시간
);
*/

uint8_t IMU_Driver::readRegister(uint8_t reg) {
    uint8_t value = 0;
    i2c_master_write_read_device(I2C_MASTER_NUM, _device_address, &reg, 1, &value, 1, 100 / portTICK_PERIOD_MS);
    return value;
}

void IMU_Driver::writeRegister(uint8_t reg, uint8_t data) {
    uint8_t write_buf[2] = {reg, data};
    this->_err = i2c_master_write_to_device(I2C_MASTER_NUM, _device_address, write_buf, 2, 100 / portTICK_PERIOD_MS);
}