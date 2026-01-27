#include "Magnet_Driver.hpp"
#include "driver/i2c.h"

#define I2C_MASTER_NUM I2C_NUM_0

// QMC5883L 전용 레지스터 주소 정의 (HMC와 완전히 다름)
// QMC5883L Specific Register addresses (Completely different from HMC)
#define QMC_REG_DATA_START  0x00 // 데이터 시작점 (X-LSB) / Data start (X-LSB)
#define QMC_REG_STATUS      0x06 // 상태 레지스터 / Status register
#define QMC_REG_CONTROL_1   0x09 // 제어 레지스터 1 (모드, 주사율 등) / Control register 1
#define QMC_REG_CONTROL_2   0x0A // 제어 레지스터 2 (소프트 리셋 등) / Control register 2
#define QMC_REG_FBR         0x0B // SET/RESET 주기 설정 / SET/RESET period

bool Magnet_Driver::_is_i2c_installed = false;

Magnet_Driver::Magnet_Driver(uint8_t sda, uint8_t scl, uint8_t addr) 
    : _sda_pin(sda), _scl_pin(scl),_addr(addr), _last_err(ESP_OK) {
    // 캘리브레이션 변수들을 기본값(보정 없음)으로 초기화
    // Initialize calibration variables to default values (no correction)
    
    // 초기화 여부 플래그도 명시적으로 false 설정 (헤더에 선언되어 있다면)
    _is_initialized = false;
    
    _data.mx = _data.my = _data.mz = 0;
    _data.timestamp = 0;

    for (int i = 0; i < 3; i++) {
        this->_hard_iron[i] = 0.0f;
        for (int j = 0; j < 3; j++) {
            this->_soft_iron[i][j] = (i == j) ? 1.0f : 0.0f; // 단위 행렬 / Identity matrix
        }
    }
}

bool Magnet_Driver::begin() {
    // [1] I2C 하드웨어 파라미터 설정
    // [1] Configure I2C hardware parameters
    i2c_config_t conf = {};
    conf.mode = I2C_MODE_MASTER;             // 마스터 모드 설정 / Set to master mode
    conf.sda_io_num = (gpio_num_t)_sda_pin;               // SDA 핀 설정 / Set SDA pin
    conf.scl_io_num = (gpio_num_t)_scl_pin;               // SCL 핀 설정 / Set SCL pin
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE; // 내부 풀업 저항 활성화 / Enable internal pull-up
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = 400000;          // 통신 속도 400kHz (Fast Mode) / Bus speed 400kHz
    
    // 설정값 적용
    // Apply configuration
    _last_err = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (_last_err != ESP_OK) return false;

    // [2] I2C 드라이버 설치 (이미 설치된 경우 무시)
    // [2] Install I2C driver (Ignore if already installed)
    _last_err = i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
    if (_last_err != ESP_OK && _last_err != ESP_ERR_INVALID_STATE) { 
        return false; // 설치 실패 시 종료 / Return false if installation fails
    }

    // [3] 센서 연결 확인 (더미 읽기 시도)
    // [3] Check sensor connection (Attempt dummy read)
    uint8_t start_reg = QMC_REG_CONTROL_1; // QMC는 0x09번지를 읽어봅니다. / QMC probes 0x09 address.
    uint8_t temp_val;
    // 100ms 타임아웃을 두고 읽기 시도 / Attempt read with 100ms timeout
    this->_last_err = i2c_master_write_read_device(I2C_MASTER_NUM, this->_addr, &start_reg, 1, &temp_val, 1, 100 / portTICK_PERIOD_MS);
    
    if (this->_last_err != ESP_OK) return false; // 센서 응답 없음 / Sensor not responding

    // 3-1. 소프트 리셋 (필수는 아니지만 권장됨)
    // 3-1. Soft reset (Recommended but not mandatory)
    _write_reg(QMC_REG_CONTROL_2, 0x80); 
    vTaskDelay(10 / portTICK_PERIOD_MS); // 리셋 대기 / Wait for reset
    
    // 3-2. SET/RESET 주기 설정 (데이터시트 권장값 0x01)
    // 3-2. Set SET/RESET period (Datasheet recommended value 0x01)
    _write_reg(QMC_REG_FBR, 0x01);
    
    // 3-3. 모드 설정: 연속 측정(Continuous), 200Hz 출력, 8G 범위, 512 오버샘플링 = 0x1D
    // 3-3. Mode set: Continuous, 200Hz ODR, 8G Range, 512 Over-sampling = 0x1D
    _write_reg(QMC_REG_CONTROL_1, 0x1D);

    if (this->_last_err == ESP_OK) {
        _is_initialized = true;
    }
    return _is_initialized;
}

void Magnet_Driver::set_calibration(float hard_iron[3], float soft_iron[3][3]) {
    // 외부에서 계산된 보정값을 내부 멤버 변수에 저장
    // Store externally calculated calibration values into internal member variables
    for (int i = 0; i < 3; i++) {
        this->_hard_iron[i] = hard_iron[i];
        for (int j = 0; j < 3; j++) {
            this->_soft_iron[i][j] = soft_iron[i][j];
        }
    }
}

void Magnet_Driver::update() {
    // 초기화되지 않았다면 실행 중단
    // Abort execution if not initialized
    if (!_is_initialized) return;

    uint8_t start_reg = 0x00;
    uint8_t buffer[6];

    // 데이터 읽기 직전에 타임스탬프를 찍습니다.
    // Take a timestamp right before reading data.
    this->_data.timestamp = (uint32_t)esp_timer_get_time();

    // [1] I2C로 6바이트(X, Z, Y) 데이터 읽기
    // [1] Read 6 bytes (X, Z, Y) via I2C
    this->_last_err = i2c_master_write_read_device(I2C_MASTER_NUM, this->_addr, &start_reg, 1, buffer, 6, 100 / portTICK_PERIOD_MS);

    // 에러 발생 시(연결 끊김 등) 0 벡터 반환하여 튀는 값 방지
    // QMC sends LSB first, then MSB. (buffer[1] is high byte)
    if (this->_last_err != ESP_OK) return;

    // [2] QMC는 하위 바이트(LSB)가 먼저 오고 상위 바이트(MSB)가 나중에 옵니다. (buffer[1]이 상위)
    // [2] Parse HMC5883L data order: X -> Z -> Y (Big Endian)
    // 순서: X(0,1) -> Y(2,3) -> Z(4,5)
    // Order: X(0,1) -> Y(2,3) -> Z(4,5)
    int16_t rx = (int16_t)(buffer[1] << 8 | buffer[0]); 
    int16_t ry = (int16_t)(buffer[3] << 8 | buffer[2]); 
    int16_t rz = (int16_t)(buffer[5] << 8 | buffer[4]);

    // [3] 하드 아이언 보정 (오프셋 제거)
    // [3] Hard iron calibration (Remove offset)
    float tx = (float)rx - this->_hard_iron[0];
    float ty = (float)ry - this->_hard_iron[1];
    float tz = (float)rz - this->_hard_iron[2];

    // [4] 소프트 아이언 보정 (행렬 곱을 통한 스케일/회전 왜곡 보정)
    // [4] Soft iron calibration (Compensate scale/rotation distortion via matrix multiplication)
    this->_data.mx = this->_soft_iron[0][0] * tx + this->_soft_iron[0][1] * ty + this->_soft_iron[0][2] * tz;
    this->_data.my = this->_soft_iron[1][0] * tx + this->_soft_iron[1][1] * ty + this->_soft_iron[1][2] * tz;
    this->_data.mz = this->_soft_iron[2][0] * tx + this->_soft_iron[2][1] * ty + this->_soft_iron[2][2] * tz;
}

esp_err_t Magnet_Driver::_write_reg(uint8_t reg, uint8_t val) {
    uint8_t data[2] = {reg, val};
    // ESP-IDF 방식의 쓰기 명령 전송
    // Send write command using ESP-IDF style
    this->_last_err = i2c_master_write_to_device(I2C_MASTER_NUM, this->_addr, data, 2, 100 / portTICK_PERIOD_MS);
    return this->_last_err;
}