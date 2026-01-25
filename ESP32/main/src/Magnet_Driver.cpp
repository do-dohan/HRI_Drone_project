#include "Magnet_Driver.hpp"
#include "driver/i2c.h" // ESP-IDF I2C 드라이버 필수 포함 / Must include ESP-IDF I2C driver

// I2C 마스터 포트 번호 정의 (ESP32는 포트 0, 1 두 개 있음)
// Define I2C master port number (ESP32 has ports 0 and 1)
#define I2C_MASTER_NUM I2C_NUM_0

// HMC5883L 레지스터 주소 정의
// HMC5883L Register address definitions
#define HMC_REG_CONFIG_A    0x00
#define HMC_REG_CONFIG_B    0x01
#define HMC_REG_MODE        0x02
#define HMC_REG_DATA_START  0x03

Magnet_Driver::Magnet_Driver(uint8_t addr) 
    : _addr(addr), _last_err(ESP_OK) {
    // 캘리브레이션 변수들을 기본값(보정 없음)으로 초기화
    // Initialize calibration variables to default values (no correction)
    for (int i = 0; i < 3; i++) {
        this->_hard_iron[i] = 0.0f;
        for (int j = 0; j < 3; j++) {
            this->_soft_iron[i][j] = (i == j) ? 1.0f : 0.0f; // 단위 행렬 / Identity matrix
        }
    }
}

bool Magnet_Driver::begin(int sda_pin, int scl_pin) {
    // [1] I2C 하드웨어 파라미터 설정
    // [1] Configure I2C hardware parameters
    i2c_config_t conf = {};
    conf.mode = I2C_MODE_MASTER;             // 마스터 모드 설정 / Set to master mode
    conf.sda_io_num = sda_pin;               // SDA 핀 설정 / Set SDA pin
    conf.scl_io_num = scl_pin;               // SCL 핀 설정 / Set SCL pin
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE; // 내부 풀업 저항 활성화 / Enable internal pull-up
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = 400000;          // 통신 속도 400kHz (Fast Mode) / Bus speed 400kHz
    
    // 설정값 적용
    // Apply configuration
    esp_err_t err = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (err != ESP_OK) return false;

    // [2] I2C 드라이버 설치 (이미 설치된 경우 무시)
    // [2] Install I2C driver (Ignore if already installed)
    err = i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) { 
        return false; // 설치 실패 시 종료 / Return false if installation fails
    }

    // [3] 센서 연결 확인 (더미 읽기 시도)
    // [3] Check sensor connection (Attempt dummy read)
    uint8_t start_reg = HMC_REG_CONFIG_A;
    uint8_t temp_val;
    // 100ms 타임아웃을 두고 읽기 시도 / Attempt read with 100ms timeout
    this->_last_err = i2c_master_write_read_device(I2C_MASTER_NUM, this->_addr, &start_reg, 1, &temp_val, 1, 100 / portTICK_PERIOD_MS);
    
    if (this->_last_err != ESP_OK) return false; // 센서 응답 없음 / Sensor not responding

    // [4] 센서 설정값 기록: 8-평균, 15Hz 주사율, 연속 측정 모드
    // [4] Write sensor config: 8-average, 15Hz rate, continuous measurement mode
    _write_reg(HMC_REG_CONFIG_A, 0x70);
    _write_reg(HMC_REG_CONFIG_B, 0x20);
    _write_reg(HMC_REG_MODE, 0x00);
    
    return true; // 초기화 성공 / Initialization successful
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

Magnet_Driver::Magnet_Vector Magnet_Driver::read() {
    Magnet_Vector output = {0.0f, 0.0f, 0.0f};
    uint8_t start_reg = HMC_REG_DATA_START;
    uint8_t buffer[6];

    // [1] I2C로 6바이트(X, Z, Y) 데이터 읽기
    // [1] Read 6 bytes (X, Z, Y) via I2C
    this->_last_err = i2c_master_write_read_device(I2C_MASTER_NUM, this->_addr, &start_reg, 1, buffer, 6, 100 / portTICK_PERIOD_MS);

    // 에러 발생 시(연결 끊김 등) 0 벡터 반환하여 튀는 값 방지
    // If error occurs (e.g., disconnection), return zero vector to prevent spikes
    if (this->_last_err != ESP_OK) return output;

    // [2] HMC5883L 데이터 순서 파싱: X(0,1) -> Z(2,3) -> Y(4,5) (빅 엔디안)
    // [2] Parse HMC5883L data order: X -> Z -> Y (Big Endian)
    int16_t rx = (int16_t)(buffer[0] << 8 | buffer[1]);
    int16_t rz = (int16_t)(buffer[2] << 8 | buffer[3]);
    int16_t ry = (int16_t)(buffer[4] << 8 | buffer[5]);

    // [3] 하드 아이언 보정 (오프셋 제거)
    // [3] Hard iron calibration (Remove offset)
    float tx = (float)rx - this->_hard_iron[0];
    float ty = (float)ry - this->_hard_iron[1];
    float tz = (float)rz - this->_hard_iron[2];

    // [4] 소프트 아이언 보정 (행렬 곱을 통한 스케일/회전 왜곡 보정)
    // [4] Soft iron calibration (Compensate scale/rotation distortion via matrix multiplication)
    output.x = this->_soft_iron[0][0] * tx + this->_soft_iron[0][1] * ty + this->_soft_iron[0][2] * tz;
    output.y = this->_soft_iron[1][0] * tx + this->_soft_iron[1][1] * ty + this->_soft_iron[1][2] * tz;
    output.z = this->_soft_iron[2][0] * tx + this->_soft_iron[2][1] * ty + this->_soft_iron[2][2] * tz;

    return output;
}

esp_err_t Magnet_Driver::_write_reg(uint8_t reg, uint8_t val) {
    uint8_t data[2] = {reg, val};
    // ESP-IDF 방식의 쓰기 명령 전송
    // Send write command using ESP-IDF style
    this->_last_err = i2c_master_write_to_device(I2C_MASTER_NUM, this->_addr, data, 2, 100 / portTICK_PERIOD_MS);
    return this->_last_err;
}