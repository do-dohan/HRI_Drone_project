#include "Magnet_Driver.hpp"
#include "driver/i2c.h"

#define I2C_MASTER_NUM I2C_NUM_0

// ==========================================
// [AK8963 Register Map] (MPU9250 Internal Mag)
// ==========================================
#define AK8963_WHO_AM_I     0x00 // ID Register (Should be 0x48)
#define AK8963_INFO         0x01
#define AK8963_ST1          0x02 // Status 1 (Data Ready)
#define AK8963_XOUT_L       0x03 // Data Start
#define AK8963_ST2          0x09 // Status 2 (Overflow / Read End)
#define AK8963_CNTL1        0x0A // Control 1 (Mode Setting)
#define AK8963_CNTL2        0x0B // Control 2 (Soft Reset)
#define AK8963_ASAX         0x10 // Fuse ROM X-axis sensitivity adjustment

// [Modes]
#define AK8963_MODE_POWERDOWN 0x00
#define AK8963_MODE_FUSEROM   0x0F
#define AK8963_MODE_CONTINUOUS_100HZ 0x16 // 16-bit output, 100Hz

bool Magnet_Driver::_is_i2c_installed = false;

Magnet_Driver::Magnet_Driver(uint8_t sda, uint8_t scl, uint8_t addr) 
    : _sda_pin(sda), _scl_pin(scl),_addr(addr), _last_err(ESP_OK), _is_initialized(false) {
    _data.mx = _data.my = _data.mz = 0;
    _data.timestamp = 0;
    
    // 캘리브레이션 변수들을 기본값(보정 없음)으로 초기화
    // Initialize calibration variables to default values (no correction)
    for (int i = 0; i < 3; i++) {
        this->_hard_iron[i] = 0.0f;
        for (int j = 0; j < 3; j++) {
            this->_soft_iron[i][j] = (i == j) ? 1.0f : 0.0f; // 단위 행렬 / Identity matrix
        }
    }

    // ASA 보정값 기본 초기화 (1.0 = 보정 없음)
    _asa_adj[0] = _asa_adj[1] = _asa_adj[2] = 1.0f;
}

bool Magnet_Driver::begin() {
    // [1] I2C 하드웨어 파라미터 설정
    // [1] Configure I2C hardware parameters
    if (!_is_i2c_installed) {
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

        _last_err = i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
        if (_last_err == ESP_OK) {
            _is_i2c_installed = true;
        } else {
            return false;
        }
    }

    // 2. ID 확인 (Who Am I)
    uint8_t who_am_i = _read_reg(AK8963_WHO_AM_I);
    if (who_am_i != 0x48) {
        return false; // AK8963 인식 실패 (Bypass 안 켜졌거나 배선 문제)
    }

    // 3. ASA(Sensitivity Adjustment) 값 읽기 시퀀스
    // AK8963은 Fuse ROM 모드로 들어가야만 이 값을 읽을 수 있습니다.
    
    // 3-1. Power Down Mode 설정
    _write_reg(AK8963_CNTL1, AK8963_MODE_POWERDOWN);
    vTaskDelay(10 / portTICK_PERIOD_MS);

    // 3-2. Fuse ROM Access Mode 설정
    _write_reg(AK8963_CNTL1, AK8963_MODE_FUSEROM);
    vTaskDelay(10 / portTICK_PERIOD_MS);

    // 3-3. ASA 값 3바이트 읽기 (ASAX, ASAY, ASAZ)
    uint8_t asa_buff[3];
    uint8_t start_reg = AK8963_ASAX;
    i2c_master_write_read_device(I2C_MASTER_NUM, _addr, &start_reg, 1, asa_buff, 3, 100 / portTICK_PERIOD_MS);

    // 3-4. ASA 공식 적용: Adj = ((Val - 128) * 0.5 / 128) + 1
    for(int i=0; i<3; i++) {
        _asa_adj[i] = (float)(asa_buff[i] - 128) / 256.0f + 1.0f;
    }

    // 3-5. 다시 Power Down (모드 변경 전 필수)
    _write_reg(AK8963_CNTL1, AK8963_MODE_POWERDOWN);
    vTaskDelay(10 / portTICK_PERIOD_MS);

    // 4. 최종 동작 모드 설정: 16-bit 출력, 100Hz 연속 측정
    _write_reg(AK8963_CNTL1, AK8963_MODE_CONTINUOUS_100HZ);
    vTaskDelay(10 / portTICK_PERIOD_MS);

    _is_initialized = true;
    return true;
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

    // 데이터 읽기 직전에 타임스탬프를 찍습니다.
    // Take a timestamp right before reading data.
    this->_data.timestamp = (uint32_t)esp_timer_get_time();

    // 1. ST1 (Status 1) 확인 - 데이터 준비되었는지?
    uint8_t st1 = _read_reg(AK8963_ST1);
    if (!(st1 & 0x01)) { 
        return; // 데이터 준비 안 됨
    }

    // 2. 데이터 읽기 (HXL ~ ST2 까지 총 7바이트를 한 번에 읽어야 함)
    // [중요] ST2(0x09) 레지스터까지 읽어야 센서가 "읽기 완료"로 인식하고 다음 측정을 시작함
    uint8_t buffer[7];
    uint8_t start_reg = AK8963_XOUT_L;

    // [1] I2C로 6바이트(X, Z, Y) 데이터 읽기
    // [1] Read 6 bytes (X, Z, Y) via I2C
    this->_last_err = i2c_master_write_read_device(I2C_MASTER_NUM, _addr, &start_reg, 1, buffer, 7, 100 / portTICK_PERIOD_MS);
    if (this->_last_err != ESP_OK) return;

    // 3. Magnetic Sensor Overflow 확인 (ST2의 3번째 비트 HOFL)
    uint8_t st2 = buffer[6];
    if (st2 & 0x08) {
        return; // 자기장 포화 상태(자석이 너무 가까움), 데이터 버림
    }

    // 4. 데이터 파싱 (Little Endian)
    int16_t raw_x = (int16_t)((buffer[1] << 8) | buffer[0]);
    int16_t raw_y = (int16_t)((buffer[3] << 8) | buffer[2]);
    int16_t raw_z = (int16_t)((buffer[5] << 8) | buffer[4]);

    // 5. 1차 보정: ASA (Factory Sensitivity) 적용
    // 데이터시트 단위 변환 (보통 uT 단위로 맞추려면 스케일링 필요, 여기선 Raw값 기준 보정만 수행)
    // AK8963 16-bit: 0.15 uT/LSB -> 4912 uT max
    float adj_x = (float)raw_x * _asa_adj[0];
    float adj_y = (float)raw_y * _asa_adj[1];
    float adj_z = (float)raw_z * _asa_adj[2];

    // 6. 2차 보정: Hard Iron (Offset)
    adj_x -= _hard_iron[0];
    adj_y -= _hard_iron[1];
    adj_z -= _hard_iron[2];

    // 7. 3차 보정: Soft Iron (Matrix Transform)
    float temp_mx = _soft_iron[0][0] * adj_x + _soft_iron[0][1] * adj_y + _soft_iron[0][2] * adj_z;
    float temp_my = _soft_iron[1][0] * adj_x + _soft_iron[1][1] * adj_y + _soft_iron[1][2] * adj_z;
    float temp_mz = _soft_iron[2][0] * adj_x + _soft_iron[2][1] * adj_y + _soft_iron[2][2] * adj_z;
  
    // ============================================================
    // [중요] 축 변경 (MPU9250 내부 물리적 배치에 따른 보정)
    // Sensor Frame -> Body Frame (Matched with Accel/Gyro)
    // ============================================================
    this->_data.mx = temp_my;   // Mag Y -> Body X
    this->_data.my = temp_mx;   // Mag X -> Body Y
    this->_data.mz = -temp_mz;  // Mag -Z -> Body Z
}

esp_err_t Magnet_Driver::_write_reg(uint8_t reg, uint8_t val) {
    uint8_t data[2] = {reg, val};
    this->_last_err = i2c_master_write_to_device(I2C_MASTER_NUM, _addr, data, 2, 100 / portTICK_PERIOD_MS);
    return this->_last_err;
}

uint8_t Magnet_Driver::_read_reg(uint8_t reg) {
    uint8_t val = 0;
    i2c_master_write_read_device(I2C_MASTER_NUM, _addr, &reg, 1, &val, 1, 100 / portTICK_PERIOD_MS);
    return val;
}