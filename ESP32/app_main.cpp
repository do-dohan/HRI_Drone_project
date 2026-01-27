#include <stdio.h>
#include <unistd.h> //sleep, usleep
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

// 1. 드라이버 및 노드 헤더 포함
#include "IMU_Driver.hpp"
#include "Magnet_Driver.hpp"
#include "Flex_Driver.hpp"
#include "Signal_Bridge_Node.hpp"

// 2. micro-ROS 필수 헤더
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclcpp/rclc.h>
#include <rmw_microros/rmw_microros.h>

//로그 태그
static const char *TAG = "MAIN_APP";

//에러 확인용 매크로 (실패시 재부팅)
#define RCCHECK(fn) {rcl_ret_t temp_rc = fn;if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc); vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

extern "C" {
    void app_main(void);
}

void app_main(void) {
    // [1] 통신 설정: UART (USB Serial)
    // ESP-IDF menuconfig에서 설정된 Custom Transport(UART)를 사용합니다.
    // WiFi를 쓰려면 여기서 rmw_microros_custom_transport 등의 설정이 필요하지만,
    // Serial은 기본 설정으로 바로 작동합니다.

    // [2] micro-ROS 할당자 및 초기화 옵션 설정

    //여기서부터 공부 필요!
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;
    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    RCCHECK(rcl_init_options_init(&init_options, allocator));

    // [3] 에이전트(PC) 연결 대기 루프 (Ping)
    // PC에서 'micro-ros-agent'를 켜지 않으면 여기서 대기합니다.
    ESP_LOGI(TAG, "Connecting to micro-ROS Agent...");
    while (rmw_uros_ping_agent(1000, 1) != RMW_RET_OK) {
        vTaskDelay(pdMS_TO_TICKS(1000)); // 1초마다 재시도
        ESP_LOGW(TAG, "Waiting for Agent...");
    }
    ESP_LOGI(TAG, "Connected to Agent!");

    // [4] rclc_support 초기화
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

    // =============================================================
    // [5] 하드웨어 객체 생성 (여기가 핵심!)
    // =============================================================
    
    // I2C 핀 설정 (SDA=21, SCL=22) - ESP32 표준
    uint8_t sda_pin = 21;
    uint8_t scl_pin = 22;

    // 5-1. 손목 IMU (주소 0x68, AD0=GND)
    IMU_Driver imu_wrist(sda_pin, scl_pin, 0x68);

    // 5-2. 팔 IMU (주소 0x69, AD0=VCC)
    // 주의: 두 IMU가 같은 I2C 버스를 공유하지만 주소가 달라 충돌 안 함.
    IMU_Driver imu_arm(sda_pin, scl_pin, 0x69);

    // 5-3. 지자기 센서 (주소 0x0D) - 같은 I2C 버스 공유
    Magnet_Driver magnet(sda_pin, scl_pin, 0x0D);

    // 5-4. 굴곡 센서 (ADC1_CHANNEL_6 -> GPIO 34)
    // 생성자에서 채널을 받게 수정하셨다면 변경, 핀 번호라면 매핑 확인 필수
    Flex_Driver flex(34); 

    // [6] 시그널 브릿지 노드 생성 및 초기화
    Signal_Bridge_Node bridge;
    
    // 여기서 각 드라이버의 begin()이 내부적으로 호출됩니다.
    // I2C Driver Install은 첫 번째 호출(imu_wrist)에서 수행되고,
    // 나머지는 "이미 설치됨"을 감지하고 스킵하므로 안전합니다.
    bridge.init(&support, imu_wrist, imu_arm, magnet, flex);

    ESP_LOGI(TAG, "System Initialized. Starting Main Loop...");

    // [7] 메인 루프 (제어 주기 20ms = 50Hz)
    // HRI 제스처 인식에는 50Hz~100Hz가 적당합니다.
    const int loop_period_ms = 20;
    TickType_t last_wake_time = xTaskGetTickCount();

    while (1) {
        // 센서 업데이트 및 퍼블리시
        bridge.update_Wrist_IMU(imu_wrist);
        bridge.update_Arm_IMU(imu_arm);
        bridge.update_Magnet(magnet);
        bridge.update_Flex(flex);

        // 주기적 실행 보장 (vTaskDelayUntil과 유사 효과)
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(loop_period_ms));
    }

    // 만약 루프를 탈출하면 리소스 해제 (실제로는 도달 안 함)
    RCCHECK(rcl_node_fini(&bridge._node)); // bridge 객체 내 node 접근 필요 (friend 혹은 getter 필요하나 생략)
}
