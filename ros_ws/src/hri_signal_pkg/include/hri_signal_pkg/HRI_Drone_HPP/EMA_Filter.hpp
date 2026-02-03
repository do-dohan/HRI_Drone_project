#ifndef EMA_FILTER_HPP
#define EMA_FILTER_HPP

#include <cstdint>

/**
 * @brief 지수 이동 평균(EMA) 필터 클래스
 * @details Exponential Moving Average (EMA) Filter Class
 */
class EMA_Filter {
private:
    float _alpha;          // 필터 가중치 (0.0 ~ 1.0 사이의 값)
    // Filter weight (coefficient) between 0.0 and 1.0
    
    float _y_output;       // 필터링된 현재 출력값 (이전 상태 저장)
    // Current filtered output value (stores the previous state)
    
    bool _is_initialized;  // 필터 초기 상태 설정 여부 확인 플래그
    // Flag to check if the initial filter state is set
    
public:
    /**
     * @brief 생성자: 초기 가중치를 설정함
     * @param a 초기 가중치 값 (alpha)
     */
    EMA_Filter(float a);
    
    /**
     * @brief 필터 초기화: 첫 번째 입력값으로 초기 상태를 설정함
     * @param init_y 초기 입력값
     */
    void initialize(float init_y);
    
    /**
     * @brief 필터링 수행: 새로운 입력값에 대해 EMA 연산을 수행함
     * @param input 새로운 센서 입력값
     * @return float 필터링된 결과값
     */
    float filter(float input);
    
    /**
     * @brief 가중치 동적 업데이트: 입력값의 오차에 따라 alpha 값을 조정함
     * @param input 현재 센서 입력값
     */
    void update_alpha(float input);
};

#endif