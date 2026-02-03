import math # 수학 계산을 위한 라이브러리 / Library for mathematical calculations

def mapping_flex_sensor(current_angle):
    # 1. 설정값 정의
    # Define configuration values
    min_angle = 0.0      # 변수: 손가락을 폈을 때 각도 (라디안) / Min angle (0 degrees)
    max_angle = 1.57     # 변수: 손가락을 쥐었을 때 각도 (90도 = pi/2) / Max angle (90 degrees)
    
    # 2. 클램핑 (범위 제한)
    # Clamping (Restrict the value within the defined range)
    # 각도가 범위를 벗어나지 않도록 강제합니다.
    current_angle = max(min_angle, min(current_angle, max_angle))
    
    # 3. 선형 매핑 공식 적용 (Linear Mapping Formula)
    # (입력값 - 최소값) / (최대값 - 최소값) * (목표최대 - 목표최소) + 목표최소
    # (Input - Min) / (Max - Min) * (TargetMax - TargetMin) + TargetMin
    flex_scale = ((current_angle - min_angle) / (max_angle - min_angle)) * (10 - 1) + 1
    
    # 4. 정수형 변환 및 반올림
    # Convert to integer and round the value
    return round(flex_scale) # 최종 1~10 사이의 정수 반환 / Return integer between 1-10

# 예시 데이터 테스트
# Example data test
print(f"완전히 폈을 때 (0 rad): {mapping_flex_sensor(0.0)}")   # 출력: 1
print(f"절반 굽혔을 때 (0.78 rad): {mapping_flex_sensor(0.78)}") # 출력: 5 또는 6
print(f"완전히 쥐었을 때 (1.57 rad): {mapping_flex_sensor(1.57)}") # 출력: 10