cmake_minimum_required(VERSION 3.10)
project(hri_drone_project)

# 1. 컴파일 옵션 설정
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# 2. 필요한 패키지 불러오기 (여기에 sensor_msgs가 반드시 있어야 합니다!)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED) # <--- 팩트: 이게 빠져서 에러가 났던 것입니다.

# 공통 헤더 경로 설정 (중복 방지를 위해 상단에 한 번만 작성)
include_directories(include)

# [A] Signal_Bridge_Node
add_executable(Signal_Bridge_Node
    src/Signal_Bridge_main.cpp 
    src/Signal_Bridge_Node.cpp
)
ament_target_dependencies(Signal_Bridge_Node rclcpp std_msgs)

# [B] Signal_Processor_Node
add_executable(Signal_Processor_Node
    src/Signal_Processor_main.cpp
    src/Signal_Processor_Node.cpp
    src/Madgwick_Filter.cpp
)
# 여기도 sensor_msgs를 쓸 수 있으니 추가해두는 것이 안전합니다.
ament_target_dependencies(Signal_Processor_Node rclcpp std_msgs sensor_msgs)

# [C] 테스트용 노드 (Test_Sensor_Node)
add_executable(Test_Sensor_Node
    test/Test_Sensor_Node.cpp
)
ament_target_dependencies(Test_Sensor_Node rclcpp std_msgs sensor_msgs)

# [D] 신호 처리 테스트 노드 (Signal_Processor_Test)
add_executable(Signal_Processor_Test src/Signal_Processor_Test.cpp)
ament_target_dependencies(Signal_Processor_Test rclcpp sensor_msgs)

# 3. 설치 설정 (한꺼번에 모아서 관리)
install(TARGETS
  Signal_Bridge_Node
  Signal_Processor_Node
  Test_Sensor_Node
  Signal_Processor_Test
  DESTINATION lib/${PROJECT_NAME}
)

# 4. 패키지 구성 마무리 (무조건 맨 마지막에 딱 한 번만!)
ament_package()