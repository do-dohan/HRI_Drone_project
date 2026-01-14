cmake_minimum_required(VERSION 3.10)
project(hri_drone_project) # 프로젝트 이름 (본인의 실제 프로젝트 이름으로 수정하세요)

# 1. 컴파일 옵션 설정 (C++ 17 표준 사용)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# 2. 필요한 패키지 불러오기 (ROS2 핵심 라이브러리들)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# 1. Signal_Bridge_Node노드 빌드
add_executable(Signal_Bridge_Node
    src/Signal_Bridge_main.cpp 
    src/Signal_Bridge_Node.cpp
)
ament_target_dependencies(Signal_Bridge_Node rclcpp std_msgs)

target_include_directories(Signal_Bridge_Node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

install(TARGETS
  Signal_Bridge_Node
  DESTINATION lib/${PROJECT_NAME})

ament_package()