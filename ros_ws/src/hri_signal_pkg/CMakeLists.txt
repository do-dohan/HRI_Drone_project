cmake_minimum_required(VERSION 3.8)
project(hri_signal_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# C++ 표준 고정 (너 코드가 C++17 문법 쓰는 경우가 많아서 17 권장)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# =============================================================================
# Dependencies
# =============================================================================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_default_runtime REQUIRED)

# =============================================================================
# 메시지 생성 (msg)
# =============================================================================
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/HRIAttitude.msg"
  "msg/HRIFlex.msg"
  DEPENDENCIES std_msgs
)

# =============================================================================
# 실행 노드 빌드 (C++ node executable)
# =============================================================================
add_executable(signal_processor_node
  src/Signal_Processor_main.cpp
  src/Signal_Processor_Node.cpp

  src/Mag_Quality.cpp
  src/EMA_Filter.cpp
  src/Madgwick_Filter.cpp
  src/Mapping_Utils.cpp
  src/Quaternion_to_Euler.cpp
  src/Smooth_Step_Utils.cpp
)

# include/ 아래를 include path로 등록
target_include_directories(signal_processor_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(signal_processor_node
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
# 🇰🇷 한글: 우리가 만든 HRI 메시지들의 C++ 변환 규격을 찾아 'cpp_typesupport_target'이라는 임시 이름표를 붙입니다.
# 🇬🇧 영어: Gets the C++ typesupport target for the custom messages and assigns it to a variable.
# 🛠️ 규칙: cpp_typesupport_target은 내가 짓는 [유동 변수], 나머지는 [고정 문법]입니다.

target_link_libraries(signal_processor_node "${cpp_typesupport_target}")
# 🇰🇷 한글: 방금 찾은 메시지 규격을 실제 내 신호 처리 로봇 뇌(signal_processor_node)와 단단히 연결(링크)합니다.
# 🇬🇧 영어: Links the generated typesupport target to our executable node.
# 🛠️ 규칙: signal_processor_node와 변수명은 [유동 변수], target_link_libraries는 [고정 문법]입니다.

# =============================================================================
# Install
# =============================================================================
install(TARGETS signal_processor_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

# msg 런타임 의존성 export (다른 패키지에서 이 msg를 쓸 때 필요)
ament_export_dependencies(rosidl_default_runtime)
ament_export_include_directories(include)

# =============================================================================
# Testing
# =============================================================================
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()