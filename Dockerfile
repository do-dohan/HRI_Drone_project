# (바꾸면 안 됨) ROS2 Humble 데스크탑 이미지를 기본 베이스로 사용합니다.
# (Fixed) Use ROS2 Humble Desktop as the base image.
FROM osrf/ros:humble-desktop

# (바꾸면 안 되는 내용) 시스템 패키지를 최신화하고 빌드에 필요한 모든 의존성을 설치합니다.
# (Fixed) Update system packages and install all necessary build dependencies.
RUN apt-get update && apt-get install -y \
    # 빌드 시스템 및 파이썬 환경 관리 도구들입니다.
    # Build systems and Python environment management tools.
    cmake ninja-build python3-pip python3-venv \
    # USB 장치 제어 및 소스 코드 관리를 위한 라이브러리입니다.
    # Libraries for USB device control and source code management.
    libusb-1.0-0-dev git wget \
    # ESP-IDF 툴체인 빌드 시 필요한 코드 생성 도구들입니다.
    # Code generation tools required for the ESP-IDF toolchain.
    flex bison gperf \
    # ESP32 펌웨어 빌드 시 사용되는 각종 파이썬 모듈들입니다.
    # Various Python modules used during ESP32 firmware builds.
    python3-setuptools python3-serial python3-click \
    # 보안 및 시스템 하위 계층 인터페이스를 위한 파일들입니다.
    # Files for security and low-level system interfaces.
    libffi-dev libssl-dev \
    # USB를 통해 펌웨어를 업로드(Flash)할 때 사용하는 핵심 도구입니다.
    # Core tool used for flashing firmware via USB.
    dfu-util libusb-1.0-0 \
    # 설치가 끝난 후 불필요한 임시 파일을 삭제하여 이미지 용량을 줄입니다.
    # Clean up temporary files to reduce the final image size.
    && rm -rf /var/lib/apt/lists/*

# (유동적 변경 가능) 픽시를 설치하고 실행 경로를 시스템 환경 변수에 등록합니다.
# (Modifiable) Install Pixi and add its binary path to ENV.
RUN curl -fsSL https://pixi.sh/install.sh | bash
ENV PATH="/root/.pixi/bin:$PATH"

# (바꾸면 안 됨) 빌드 중에 'source' 명령어를 쓰기 위해 기본 셸을 bash로 바꿉니다.
# (Fixed) Set default shell to bash for 'source' command compatibility.
SHELL ["/bin/bash", "-c"]

# (유동적 변경 가능) ESP-IDF를 설치할 폴더로 이동하여 소스 코드를 내려받습니다.
# (Modifiable) Move to the ESP-IDF directory and clone the repository.
WORKDIR /opt/esp
RUN git clone --recursive -b v5.1 https://github.com/espressif/esp-idf.git . && \
    # (바꾸면 안 됨) ESP32 칩 전용 빌드 도구들을 설치합니다.
    # (Fixed) Install the toolchain specifically for the esp32 chip.
    ./install.sh esp32

# (유동적 변경 가능) 마이크로 ROS 에이전트 빌드를 위한 작업 공간을 만듭니다.
# (Modifiable) Create a workspace for the Micro-ROS Agent.
WORKDIR /micro_ros_ws

# (바꾸면 안 됨) 에이전트 소스를 받고 ROS2 환경을 불러온 뒤(source) 빌드합니다.
# (Fixed) Clone agent source, source ROS2, and then run colcon build.
RUN git clone -b humble https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup && \
    apt-get update && rosdep update && \
    rosdep install --from-paths src --ignore-src -y && \
    source /opt/ros/humble/setup.bash && colcon build

# (바꾸면 안 됨) 마이크로 ROS 에이전트의 실제 실행 파일들을 생성하고 빌드합니다.
# (Fixed) Create and build the actual Micro-ROS Agent firmware tools.
RUN source /opt/ros/humble/setup.bash && source install/setup.bash && \
    ros2 run micro_ros_setup create_agent_ws.sh && \
    ros2 run micro_ros_setup build_agent.sh

# (유동적 변경 가능) 우리 드론 프로젝트의 실제 코드가 들어갈 폴더를 설정합니다.
# (Modifiable) Set the main application folder for our drone project.
WORKDIR /app
COPY . .

# (바꾸면 안 됨) 터미널을 켤 때마다 ROS2와 ESP-IDF 환경을 자동으로 불러오게 만듭니다.
# (Fixed) Auto-source ROS2 and ESP-IDF environments on shell startup.
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "source /micro_ros_ws/install/setup.bash" >> ~/.bashrc && \
    echo "source /opt/esp/export.sh" >> ~/.bashrc

CMD ["bash"]