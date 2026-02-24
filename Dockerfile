# (바꾸면 안 됨) ROS2 Humble 데스크탑 이미지를 기본 베이스로 사용합니다.
# (Fixed) Use ROS2 Humble Desktop as the base image.
FROM osrf/ros:humble-desktop

# (바꾸면 안 되는 내용) 시스템 패키지를 최신화하고 빌드에 필요한 모든 의존성을 설치합니다.
# (Fixed) Update system packages and install all necessary build dependencies.
RUN apt-get update && apt-get install -y curl gnupg lsb-release && \
    # (핵심 추가) Gazebo 공식 저장소를 등록하기 위한 열쇠(GPG Key)를 내려받습니다.
    # (Core) Download the GPG key for the official Gazebo repository.
    curl https://packages.osrfoundation.org/gazebo.key | apt-key add - && \
    # (핵심 추가) Gazebo 패키지가 들어있는 저장소 주소를 리스트에 추가합니다.
    # (Core) Add the Gazebo repository URL to the source list.
    echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -sc) main" > /etc/apt/sources.list.d/gazebo-stable.list && \
    apt-get update && apt-get install -y \
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
    # 그래픽 렌더링을 위한 필수 유틸리티입니다. (이게 없으면 화면이 검게 나옵니다.)
    # Essential utilities for graphics rendering. (Fixes black screen issues.)
    mesa-utils libgl1-mesa-glx libgl1-mesa-dri \
    # (수정됨) ROS2와 가제보 하모닉 버전을 연결해주는 브릿지 패키지입니다.
    # (Modified) Bridge package connecting ROS2 with Gazebo Harmonic
    ros-humble-ros-gzharmonic \
    # (핵심 수정) 가든(Garden) 대신 최신 LTS 버전인 하모닉(Harmonic) 라이브러리들을 설치합니다.
    # (Modified) Install Harmonic (LTS) libraries instead of Garden.
    gz-harmonic \
    # (핵심) 물리 엔진과 시뮬레이션 월드를 담당하는 코어 라이브러리입니다. (하모닉은 버전 8을 씁니다.)
    # (Core) Core library for physics engine and simulation world. (Harmonic uses v8)
    libgz-sim8-dev \
    # (핵심) 프로세스 간 고속 데이터 통신(Pub/Sub)을 담당하는 전송 계층입니다. (버전 13)
    # (Core) Transport layer for high-speed inter-process communication. (v13)
    libgz-transport13-dev \
    # (핵심) 드론 제어 신호 등을 주고받을 때 쓰는 표준 메시지 형식 정의 파일입니다. (버전 10)
    # (Core) Standard message definitions used for control signals. (v10)
    libgz-msgs10-dev \
    # (필수) 파일 입출력, 오디오 처리 등 시뮬레이터의 기초 유틸리티 기능입니다. (버전 5)
    # (Required) Basic utility functions like file I/O and audio. (v5)
    libgz-common5-dev \
    # (필수) 로봇의 위치와 회전(Quaternion) 계산에 필수적인 수학 라이브러리입니다. (버전 7)
    # (Required) Math library essential for position and rotation calculations. (v7)
    libgz-math7-dev \
    # (필수) 작성한 URDF 모델을 별도의 런치 파일 없이 Rviz에서 바로 띄워 확인(display.launch.py)하기 위해 필요합니다.
    # (Required) Essential for visualizing URDF models in Rviz immediately without custom launch files. 
    ros-humble-urdf-tutorial \ 
    # (필수) 키보드 제어(Teleop) 노드가 실행될 때, 입력을 받기 위한 별도의 터미널 창을 띄우는 데 사용됩니다.
    # (Required) A terminal emulator used to spawn a separate window for capturing keyboard input during teleoperation.
    xterm \
    # 설치가 끝난 후 불필요한 임시 파일을 삭제하여 이미지 용량을 줄입니다.
    # Clean up temporary files to reduce the final image size.
    && rm -rf /var/lib/apt/lists/*

# (추가됨) 시스템 전체에 "우리는 하모닉을 쓴다"고 공표합니다. (빌드 에러 방지용)
# (Added) Set Global Environment Variable for Gazebo Harmonic.
ENV GZ_VERSION=harmonic

# (유동적 변경 가능) 픽시를 설치하고 실행 경로를 시스템 환경 변수에 등록합니다.
# (Modifiable) Install Pixi and add its binary path to ENV.
RUN curl -fsSL https://pixi.sh/install.sh | bash
ENV PATH="/root/.pixi/bin:$PATH"

# (추가됨) Git 소유권 에러 방지 (호스트-컨테이너 간 사용자 불일치 해결)
# (Added) Prevent Git ownership errors between host and container.
RUN git config --global --add safe.directory '*'

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

# (바꾸면 안 됨) 에이전트 소스를 받고, 빌드하고, 찌꺼기 파일까지 한 번에 정리합니다.
# (Fixed) Clone, build the agent, and clean up temp files in a single step.
RUN git clone -b humble https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup && \
    # 패키지 의존성 설치 (에러 방지를 위해 update 필수)
    apt-get update && rosdep update && \
    rosdep install --from-paths src --ignore-src -y && \
    # ROS2 환경 로드 후 1차 빌드
    source /opt/ros/humble/setup.bash && \
    colcon build && \
    # 빌드된 환경 로드 후 에이전트 생성 및 2차 빌드
    source install/setup.bash && \
    ros2 run micro_ros_setup create_agent_ws.sh && \
    ros2 run micro_ros_setup build_agent.sh && \
    # (중요) 빌드 후 남은 소스코드와 로그를 삭제하여 용량을 줄입니다.
    rm -rf log/ build/ src/

# (유동적 변경 가능) 우리 드론 프로젝트의 실제 코드가 들어갈 폴더를 설정합니다.
# (Modifiable) Set the main application folder for our drone project.
WORKDIR /app
COPY . .

# (바꾸면 안 됨) 터미널 설정을 진행합니다. ESP-IDF는 'get_esp' 명령어로 필요할 때만 불러옵니다.
# (Fixed) Setup shell environment. Load ESP-IDF only on demand via 'get_esp' alias.
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "source /micro_ros_ws/install/setup.bash" >> ~/.bashrc && \
    echo "export GZ_VERSION=harmonic" >> ~/.bashrc && \
    echo "alias get_esp='. /opt/esp/export.sh'" >> ~/.bashrc

# (추가됨) ESP-IDF 가상환경 내부에 PX4 빌드에 필요한 파이썬 라이브러리를 미리 설치합니다.
# (Added) Pre-install Python libraries required for PX4 build into the ESP-IDF virtual env.
RUN source /opt/esp/export.sh && \
    pip3 install empy==3.3.4 jsonschema jinja2 pyros-genmsg packaging

RUN apt-get update && apt-get install -y ros-humble-foxglove-bridge && \
    # (바꾸면 안 됨) 설치 후 불필요한 패키지 목록을 삭제하여 이미지 용량을 최적화합니다.
    # (Fixed) Clean up apt cache to minimize the Docker image size.
    rm -rf /var/lib/apt/lists/*

RUN pip3 install pygame 

RUN apt-get update && apt-get install -y \
    ros-humble-plotjuggler-ros \
    ros-humble-rosbag2-storage-mcap

RUN curl -fL -o /usr/bin/mcap https://github.com/foxglove/mcap/releases/download/releases%2Fmcap-cli%2Fv0.0.45/mcap-linux-amd64 && \
    chmod +x /usr/bin/mcap
    # [유틸리티] MCAP CLI 도구 설치 (변환용)

# (바꾸면 안 됨) 컨테이너가 시작될 때 기본적으로 실행할 명령어입니다. (터미널 대기)
# (Fixed) Default command to execute when the container starts. (Shell standby)
CMD ["bash"]