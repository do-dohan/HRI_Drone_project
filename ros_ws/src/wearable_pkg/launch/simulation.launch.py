import os                                             # 운영체제 기능을 사용하여 파일 경로 등을 다루기 위해 가져옵니다.
                                                      # Import OS module for path manipulation.
from ament_index_python.packages import get_package_share_directory # 설치된 ROS 패키지의 위치를 찾아주는 함수입니다.
                                                      # Helper to find package directories.
from launch import LaunchDescription                  # 런치 파일을 구성하는 최상위 컨테이너입니다.
                                                      # Main container for launch description.
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable # 다른 런치 파일을 실행하거나 환경 변수를 설정하는 액션입니다.
                                                      # Actions to include files and set env vars.
from launch.launch_description_sources import PythonLaunchDescriptionSource # 파이썬으로 작성된 런치 파일을 읽어오는 도구입니다.
                                                      # Source handler for Python launch files.
from launch_ros.actions import Node                   # ROS 2 노드(실행 파일)를 실행하는 액션입니다.
                                                      # Action to run a ROS 2 node.
import xacro                                          # URDF 로봇 모델 파일을 파싱(해석)하는 도구입니다.
                                                      # Library to parse URDF files.

def generate_launch_description():
    """
    ROS 2 런치 시스템이 호출하는 메인 함수입니다. 실행할 모든 노드와 설정을 정의하여 반환합니다.
    Main function called by ROS 2 launch system. Returns list of actions.
    """

    # -------------------------------------------------------------------------
    # 1. 기본 설정 (Configuration)
    # -------------------------------------------------------------------------
    pkg_name = 'wearable_pkg'                         # 패키지 이름입니다. 경로 탐색의 기준이 됩니다.
                                                      # Package name.
    urdf_file = 'wearable_hand.urdf'                  # 불러올 로봇 모델 파일(.urdf) 이름입니다.
                                                      # URDF file name.
    
    # 패키지가 설치된 절대 경로를 찾습니다.
    # Get absolute path of the installed package.
    pkg_path = get_package_share_directory(pkg_name)
    
    # URDF 파일의 전체 경로를 완성합니다.
    # Build full path to URDF file.
    urdf_path = os.path.join(pkg_path, 'urdf', urdf_file)

    # -------------------------------------------------------------------------
    # [핵심] 가제보 환경 변수 설정 (Environment Variable)
    # -------------------------------------------------------------------------
    # 가제보는 외부 프로그램이므로, 우리 패키지의 메쉬(.stl) 파일이 어디 있는지 모릅니다.
    # Gazebo needs to know where mesh files are located via env var.
    
    # install/share 폴더 경로를 구합니다.
    # Get install/share directory.
    install_dir = os.path.join(get_package_share_directory(pkg_name), '..')
    
    # 'GZ_SIM_RESOURCE_PATH' 환경 변수에 경로를 등록하여 가제보가 메쉬를 찾을 수 있게 합니다.
    # Set GZ_SIM_RESOURCE_PATH so Gazebo can find meshes.
    resource_env = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=install_dir
    )

    # -------------------------------------------------------------------------
    # 2. 로봇 모델 파싱 (Parse URDF)
    # -------------------------------------------------------------------------
    # URDF 파일을 열어서 내용을 읽습니다.
    # Open and parse URDF file.
    doc = xacro.parse(open(urdf_path))
    
    # Xacro 매크로를 처리하여 완전한 XML로 변환합니다.
    # Process xacro macros.
    xacro.process_doc(doc)
    
    # XML 문자열로 변환하여 파라미터로 준비합니다.
    # Convert to XML string parameter.
    params = {'robot_description': doc.toxml()}

    # -------------------------------------------------------------------------
    # 3. 노드 정의 (Nodes Definition)
    # -------------------------------------------------------------------------

    # [A] 로봇 상태 퍼블리셔 (Robot State Publisher)
    # 로봇의 뼈대 정보를 '/robot_description' 토픽으로 방송합니다.
    # Publishes robot structure to /robot_description.
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    # [B] 가제보 시뮬레이터 (Gazebo Sim)
    # ros_gz_sim 패키지의 기본 런치 파일을 불러와 가제보를 실행합니다.
    # Include standard Gazebo launch file.
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        # '-r': 즉시 실행, 'empty.sdf': 빈 맵 열기
        # Run immediately in empty world.
        launch_arguments={'gz_args': '-r empty.sdf'}.items(),
    )

    # [C] 로봇 소환 (Spawn Entity)
    # 가제보 월드 안에 로봇을 생성(Spawn)합니다.
    # Spawns robot into Gazebo.
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'wearable_hand',     # 로봇 이름
            '-topic', 'robot_description',# 모델 정보 토픽
            '-z', '1.0'                   # 높이 1m
        ],
        output='screen',
    )
    
    # [D] Rviz2 시각화 (Visualization)
    # Rviz를 자동으로 실행하여 로봇 움직임을 눈으로 확인합니다.
    # Automatically starts Rviz2.
    node_rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen'
    )

    # [E] 키보드 조종 노드 (Teleop Node) - [핵심 포인트!]
    # 키보드 입력을 받기 위해 'xterm'이라는 새 터미널 창을 띄워서 실행합니다.
    # Runs teleop node in a new 'xterm' window for keyboard input.
    node_teleop = Node(
        package=pkg_name,
        executable='teleop_node',
        name='teleop_node',
        output='screen',
        # 'prefix'는 명령어를 실행하기 전에 앞에 붙이는 접두어입니다.
        # "xterm -e"는 "새 xterm 창을 열고 그 안에서 실행하라"는 뜻입니다.
        # Opens a new terminal window.
        prefix='xterm -e' 
    )

    # [F] 신호 처리 노드 (Signal Processor)
    # 노이즈가 낀 센서 데이터를 필터링하는 노드입니다.
    # Runs the signal processor (filter).
    node_filter = Node(
        package=pkg_name,
        executable='signal_processor',
        name='signal_processor',
        output='screen'
    )

    # -------------------------------------------------------------------------
    # 4. 실행 목록 반환 (Return Description)
    # -------------------------------------------------------------------------
    return LaunchDescription([
        resource_env,                 # 1. 환경변수 설정 (가장 먼저!)
        gazebo,                       # 2. 가제보 실행
        node_robot_state_publisher,   # 3. 로봇 뼈대 방송
        spawn,                        # 4. 로봇 소환
        node_rviz,                    # 5. Rviz 실행
        node_filter,                  # 6. 필터 노드 실행
        node_teleop,                  # 7. 키보드 조종석 실행 (새 창 뜸)
        
        # [참고] Bridge는 아직 끕니다. (물리 엔진 충돌 방지)
        # Bridge is currently disabled.
    ])