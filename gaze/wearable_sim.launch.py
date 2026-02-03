import os
from ament_index_python.packages import get_package_share_directory # 패키지 경로를 찾기 위한 함수 / Function to find package path
from launch import LaunchDescription # 런치 파일을 정의하는 메인 클래스 / Main class for launch file
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler # 프로세스 실행 및 이벤트 핸들러 / Process execution & event handlers
from launch.event_handlers import OnProcessExit # 프로세스 종료 시 이벤트 처리 / Handle process exit events
from launch.launch_description_sources import PythonLaunchDescriptionSource # 파이썬 런치 소스 로딩 / Load Python launch source
from launch_ros.actions import Node # ROS2 노드 실행 액션 / Action to run ROS2 node
import xacro # URDF 파일을 처리하기 위한 라이브러리 / Library to process URDF

def generate_launch_description():
    # 1. 파일 경로 설정 (사용자 환경에 맞게 패키지 이름 수정 필요)
    # Set file paths (Update 'wearable_pkg' to your package name).
    package_name = 'wearable_pkg' 
    urdf_file_name = 'wearable_hand.urdf'

    # URDF 파일의 절대 경로를 가져옵니다.
    # Get absolute path of the URDF file.
    urdf_path = os.path.join(
        get_package_share_directory(package_name),
        'urdf',
        urdf_file_name
    )

    # 2. 로봇 상태 퍼블리셔 (Robot State Publisher) 설정
    # URDF 파일을 읽어서 로봇의 링크(Link)와 조인트(Joint) 관계를 ROS에 알려주는 노드입니다.
    # Node that reads URDF and broadcasts robot structure (TF).
    doc = xacro.parse(open(urdf_path))
    xacro.process_doc(doc)
    robot_description = {'robot_description': doc.toxml()}

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    # 3. 가제보(Gazebo) 실행 설정
    # 가제보 서버와 클라이언트를 실행하여 시뮬레이션 환경을 엽니다.
    # Launch Gazebo server and client.
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
    )

    # 4. 로봇 소환 (Spawn Entity)
    # 가제보 월드 안에 우리가 만든 로봇 모델을 생성합니다.
    # Spawn the robot model into the Gazebo world.
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description',
                   '-entity', 'wearable_hand'], # 로봇 이름 설정 / Set robot entity name
        output='screen'
    )

    # 5. 통합 제어 노드 실행 (Integrated Teleop Node)
    # 방금 만든 파이썬 키보드 제어 코드를 실행합니다.
    # Run the Python keyboard control script created earlier.
    # (주의: setup.py의 entry_points에 'hand_teleop'으로 등록되어 있어야 함)
    node_hand_teleop = Node(
        package=package_name,
        executable='hand_teleop', # setup.py에서 설정한 실행 이름 / Executable name from setup.py
        name='hand_teleop_node',
        output='screen',
        prefix='xterm -e' # [중요] 키보드 입력을 받기 위해 별도 터미널 창을 띄웁니다.
                          # [Important] Opens a new terminal window for keyboard input.
    )

    # 6. 실행 흐름 정의 (Launch Description)
    # 위에서 정의한 모든 요소들을 순서대로 실행합니다.
    # Return the LaunchDescription containing all defined actions.
    return LaunchDescription([
        gazebo,
        node_robot_state_publisher,
        spawn_entity,
        node_hand_teleop
    ])