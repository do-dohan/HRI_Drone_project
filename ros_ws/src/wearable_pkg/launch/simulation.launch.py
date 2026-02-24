import os                                             
# 운영체제 기능을 사용하여 파일 경로 등을 다루기 위해 가져옵니다.
# Import OS module for path manipulation.

from ament_index_python.packages import get_package_share_directory 
# 설치된 ROS 패키지의 위치를 찾아주는 함수입니다.
# Helper to find package directories.

from launch import LaunchDescription                  
# 런치 파일을 구성하는 최상위 컨테이너입니다.
# Main container for launch description.

from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable 
# 다른 런치 파일을 실행하거나 환경 변수를 설정하는 액션입니다.
# Actions to include files and set env vars.

from launch.launch_description_sources import PythonLaunchDescriptionSource 
# 파이썬으로 작성된 런치 파일을 읽어오는 도구입니다.
# Source handler for Python launch files.

from launch_ros.actions import Node                   
# ROS 2 노드(실행 파일)를 실행하는 액션입니다.
# Action to run a ROS 2 node.

import xacro                                          
# URDF 로봇 모델 파일을 파싱(해석)하는 도구입니다.
# Library to parse URDF files.

def generate_launch_description():
    # -------------------------------------------------------------------------
    # 1. 기본 설정 (Configuration)
    # -------------------------------------------------------------------------
    pkg_name = 'wearable_pkg'                         
    # [변수] 패키지 이름을 변수에 담아 재사용성을 높입니다.
    # [Variable] Store package name for reusability.
    
    urdf_file = 'wearable_hand.urdf'                  
    # [변수] 사용할 로봇 모델 파일명입니다.
    # [Variable] Robot model filename.
    
    pkg_path = get_package_share_directory(pkg_name)  
    # 패키지의 설치 경로를 가져옵니다.
    # Get installed package path.
    
    urdf_path = os.path.join(pkg_path, 'urdf', urdf_file) 
    # URDF 파일의 절대 경로를 완성합니다.
    # Build full path to URDF.

    # -------------------------------------------------------------------------
    # [환경 변수] 가제보 리소스 경로 설정
    # -------------------------------------------------------------------------
    install_dir = os.path.join(get_package_share_directory(pkg_name), '..')
    resource_env = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=install_dir
    )

    # -------------------------------------------------------------------------
    # 2. 로봇 모델 파싱 (Parse URDF)
    # -------------------------------------------------------------------------
    doc = xacro.parse(open(urdf_path))                
    # URDF 파일을 엽니다.
    # Open URDF file.
    
    xacro.process_doc(doc)                            
    # Xacro 매크로를 처리합니다.
    # Process macros.
    
    params = {'robot_description': doc.toxml(), 'use_sim_time': True}      
    # XML 내용을 파라미터 딕셔너리로 만듭니다.
    # Create parameter dictionary.

    # -------------------------------------------------------------------------
    # 3. 노드 정의 (Nodes Definition)
    # -------------------------------------------------------------------------

    # [A] 로봇 상태 퍼블리셔
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    # [B] 가제보 시뮬레이터
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-r empty.sdf'}.items(),
    )

    # [C] 로봇 소환 (Spawn)
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'wearable_hand',
            '-topic', 'robot_description',
            '-z', '1.0'
        ],
        output='screen',
    )
    
    # [D] Rviz2 시각화
    node_rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen'
    )

    # -------------------------------------------------------------------------
    # [E-1] 키보드 조종 노드 (Teleop - 보존용)
    # -------------------------------------------------------------------------
    node_teleop = Node(
        package=pkg_name,
        executable='teleop_node',
        # 수동 키보드 조종 노드의 실행 파일 이름입니다.
        # Executable name of the manual keyboard control node.
        name='teleop_node',
        output='screen',
        prefix='xterm -e'                             
        # 수동 조종을 위한 별도의 xterm 터미널 창을 띄웁니다.
        # Open separate xterm terminal for manual control input.
    )

    # -------------------------------------------------------------------------
    # [E-2] 시나리오 드라이버 노드 (자동화 궤적 생성)
    # -------------------------------------------------------------------------
    node_scenario = Node(
        package=pkg_name,
        executable='scenario_driver',                 
        # setup.py에 등록된 시나리오 노드의 실행 이름입니다.
        # Executable name of the scenario node from setup.py.
        name='scenario_driver_node',                  
        output='screen',                              
        parameters=[
            {'rate_hz': 100},                         
            {'slow_rate_divisor': 2},                 
            {'loop_sec': 45.0},                       
        ]
    )

    # -------------------------------------------------------------------------
    # [F] 신호 처리 노드 (필터 수신부)
    # -------------------------------------------------------------------------
    node_filter = Node(
        package=pkg_name,
        executable='signal_processor',                
        name='signal_processor',
        output='screen'
    )

    # -------------------------------------------------------------------------
    # [G] ROS-GZ 브릿지 (생명 연결)
    # -------------------------------------------------------------------------
    node_ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/world/empty/model/wearable_hand/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model',
            
            '/model/wearable_hand/joint/r_shoulder_yaw/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
            '/model/wearable_hand/joint/r_elbow_pitch/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
            '/model/wearable_hand/joint/r_forearm_roll/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
            '/model/wearable_hand/joint/r_wrist_pitch/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
            '/model/wearable_hand/joint/r_index_knuckle/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
            '/model/wearable_hand/joint/r_index_mid/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
            '/model/wearable_hand/joint/r_index_tip/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
        ],
        remappings=[
            ('/world/empty/model/wearable_hand/joint_state', '/joint_states'),
        ],
        output='screen'
    )

    # [H] Foxglove Bridge 
    node_foxglove_bridge = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        output='screen',
        parameters=[{'port': 8765}]
    )

    # -------------------------------------------------------------------------
    # 4. 실행 목록 반환 (Return Description)
    # -------------------------------------------------------------------------
    return LaunchDescription([
        resource_env,                 
        gazebo,                       
        node_robot_state_publisher,   
        spawn,                        
        node_rviz,                    
        
        # [실행 선택 구역 - 충돌을 막기 위해 셋 중 하나만 주석을 해제하여 켭니다]
        # [Execution Zone - Uncomment ONLY ONE of the three to prevent conflicts]
        
        # 1. 수동 조종 모드 (현재 꺼짐)
        # 1. Manual Teleop Mode (Currently OFF)
        # node_teleop,                  
        
        # 2. 시나리오 자동 모드 + 가제보 직접 제어 (현재 켜짐)
        # 2. Auto Scenario Mode + Direct Gazebo Control (Currently ON)
        node_scenario,                
        
        # 3. 진짜 수신부 필터 (현재 꺼짐 - 치트 모드와 충돌 방지)
        # 3. Real Filter Receiver (Currently OFF - Prevents collision with cheat mode)
        # node_filter,                  
                                      
        node_ros_gz_bridge,
        node_foxglove_bridge,
    ])