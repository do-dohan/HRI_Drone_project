import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
import xacro

def generate_launch_description():
    pkg_name = 'wearable_pkg'
    urdf_file = 'wearable_hand.urdf'
    
    pkg_path = get_package_share_directory(pkg_name)
    urdf_path = os.path.join(pkg_path, 'urdf', urdf_file)

    # 1. URDF 파싱
    doc = xacro.parse(open(urdf_path))
    xacro.process_doc(doc)
    params = {'robot_description': doc.toxml()}

    # 2. Robot State Publisher (동일)
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    # 3. Gazebo Harmonic 실행 (명령어 변경됨!)
    # 'gz_args'에 '-r'을 넣으면 시뮬레이션이 자동으로 시작됩니다.
    # empty.sdf 월드를 엽니다.
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-r empty.sdf'}.items(),
    )

    # 4. 로봇 소환 (spawn_entity -> create)
    # ros_gz_sim 패키지의 create 노드를 사용합니다.
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'wearable_hand',
            '-topic', 'robot_description', # robot_description 토픽에서 모델을 가져옴
            '-z', '1.0'
        ],
        output='screen',
    )
    
    # 5. Bridge (브릿지) - 매우 중요!
    # Gazebo의 토픽과 ROS2 토픽을 연결해줍니다.
    # 지금은 joint_states만 연결합니다.
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
        ],
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        node_robot_state_publisher,
        spawn,
        # bridge # (일단 모델 보는게 목적이면 브릿지는 나중에 켜도 됩니다)
    ])