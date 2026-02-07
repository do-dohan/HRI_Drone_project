from setuptools import setup             # 파이썬 패키지를 빌드하고 설치하기 위한 표준 도구인 setup 함수를 가져옵니다.
                                         # Import setup function for Python package installation.
import os                                # 운영체제(OS)의 파일 경로를 다루기 위해(예: 폴더 경로 합치기) 가져옵니다.
                                         # Import os module for path manipulation.
from glob import glob                    # 특정 패턴(*.stl 등)을 가진 파일들의 목록을 한 번에 찾기 위해 가져옵니다.
                                         # Import glob to find files matching a pattern.

package_name = 'wearable_pkg'            # 이 패키지의 이름을 변수에 저장합니다. 나중에 여러 군데서 쓰입니다.
                                         # Define the package name variable.

setup(
    name=package_name,                   # 패키지의 이름을 설정합니다. (wearable_pkg)
                                         # Set the package name.
    version='0.0.0',                     # 패키지의 버전을 설정합니다.
                                         # Set the package version.
    packages=[package_name],             # 포함할 파이썬 패키지들의 목록입니다. 보통 패키지 이름과 같습니다.
                                         # List of Python packages to include.
    
    # [중요] 데이터 파일 설치 규칙: (설치될 위치, [원본 파일들]) 형식의 튜플 리스트입니다.
    # [Important] Data files mapping: (Install destination, [Source files]).
    data_files=[
        # 1. 패키지 인덱스 등록
        # ROS 2가 이 패키지의 존재를 알 수 있도록 마커 파일을 설치합니다.
        # Install marker file so ROS 2 can find this package.
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        
        # 2. package.xml 설치
        # 패키지의 메타데이터(의존성 등)가 담긴 package.xml 파일을 share 폴더로 복사합니다.
        # Install package.xml to the share directory.
        ('share/' + package_name, ['package.xml']),
        
        # 3. URDF 파일 설치 (매우 중요)
        # 'urdf' 폴더에 있는 모든 .urdf 파일을 찾아서, 설치 경로의 'share/wearable_pkg/urdf'로 복사합니다.
        # Copy all .urdf files from local 'urdf/' to install 'share/<pkg>/urdf/'.
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
        
        # 4. 메쉬 파일 설치 (매우 중요)
        # 'meshes' 폴더에 있는 모든 .stl 파일을 찾아서, 설치 경로의 'share/wearable_pkg/meshes'로 복사합니다.
        # 이게 없으면 Rviz에서 로봇의 3D 모델(껍데기)이 하얗게 나오거나 안 보입니다.
        # Copy all .stl files to install 'share/<pkg>/meshes/'. Critical for Rviz visualization.
        (os.path.join('share', package_name, 'meshes'), glob('meshes/*.stl')),

        # [NEW] Launch 파일 설치
        # launch 폴더의 모든 .py 파일을 share/wearable_pkg/launch 폴더로 복사합니다.
        # Install launch files.
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    
    install_requires=['setuptools'],     # 이 패키지를 설치할 때 필요한 파이썬 의존성 라이브러리 목록입니다.
                                         # List of Python dependencies required for installation.
    zip_safe=True,                       # 패키지를 압축된 상태로 실행할지 여부입니다. ROS 2에서는 보통 True로 둡니다.
                                         # Flag to indicate if the package is zip-safe.
    maintainer='YuHan_Do',                   # 패키지 관리자의 이름입니다.
                                         # Package maintainer's name.
    maintainer_email='dyh111@konkuk.ac.kr',   # 패키지 관리자의 이메일 주소입니다.
                                         # Package maintainer's email.
    description='Wearable Hand Description', # 패키지에 대한 짧은 설명입니다.
                                             # Short description of the package.
    license='Apache-2.0',                      # 패키지의 라이선스(저작권) 정보입니다.
                                         # License type.
    tests_require=['pytest'],            # 테스트를 실행할 때 필요한 라이브러리 목록입니다.
                                         # Dependencies required for testing.
    
    # [핵심] 실행 파일(노드) 등록
    # 터미널에서 'ros2 run' 명령어로 실행할 수 있는 스크립트를 정의합니다.
    # Define executable scripts runnable via 'ros2 run'.
    entry_points={
        'console_scripts': [
            # 형식: '실행할_명령어_이름 = 패키지폴더.파이썬파일:함수이름'
            # Format: 'command_name = package.module:function'
            
            # 터미널에 'ros2 run wearable_pkg teleop_node'를 치면 -> teleop_node.py의 main 함수를 실행하라는 뜻입니다.
            # Maps 'teleop_node' command to the main function in teleop_node.py.
            'teleop_node = wearable_pkg.teleop_node:main',
            # [중요] 이 줄이 없거나 주석(#) 처리되어 있으면 에러가 납니다!
            'signal_processor = wearable_pkg.signal_processor:main',
        ],
    },
)