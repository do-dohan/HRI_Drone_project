import rclpy # ROS2 파이썬 클라이언트 라이브러리 / ROS2 Python client library
from rclpy.node import Node # 노드 클래스 / Node class
from sensor_msgs.msg import JointState # 조인트 상태 메시지 / Joint state message
import sys, select, termios, tty # 키보드 입력을 위한 시스템 라이브러리 / System libraries for keyboard input

class WristTeleop(Node):
    def __init__(self):
        super().__init__('wrist_teleop') # 노드 이름 초기화 / Initialize node name
        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10) # 변수: 퍼블리셔 (토픽 이름 확인 필요) / Publisher
        
        # 현재 Roll, Pitch, Yaw 각도 저장 (단위: 라디안)
        # Store current RPY angles (Unit: Radians)
        self.r = 0.0 # Roll
        self.p = 0.0 # Pitch
        self.y = 0.0 # Yaw
        self.step = 0.05 # 변수: 한번 누를 때 움직일 각도 (약 3도) / Angle increment per press

    def get_key(self):
        # 키보드 입력을 실시간으로 읽어오는 함수 (표준 코드)
        # Function to read keyboard input in real-time
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key

    def update_joint_state(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['wrist_roll_joint', 'wrist_pitch_joint', 'wrist_yaw_joint'] # 조인트 이름 정의 / Define joint names
        msg.position = [self.r, self.p, self.y] # 현재 계산된 각도 대입 / Assign calculated angles
        self.publisher_.publish(msg) # 메시지 발행 / Publish message

# 메인 실행 로직은 생략 (개념 설명 위주)