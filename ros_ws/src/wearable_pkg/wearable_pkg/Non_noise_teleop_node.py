import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import sys, select, termios, tty

# =============================================================================
# 설정값 정의 (Configuration Constants)
# =============================================================================
CONST_MAX_ANGLE = 1.57  # 90도 (Radians)
CONST_STEP = 0.05       # 한 번 누를 때 움직이는 각도

class IntegratedHandTeleop(Node):
    """
    URDF 구조에 맞춘 키보드 제어 노드
    Matches the 'wearable_hand.urdf' joint structure.
    """
    def __init__(self):
        super().__init__('hand_teleop_node')
        
        # JointState 메시지 발행 (큐 사이즈 10)
        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)
        
        # 0.1초(10Hz)마다 타이머 실행
        self.timer = self.create_timer(0.1, self.timer_callback)

        # ---------------------------------------------------------
        # [중요] 변수 초기화 (URDF 관절과 1:1 매칭)
        # ---------------------------------------------------------
        self.shoulder_yaw = 0.0   # 어깨 회전 (Q/E)
        self.elbow_pitch = 0.0    # 팔꿈치 굽힘 (W/S)
        self.forearm_roll = 0.0   # 전완 회전 (A/D)
        self.wrist_pitch = 0.0    # 손목 꺾기 (Z/C) - 추가됨!
        
        # 손가락 굽힘 레벨 (1: 펴짐 ~ 10: 굽혀짐)
        self.flex_level = 1 

        # 터미널 설정 저장
        self.settings = termios.tcgetattr(sys.stdin)

        print("""
        =======================================================
        [Wearable Robot Control Map]
        -------------------------------------------------------
        Shoulder Yaw (Z)  : Q (Left)   / E (Right)
        Elbow Pitch (X)   : W (Flex)   / S (Extend)
        Forearm Roll (Z)  : A (Pronate)/ D (Supinate)
        Wrist Pitch (X)   : Z (Up)     / C (Down)  <-- NEW
        -------------------------------------------------------
        Index Finger      : 1 (Open) ~ 0 (Fully Closed)
        -------------------------------------------------------
        Quit              : CTRL + C
        =======================================================
        """)

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def map_level_to_radian(self, level):
        # 레벨 1~10을 0 ~ 1.57 라디안으로 변환
        return (level - 1) * (CONST_MAX_ANGLE / 9.0)

    def timer_callback(self):
        key = self.get_key()

        # 1. 어깨 (Shoulder Yaw) - Q/E
        if key == 'q':
            self.shoulder_yaw = min(self.shoulder_yaw + CONST_STEP, CONST_MAX_ANGLE)
        elif key == 'e':
            self.shoulder_yaw = max(self.shoulder_yaw - CONST_STEP, -CONST_MAX_ANGLE)

        # 2. 팔꿈치 (Elbow Pitch) - W/S
        elif key == 'w':
            self.elbow_pitch = min(self.elbow_pitch + CONST_STEP, 2.0) # 팔꿈치는 좀 더 굽혀짐
        elif key == 's':
            self.elbow_pitch = max(self.elbow_pitch - CONST_STEP, 0.0)

        # 3. 전완 (Forearm Roll) - A/D
        elif key == 'a':
            self.forearm_roll = max(self.forearm_roll - CONST_STEP, -CONST_MAX_ANGLE)
        elif key == 'd':
            self.forearm_roll = min(self.forearm_roll + CONST_STEP, CONST_MAX_ANGLE)

        # 4. 손목 (Wrist Pitch) - Z/C (새로 추가)
        elif key == 'z':
            self.wrist_pitch = max(self.wrist_pitch - CONST_STEP, -0.5)
        elif key == 'c':
            self.wrist_pitch = min(self.wrist_pitch + CONST_STEP, 0.5)

        # 5. 손가락 (Finger) - 1~0
        elif key in ['1', '2', '3', '4', '5', '6', '7', '8', '9']:
            self.flex_level = int(key)
        elif key == '0':
            self.flex_level = 10

        # 종료 (Ctrl+C)
        if key == '\x03':
            self.destroy_node()
            rclpy.shutdown()

        # ---------------------------------------------------------
        # [핵심] JointState 메시지 생성 및 발행
        # ---------------------------------------------------------
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        
        # URDF 파일에 있는 joint 이름과 정확히 일치해야 합니다.
        msg.name = [
            'r_shoulder_yaw', 
            'r_elbow_pitch', 
            'r_forearm_roll', 
            'r_wrist_pitch',
            'r_index_knuckle', 'r_index_mid', 'r_index_tip' # 손가락 3마디
        ]
        
        # 손가락 각도 계산 (하나의 레벨로 3마디를 동시에 움직임)
        finger_angle = self.map_level_to_radian(self.flex_level)

        msg.position = [
            self.shoulder_yaw,
            self.elbow_pitch,
            self.forearm_roll,
            self.wrist_pitch,
            finger_angle, finger_angle, finger_angle # 3마디에 같은 각도 적용
        ]

        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = IntegratedHandTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()