import rclpy                                     # ROS2 통신을 위한 핵심 라이브러리입니다.
                                                 # Core library for ROS2 communication.
from rclpy.node import Node                      # 노드 클래스를 불러옵니다.
                                                 # Import Node class.
from sensor_msgs.msg import JointState           # 관절 상태를 제어할 메시지 타입입니다.
                                                 # Message type for joint control.
import sys, select, termios, tty                 # 키보드 입력을 처리하는 시스템 라이브러리입니다.
                                                 # System libraries for keyboard input.

# =============================================================================
# 설정값 정의 (Configuration Constants)
# 변수: 이 값들은 필요에 따라 변경할 수 있습니다.
# Variables: These values can be modified as needed.
# =============================================================================
CONST_MAX_ANGLE = 1.57  # 최대 회전 각도 (라디안 단위, 약 90도).
                        # Max rotation angle in radians.
CONST_STEP = 0.05       # 키를 한 번 누를 때 움직일 각도의 크기.
                        # Angle step size per key press.

class IntegratedHandTeleop(Node):
    """
    키보드 입력을 받아 팔꿈치(Elbow) 회전과 손가락(Finger) 굽힘을 제어하는 노드입니다.
    Node that controls Elbow rotation and Finger flexion via keyboard input.
    """
    def __init__(self):
        # 1. 노드 초기화
        # 부모 클래스인 Node를 초기화하며 이름을 'hand_teleop_node'로 짓습니다.
        # Initialize Node with name 'hand_teleop_node'.
        super().__init__('hand_teleop_node')

        # 2. 퍼블리셔(Publisher) 생성
        # 'joint_states'라는 주제(Topic)로 메시지를 방송합니다. 큐 사이즈는 10입니다.
        # Publish messages to 'joint_states' topic with queue size 10.
        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)

        # 3. 타이머 설정
        # 0.1초(10Hz)마다 self.timer_callback 함수를 실행하도록 설정합니다.
        # Set timer to call 'timer_callback' every 0.1s.
        self.timer_period = 0.1
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        # 4. 관절 각도 변수 초기화 (모두 0도에서 시작)
        # Initialize joint angle variables to zero.
        self.roll = 0.0   # 팔꿈치 Roll (Elbow Roll)
        self.pitch = 0.0  # 팔꿈치 Pitch (Elbow Pitch)
        self.yaw = 0.0    # 팔꿈치 Yaw (Elbow Yaw)
        
        # 5. Flex 센서 레벨 변수 (1 ~ 10 단계)
        # Flex sensor level variable (Range 1-10).
        self.flex_level = 1 # 1: 펴짐(Open), 10: 쥐어짐(Closed)

        # 6. 사용자 안내 메시지 출력
        # Print user instructions.
        print("""
        =======================================================
        [Keyboard Control Map]
        -------------------------------------------------------
        Elbow Roll (X-axis)  : A (Left) / D (Right)
        Elbow Pitch (Y-axis) : W (Up)   / S (Down)
        Elbow Yaw (Z-axis)   : Q (Left) / E (Right)
        -------------------------------------------------------
        Index Finger Flex    : 1 (Open) ~ 0 (Fully Closed)
        -------------------------------------------------------
        Quit                 : CTRL + C
        =======================================================
        """)
        
        # 현재 터미널 설정을 저장합니다 (종료 시 복구하기 위함).
        # Save current terminal settings to restore later.
        self.settings = termios.tcgetattr(sys.stdin)

    def get_key(self):
        """
        엔터 키 없이 키보드 입력을 즉시 받아오는 함수입니다.
        Function to capture key input immediately without Enter.
        """
        # 터미널을 Raw 모드로 전환하여 입력을 가로챕니다.
        # Switch terminal to Raw mode to intercept input.
        tty.setraw(sys.stdin.fileno())
        
        # 입력이 들어왔는지 확인합니다 (0.1초 대기).
        # Check if input is available (wait 0.1s).
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        
        if rlist:
            # 입력이 있다면 한 글자를 읽어옵니다.
            # Read one character if input exists.
            key = sys.stdin.read(1)
        else:
            # 입력이 없다면 빈 문자열을 반환합니다.
            # Return empty string if no input.
            key = ''
            
        # 터미널 설정을 원래대로 복구합니다.
        # Restore terminal settings.
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def map_level_to_radian(self, level):
        """
        1~10단계의 정수 레벨을 0 ~ 1.57 라디안(90도) 각도로 변환합니다.
        Maps integer level (1-10) to radian angle (0-1.57).
        """
        # (현재레벨 - 1) / 9 * 최대각도 공식을 사용합니다.
        # Use linear interpolation formula.
        radian = (level - 1) * (CONST_MAX_ANGLE / 9.0)
        return radian

    def timer_callback(self):
        """
        주기적으로 호출되어 키 입력을 처리하고 로봇 상태를 업데이트하는 함수입니다.
        Periodic callback to handle input and update robot state.
        """
        # 1. 키보드 입력 받기
        # Get keyboard input.
        key = self.get_key()

        # 2. 팔꿈치 각도 업데이트 (최소/최대값 제한 포함)
        # Update Elbow angles with clamping.
        if key == 'w':
            self.pitch = min(self.pitch + CONST_STEP, CONST_MAX_ANGLE)
        elif key == 's':
            self.pitch = max(self.pitch - CONST_STEP, -CONST_MAX_ANGLE)
            
        elif key == 'a':
            self.roll = max(self.roll - CONST_STEP, -CONST_MAX_ANGLE) # 방향 주의 / Direction check
        elif key == 'd':
            self.roll = min(self.roll + CONST_STEP, CONST_MAX_ANGLE)
            
        elif key == 'q':
            self.yaw = max(self.yaw - CONST_STEP, -CONST_MAX_ANGLE)
        elif key == 'e':
            self.yaw = min(self.yaw + CONST_STEP, CONST_MAX_ANGLE)

        # 3. 손가락 굽힘 레벨 업데이트 (숫자키 1~9, 0)
        # Update Finger flex level (Keys 1-9, 0).
        if key in ['1', '2', '3', '4', '5', '6', '7', '8', '9']:
            self.flex_level = int(key)
        elif key == '0':
            self.flex_level = 10 # 0키는 레벨 10으로 처리 / Key 0 is Level 10

        # 4. 강제 종료 처리 (Ctrl + C)
        # Handle force quit.
        if key == '\x03': # Ctrl+C의 아스키 코드
            self.destroy_node()
            rclpy.shutdown()

        # 5. JointState 메시지 생성
        # Create JointState message.
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg() # 현재 시간 / Current time
        
        # [중요] URDF에 정의된 조인트 이름과 순서, 스펠링이 정확히 일치해야 합니다.
        # [Important] Names must match URDF exactly.
        msg.name = ['elbow_roll_joint', 'elbow_pitch_joint', 'elbow_yaw_joint', 'index_finger_joint']
        
        # 계산된 각도들을 리스트로 할당합니다.
        # Assign calculated angles.
        msg.position = [
            self.roll, 
            self.pitch, 
            self.yaw, 
            self.map_level_to_radian(self.flex_level)
        ]

        # 6. 메시지 발행 (가제보로 전송)
        # Publish message to Gazebo.
        self.publisher_.publish(msg)

def main(args=None):
    # ROS2 초기화
    # Initialize ROS2.
    rclpy.init(args=args)
    
    # 노드 객체 생성
    # Create node object.
    node = IntegratedHandTeleop()
    
    # 노드 실행 (종료 전까지 반복)
    # Spin node until exit.
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 종료 시 리소스 정리
        # Cleanup on exit.
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()