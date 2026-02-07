import rclpy                                     # ROS 2의 핵심 기능을 담고 있는 파이썬 라이브러리를 불러옵니다.
                                                 # Import ROS 2 Python client library.
from rclpy.node import Node                      # ROS 노드(로봇의 뇌세포)를 만들기 위한 클래스입니다.
                                                 # Import Node class to create a ROS node.
from sensor_msgs.msg import JointState, Imu      # 로봇 관절 상태와 IMU 센서 데이터를 다루기 위한 메시지 양식입니다.
                                                 # Import message types for joint states and IMU data.
from geometry_msgs.msg import Vector3            # X, Y, Z 3차원 데이터를 다루기 위한 벡터 메시지입니다 (오일러 각도용).
                                                 # Import Vector3 message for Euler angles.
from std_msgs.msg import Float32                 # 소수점 숫자 하나를 보내기 위한 가장 기본적인 메시지입니다 (Flex 센서용).
                                                 # Import Float32 message for simple sensor data.
# 시스템 관련 파라미터와 함수를 제어합니다. (표준 입력/출력 처리)
# Provides access to system-specific parameters and functions.
import sys

# 파일 디스크립터(터미널 입력 등)의 상태를 대기하고 감시합니다.
# Monitors and waits for status changes in file descriptors (e.g., keyboard input).
import select

# 리눅스의 시리얼 포트와 터미널의 통신 속도, 모드 등을 제어합니다.
# Controls terminal attributes and serial communication settings in Linux.
import termios

# 터미널을 'Raw 모드'로 바꿔 키보드 입력을 즉시 읽을 수 있게 합니다.
# Changes the terminal to 'Raw mode' to read keyboard input immediately.
import tty
import math                                      # 삼각함수(sin, cos) 등 수학 계산을 위한 도구입니다.
                                                 # Standard math library for trigonometric functions.
import random                                    # 가짜 노이즈를 만들기 위해 무작위 숫자를 뽑는 도구입니다.
                                                 # Library for generating random numbers (noise).
import threading                                 # 프로그램이 두 가지 일(키보드 감시, 로봇 제어)을 동시에 하게 만드는 도구입니다.
                                                 # Library for multi-threading (concurrency).

# =============================================================================
# 설정값 정의 (Configuration Constants)
# 변수 이름만 봐도 내용을 알 수 있게 상수로 정의해둡니다.
# Define constants for easy tuning and readability.
# =============================================================================
CONST_MAX_ANGLE = 1.57  # 로봇 팔이 움직일 수 있는 최대 각도입니다 (약 90도, 라디안 단위).
                        # Max rotation angle in radians (approx 90 deg).
CONST_STEP = 0.05       # 키보드를 한 번 눌렀을 때 목표 각도가 변하는 양입니다.
                        # Target angle increment step per key press.
TIMER_PERIOD = 0.02     # 로봇 상태를 업데이트하는 주기입니다 (0.02초 = 1초에 50번 = 50Hz).
                        # Update loop period (0.02s = 50Hz).
SMOOTH_FACTOR = 0.15    # 현재 위치가 목표 위치를 따라가는 속도 비율입니다 (낮을수록 부드럽고 묵직함).
                        # Smoothing factor (Linear Interpolation ratio).

# [손가락 속도 설정]
# 위치를 바로 지정하는 게 아니라, 모터가 돌아가는 속도를 정의합니다.
# Finger velocity settings (Level change per tick).
VEL_SLOW = 0.02         # 아주 천천히 움직일 때의 속도입니다.
                        # Slow velocity.
VEL_MID  = 0.08         # 보통 속도입니다.
                        # Medium velocity.
VEL_FAST = 0.20         # 빠르게 움직일 때의 속도입니다.
                        # Fast velocity.

# [센서 노이즈 설정]
# 시뮬레이션을 현실처럼 만들기 위해 깨끗한 값에 더해줄 잡음의 크기입니다.
# Noise standard deviation levels for simulation realism.
NOISE_IMU_ACCEL = 0.02  # 가속도 센서의 노이즈 크기입니다.
NOISE_IMU_ORI = 0.01    # 기울기(각도) 센서의 노이즈 크기입니다.
NOISE_FLEX = 0.1        # 휨 센서(Flex)의 노이즈 크기입니다.


class WearableSensorSim(Node):
    """
    ROS 2 노드 클래스: 가상 웨어러블 로봇 팔을 시뮬레이션합니다.
    ROS 2 Node class simulating a wearable robotic arm.
    """
    def __init__(self):
        # 부모 클래스(Node)를 초기화하면서 노드 이름을 'wearable_sensor_sim'으로 짓습니다.
        # Initialize the parent Node class with name 'wearable_sensor_sim'.
        super().__init__('wearable_sensor_sim')
        
        # ---------------------------------------------------------
        # 1. 퍼블리셔(Publisher) 설정: 데이터를 밖으로 내보내는 입구들
        # ---------------------------------------------------------
        # Rviz가 로봇을 그릴 수 있도록 관절 상태를 보냅니다.
        # Publish joint states for visualization in Rviz.
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)
        
        # 가상 IMU 센서 데이터(손목, 전완)를 보냅니다.
        # Publish simulated IMU data (Wrist, Forearm).
        self.imu_wrist_pub = self.create_publisher(Imu, '/wearable/imu/wrist/raw', 10)
        self.imu_forearm_pub = self.create_publisher(Imu, '/wearable/imu/forearm/raw', 10)
        
        # 가상 Flex 센서 데이터를 보냅니다.
        # Publish simulated Flex sensor data.
        self.flex_pub = self.create_publisher(Float32, '/wearable/flex/raw', 10)
        
        # 디버깅하기 쉽도록 오일러 각도(직관적인 각도)도 따로 보냅니다.
        # Publish Euler angles for easier debugging.
        self.euler_wrist_pub = self.create_publisher(Vector3, '/wearable/imu/wrist/euler', 10)
        self.euler_forearm_pub = self.create_publisher(Vector3, '/wearable/imu/forearm/euler', 10)
        
        # 주기적으로 timer_callback 함수를 실행할 타이머를 가동합니다 (50Hz).
        # Start a timer to run the main loop at 50Hz.
        self.timer = self.create_timer(TIMER_PERIOD, self.timer_callback)

        # ---------------------------------------------------------
        # 2. 상태 변수 초기화: 목표값(Target) vs 현재값(Current)
        # ---------------------------------------------------------
        # [목표값] 키보드로 사용자가 "여기까지 가라"고 명령한 위치입니다.
        # Target values: Desired positions set by user input.
        self.target_shoulder_yaw = 0.0
        self.target_elbow_pitch = 0.0
        self.target_forearm_roll = 0.0
        self.target_wrist_pitch = 0.0
        
        # [현재값] 실제 로봇이 부드럽게 움직이며 도달한 현재 위치입니다.
        # Current values: Actual interpolated positions of the robot.
        self.curr_shoulder_yaw = 0.0
        self.curr_elbow_pitch = 0.0
        self.curr_forearm_roll = 0.0
        self.curr_wrist_pitch = 0.0

        # [손가락 제어 변수] 손가락은 위치가 아니라 '속도'로 제어합니다.
        # Finger control variables (Velocity control).
        self.curr_flex_level = 1.0  # 현재 손가락 굽힘 정도 (1:펴짐 ~ 10:쥐어짐).
                                    # Current finger flex level (1-10).
        self.flex_velocity = 0.0    # 현재 손가락이 움직이는 속도입니다 (0이면 정지).
                                    # Current finger moving velocity.

        # [쓰레드 락] 키보드 입력 쓰레드와 메인 쓰레드가 서로 변수를 건드려 충돌하지 않게 막는 안전장치입니다.
        # Mutex lock to prevent data races between threads.
        self.lock = threading.Lock()

        # [상태 변수 추가] 현재 눌려있는 키들을 담는 집합 (중복 방지)
        # Set to keep track of currently pressed keys.
        self.pressed_keys = set()

        # 시작 메시지 출력
        print(">>> Velocity Control Mode Activated <<<")
        print("Keys 1-3: Flex / 4-6: Extend / 0: Stop")

    # -------------------------------------------------------------------------
    # [직원 A] 키보드 입력 처리 담당 함수 (별도 쓰레드에서 실행됨)
    # Worker A: Handles keyboard input updates (Runs in a separate thread).
    # -------------------------------------------------------------------------
    def update_target(self):
        
        #눌려있는 모든 키(self.pressed_keys)를 검사하여 
        #여러 관절을 동시에 움직입니다.
        with self.lock:
            # --- [그룹 1] Shoulder Yaw (Q, E) ---
            if 'q' in self.pressed_keys:
                self.target_shoulder_yaw = min(self.target_shoulder_yaw + CONST_STEP, 1.57)
            if 'e' in self.pressed_keys:
                self.target_shoulder_yaw = max(self.target_shoulder_yaw - CONST_STEP, -1.57)

            # --- [그룹 2] Elbow Pitch (W, S) ---
            if 'w' in self.pressed_keys:
                self.target_elbow_pitch = min(self.target_elbow_pitch + CONST_STEP, 2.0)
            if 's' in self.pressed_keys:
                self.target_elbow_pitch = max(self.target_elbow_pitch - CONST_STEP, 0.0)

            # --- [그룹 3] Forearm Roll (A, D) ---
            if 'a' in self.pressed_keys:
                self.target_forearm_roll = max(self.target_forearm_roll - CONST_STEP, -3.14)
            if 'd' in self.pressed_keys:
                self.target_forearm_roll = min(self.target_forearm_roll + CONST_STEP, 3.14)

            # --- [그룹 4] Wrist Pitch (Z, C) ---
            if 'z' in self.pressed_keys:
                self.target_wrist_pitch = max(self.target_wrist_pitch - CONST_STEP, -0.5)
            if 'c' in self.pressed_keys:
                self.target_wrist_pitch = min(self.target_wrist_pitch + CONST_STEP, 0.5)

            # --- [그룹 5] Finger Flex (1~6) ---
            # 손가락은 속도 제어이므로, 눌린 키 중 가장 높은 속도를 하나 선택합니다 (elif 사용 가능).
            if '1' in self.pressed_keys: self.flex_velocity = VEL_SLOW
            elif '2' in self.pressed_keys: self.flex_velocity = VEL_MID
            elif '3' in self.pressed_keys: self.flex_velocity = VEL_FAST
            elif '4' in self.pressed_keys: self.flex_velocity = -VEL_SLOW
            elif '5' in self.pressed_keys: self.flex_velocity = -VEL_MID
            elif '6' in self.pressed_keys: self.flex_velocity = -VEL_FAST
            else: self.flex_velocity = 0.0

    # -------------------------------------------------------------------------
    # [수학 도구] 오일러 각도(롤,피치,요)를 쿼터니언(x,y,z,w)으로 변환
    # Math Helper: Converts Euler angles to Quaternion.
    # -------------------------------------------------------------------------
    def euler_to_quaternion(self, roll, pitch, yaw):
        # 복잡한 삼각함수 공식입니다. 로봇이나 드론은 '짐벌 락(회전 축이 겹쳐 마비되는 현상)'을 피하기 위해 쿼터니언을 씁니다.
        # Standard formula to convert Euler to Quaternion to avoid Gimbal Lock.
        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        return [qx, qy, qz, qw]

    # -------------------------------------------------------------------------
    # [노이즈 도구] 값에 랜덤한 잡음을 섞어줍니다.
    # Noise Helper: Adds Gaussian noise to a value.
    # -------------------------------------------------------------------------
    def add_noise(self, value, noise_level):
        # 평균이 0이고 표준편차가 noise_level인 가우시안 분포 랜덤값을 더합니다.
        # Returns value + random noise from Gaussian distribution.
        return value + random.gauss(0, noise_level)

    # -------------------------------------------------------------------------
    # [움직임 도구] 현재값을 목표값 쪽으로 부드럽게 이동시킵니다 (선형 보간).
    # Motion Helper: Smoothly moves Current towards Target (LERP).
    # -------------------------------------------------------------------------
    def smooth_move(self, current, target):
        # 목표까지 남은 거리(차이)를 계산합니다.
        # Calculate the difference.
        diff = target - current
        
        # 차이가 아주 작으면 그냥 목표값에 도착한 것으로 칩니다 (떨림 방지).
        # Stop if close enough to prevent jitter.
        if abs(diff) < 0.001: return target
        
        # 현재 위치에 (남은 거리 * 15%) 만큼을 더해줍니다. 매번 15%씩 다가가므로 부드럽게 감속하며 도착합니다.
        # Move 15% of the way towards the target.
        return current + diff * SMOOTH_FACTOR

    # -------------------------------------------------------------------------
    # [직원 B] 메인 루프 (0.02초마다 실행) - 물리 계산 및 데이터 전송
    # Worker B: Main loop running at 50Hz for physics and publishing.
    # -------------------------------------------------------------------------
    def timer_callback(self):
        # 역시 변수를 읽고 써야 하므로 락을 겁니다.
        # Acquire lock for thread safety.
        with self.lock:
            # 1. 팔 관절: 부드러운 위치 추적 계산 (Smoothing)
            # Arm: Calculate smooth interpolation.
            self.curr_shoulder_yaw = self.smooth_move(self.curr_shoulder_yaw, self.target_shoulder_yaw)
            self.curr_elbow_pitch = self.smooth_move(self.curr_elbow_pitch, self.target_elbow_pitch)
            self.curr_forearm_roll = self.smooth_move(self.curr_forearm_roll, self.target_forearm_roll)
            self.curr_wrist_pitch = self.smooth_move(self.curr_wrist_pitch, self.target_wrist_pitch)

            # 2. 손가락: 속도 기반 위치 계산 (Integration, 적분)
            # 공식: 현재위치 = 이전위치 + (속도 * 시간). 여기선 시간 간격이 일정하므로 그냥 더합니다.
            # Finger: Integrate velocity to get position.
            self.curr_flex_level += self.flex_velocity
            
            # 3. 손가락 한계 제한 (Clamping)
            # 1.0보다 작아지거나 10.0보다 커지지 않게 막습니다.
            # Clamp flex level between 1.0 and 10.0.
            if self.curr_flex_level > 10.0:
                self.curr_flex_level = 10.0
                self.flex_velocity = 0.0 # 벽에 닿았으니 속도도 0으로 만듦
            elif self.curr_flex_level < 1.0:
                self.curr_flex_level = 1.0
                self.flex_velocity = 0.0

        # 4. JointState 메시지 포장 및 발송 (Rviz용 정답 데이터)
        # Pack and publish JointState message for visualization.
        joint_msg = JointState()
        joint_msg.header.stamp = self.get_clock().now().to_msg()
        # URDF에 정의된 관절 이름과 똑같이 적어야 합니다.
        joint_msg.name = ['r_shoulder_yaw', 'r_elbow_pitch', 'r_forearm_roll', 'r_wrist_pitch', 'r_index_knuckle', 'r_index_mid', 'r_index_tip']
        
        # 1~10 레벨을 라디안 각도로 변환합니다.
        finger_rad = (self.curr_flex_level - 1) * (1.57 / 9.0)
        
        # 계산된 모든 각도를 리스트에 담습니다.
        joint_msg.position = [self.curr_shoulder_yaw, self.curr_elbow_pitch, self.curr_forearm_roll, self.curr_wrist_pitch, finger_rad, finger_rad, finger_rad]
        self.joint_pub.publish(joint_msg)

        # -------------------------------------------------------------
        # 5. 가상 센서 데이터 생성 (노이즈 추가)
        # Generate simulated sensor data with noise.
        # -------------------------------------------------------------
        
        # --- [A] 전완 (Forearm) IMU 생성 ---
        # 실제값에 노이즈를 섞어서 '더러운' 센서값을 만듭니다.
        # Create noisy Euler angles.
        f_roll = self.add_noise(self.curr_forearm_roll, NOISE_IMU_ORI)
        f_pitch = self.add_noise(self.curr_elbow_pitch, NOISE_IMU_ORI)
        f_yaw = self.add_noise(self.curr_shoulder_yaw, NOISE_IMU_ORI)

        # 오일러 데이터 발행 (센서 프로세서에서 받을 예정)
        # Publish Euler angles.
        euler_forearm = Vector3()
        euler_forearm.x, euler_forearm.y, euler_forearm.z = f_roll, f_pitch, f_yaw
        self.euler_forearm_pub.publish(euler_forearm)

        # 쿼터니언 변환 후 IMU 메시지 발행 (ROS 표준)
        # Convert to Quaternion and publish IMU message.
        imu_forearm = Imu()
        imu_forearm.header.stamp = self.get_clock().now().to_msg()
        imu_forearm.header.frame_id = "imu_brachioradialis"
        q = self.euler_to_quaternion(f_roll, f_pitch, f_yaw)
        imu_forearm.orientation.x, imu_forearm.orientation.y, imu_forearm.orientation.z, imu_forearm.orientation.w = q[0], q[1], q[2], q[3]
        imu_forearm.linear_acceleration.z = self.add_noise(9.8, NOISE_IMU_ACCEL) # 중력가속도(9.8)에도 노이즈 추가
        self.imu_forearm_pub.publish(imu_forearm)

        # --- [B] 손목 (Wrist) IMU 생성 ---
        # 손목은 '전완의 회전'은 그대로 받고, '손목 꺾임'만 추가됩니다.
        # Wrist inherits Forearm roll/yaw, but adds wrist pitch.
        w_roll = f_roll
        w_pitch = self.add_noise(self.curr_elbow_pitch + self.curr_wrist_pitch, NOISE_IMU_ORI)
        w_yaw = f_yaw

        # 오일러 발행
        euler_wrist = Vector3()
        euler_wrist.x, euler_wrist.y, euler_wrist.z = w_roll, w_pitch, w_yaw
        self.euler_wrist_pub.publish(euler_wrist)

        # IMU 메시지 발행
        imu_wrist = Imu()
        imu_wrist.header.stamp = self.get_clock().now().to_msg()
        imu_wrist.header.frame_id = "imu_wrist"
        q_wrist = self.euler_to_quaternion(w_roll, w_pitch, w_yaw)
        imu_wrist.orientation.x, imu_wrist.orientation.y, imu_wrist.orientation.z, imu_wrist.orientation.w = q_wrist[0], q_wrist[1], q_wrist[2], q_wrist[3]
        imu_wrist.linear_acceleration.z = self.add_noise(9.8, NOISE_IMU_ACCEL)
        self.imu_wrist_pub.publish(imu_wrist)

        # --- [C] Flex 센서 생성 ---
        # 1~10 레벨을 전압(0~3.3V)으로 바꾸고 노이즈를 섞습니다.
        # Convert level to voltage and add noise.
        flex_msg = Float32()
        ideal_voltage = (self.curr_flex_level - 1) * (3.3 / 9.0)
        flex_msg.data = self.add_noise(ideal_voltage, NOISE_FLEX * 0.1)
        self.flex_pub.publish(flex_msg)

# -----------------------------------------------------------------------------
# [키보드 입력 도구] 엔터키 없이 키 하나를 바로 읽어오는 함수
# Helper: Reads a single keypress without waiting for Enter.
# -----------------------------------------------------------------------------
def get_key(settings):
    tty.setraw(sys.stdin.fileno())   # 터미널을 날것(Raw) 모드로 바꿉니다.
    key = sys.stdin.read(1)          # 키 하나를 읽을 때까지 기다립니다 (Blocking).
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings) # 터미널을 원래대로 돌려놓습니다.
    return key

# -----------------------------------------------------------------------------
# [쓰레드 함수] 무한루프를 돌며 키보드를 감시하는 경비원
# Thread Function: Continuously monitors keyboard input.
# -----------------------------------------------------------------------------
def input_thread(node):
    settings = termios.tcgetattr(sys.stdin) # 현재 터미널 설정을 저장해둡니다.
    try:
        while True:
            key = get_key(settings) # 키가 눌릴 때까지 여기서 대기합니다.
            if key == '\x03': # Ctrl+C가 눌리면 종료 절차를 밟습니다.
                node.destroy_node()
                rclpy.shutdown()
                break
            # 키가 눌리면 노드의 update_target 함수를 호출해 값을 바꿉니다.
            node.update_target(key)
    except Exception:
        pass
    finally:
        # 프로그램이 죽을 때 터미널이 깨지지 않도록 설정을 복구합니다.
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

# -----------------------------------------------------------------------------
# [메인 함수] 프로그램의 시작점
# Main Entry Point.
# -----------------------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)           # ROS 2 시스템을 초기화합니다.
    node = WearableSensorSim()      # 우리가 만든 노드 객체를 생성합니다.
    
    # 키보드 감시용 쓰레드를 만들고 시작시킵니다.
    # Create and start the input monitoring thread.
    t = threading.Thread(target=input_thread, args=(node,))
    t.daemon = True # 메인 프로그램이 죽으면 이 쓰레드도 같이 죽도록 설정합니다.
    t.start()
    
    try:
        rclpy.spin(node) # 메인 쓰레드는 여기서 무한루프를 돌며 ROS 통신을 처리합니다.
                         # Main thread handles ROS callbacks/timers here.
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node() # 노드를 깔끔하게 정리합니다.
        if rclpy.ok():
            rclpy.shutdown() # ROS 시스템을 종료합니다.

if __name__ == '__main__':
    main()


#구현할 로직
#노이즈가 낀 쿼터니안은 가재보 가상의 팔에 그대로 반영(근육 떨림 등/가능하면 시간이 지남에 따라 쳐짐도 추가)
#노이즈가 낀 오일러 각을 시그널 프로세서로 보내 필터링 한 값을 드론에 적용
#샘플링 레이트 불일치 문제
#센서의 주기와 드론 제어기의 주기의 불일치(드론 제어기의 주기가 느릴 경우 연속적인 움직임을 하며 대기?)
#센서의 raw 데이터 주기 계산, 필터링 주기, 드론의 제어 주기 필요
#패킷 유실
#random.random() < 0.01 이면 데이터를 발행하지 않는 로직
#오일러 발행 이후부터 추가 공부 필요