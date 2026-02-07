import rclpy
# Import the ROS 2 Python client library to handle node lifecycle and communication.
# ROS 2 노드의 생명주기 관리와 통신 처리를 위해 ROS 2 파이썬 클라이언트 라이브러리를 가져옵니다.

from rclpy.node import Node
# Import the base Node class to create a custom ROS 2 node.
# 커스텀 ROS 2 노드를 생성하기 위해 기본 Node 클래스를 가져옵니다.

from sensor_msgs.msg import JointState
# Import JointState message for visualizing the robot's joint states in Rviz.
# Rviz에서 로봇 모델의 관절 상태를 시각화하기 위해 JointState 메시지를 가져옵니다.

from std_msgs.msg import Float32MultiArray, Float64, UInt8MultiArray
# Import standard message types for sensor arrays, Gazebo commands, and binary packets.
# 센서 데이터 배열, 가제보 명령, 그리고 바이너리 패킷 전송을 위해 표준 메시지 타입을 가져옵니다.

import pygame
# Use Pygame library to handle simultaneous inputs and display the control window.
# 키보드 동시 입력을 처리하고 조종창을 띄우기 위해 Pygame 라이브러리를 사용합니다.

import sys, math, random, threading, os
# Libraries for system control, math, RNG, and multi-threading.
# 시스템 제어, 수학 연산, 난수 생성 및 멀티쓰레딩을 위한 라이브러리입니다.

import struct
# Library for packing Python values into C structs (binary data).
# 파이썬 값을 C 언어 구조체(바이너리 데이터)로 패킹하기 위한 라이브러리입니다.

# =============================================================================
# Configuration & Constants (설정 및 상수 정의)
# =============================================================================

TIMER_PERIOD = 0.01 
# Base timer period; 0.01s corresponds to 100Hz frequency.
# 기본 타이머 주기이며 0.01초는 100Hz의 통신 속도를 의미합니다.

DT_US = 10000 
# Fixed time step for monotonic counter (10ms = 10000us).
# 단조 증가 카운터를 위한 고정 시간 스텝입니다 (10ms = 10000마이크로초).

SLOW_RATE_DIVISOR = 2
# Update Magnetometer every 2 ticks (50Hz).
# 지자계 센서를 2틱마다(50Hz) 갱신하여 멀티레이트를 구현합니다.

LOSS_PROBABILITY = 0.05 
# Assumes a 5% packet loss probability to simulate real-world conditions.
# 실제 환경을 모사하기 위해 5%의 패킷 유실 확률을 가정합니다.

RADIUS_UPPER = 0.350
# Physical length of the upper arm (meters).
# 상완(어깨~팔꿈치)의 물리적 길이(미터)입니다.

RADIUS_FOREARM = 0.064 
# Distance to the forearm sensor attachment point.
# 전완 센서가 부착된 위치까지의 거리입니다.

RADIUS_WRIST = 0.240   
# Distance to the wrist sensor attachment point.
# 손목 센서가 부착된 위치까지의 거리입니다.

CONST_STEP = 0.01     
# Joint angle step size per key press.
# 키를 한 번 누를 때 변하는 관절 각도의 크기입니다.

# [Flex Sensor Settings]
FLEX_MIN = 0.0
# Minimum sensor value when fully extended.
# 센서가 완전히 펴졌을 때의 최소값입니다.

FLEX_MAX = 3.5
# Maximum sensor value allowed by user (User limit).
# 사용자가 지정한 센서의 최대 굽힘 값입니다 (사용자 제한).

FLEX_THEORETICAL_MAX = 6.0
# Theoretical maximum value for 90 degree bend.
# 90도 굽힘에 해당하는 이론적인 최대값입니다.

FLEX_USER_LIMIT = 3.5
# User defined limit for the simulation logic.
# 시뮬레이션 로직에서 사용할 사용자 정의 한계값입니다.

# [ADC Settings]
ADC_MAX = 4095.0
# Maximum value for a 12-bit ADC (ESP32).
# 12비트 ADC(ESP32)가 표현할 수 있는 최대값입니다.

# [Mouse Sensitivity]
MOUSE_FOLLOW_SPEED = 0.05
# Slider tracking speed. Lower values mean smoother, heavier movement.
# 슬라이더가 마우스를 따라오는 속도입니다. 낮을수록 묵직하고 부드럽게 움직입니다.

SLIDER_SENSITIVITY = 0.001
# Value change increment for fine adjustments.
# 미세 조정 시 값이 변하는 단위입니다.

# [PD Control Gains]
KP = 3.0
# Proportional Gain: Spring force moving towards the target.
# 비례 게인: 목표값으로 가려는 스프링의 힘을 결정합니다.

KD = 0.5
# Derivative Gain: Damper to suppress oscillations.
# 미분 게인: 진동을 억제하는 댐퍼(저항) 역할을 합니다.

# [Noise Levels]
NOISE_ACCEL = 0.02
# Noise intensity for the accelerometer.
# 가속도 센서에 추가될 잡음의 세기입니다.

NOISE_GYRO = 0.005
# Noise intensity for the gyroscope.
# 자이로 센서(각속도)에 추가될 잡음의 세기입니다.

NOISE_MAG = 0.000002
# Noise intensity for the magnetometer.
# 지자계 센서에 추가될 잡음의 세기입니다.

NOISE_FLEX_TREMOR = 0.05
# Intensity of high-frequency tremor/electrical noise.
# 수전증이나 전기적 잡음 같은 고주파 노이즈의 강도입니다.

NOISE_FLEX_DRIFT = 0.02
# Intensity of low-frequency drift due to heat or fatigue.
# 열이나 피로도로 인해 값이 서서히 변하는 드리프트 노이즈의 강도입니다.

DRIFT_STEP = 0.0002
# Rate at which gyro bias accumulates over time.
# 자이로 센서의 바이어스(오차)가 시간이 지남에 따라 누적되는 속도입니다.

FLEX_G_SENSITIVITY = 0.05
# Sensitivity of flex sensor to centrifugal forces during maneuvers.
# 드론의 급격한 기동(원심력)이 Flex 센서 값에 영향을 주는 민감도입니다.

class WearableSensorSim(Node):
    """
    ESP32 임베디드 장치와 조종기를 동시에 시뮬레이션하는 ROS 2 노드입니다.
    ROS 2 node simulating both an ESP32 embedded device and a controller.
    """
    def __init__(self):
        super().__init__('wearable_sensor_sim')
        # Initialize the parent Node class to create the 'wearable_sensor_sim' node.
        # 부모 클래스인 Node를 초기화하여 'wearable_sensor_sim' 노드를 생성합니다.
        
        os.environ['SDL_VIDEODRIVER'] = 'x11' 
        # Force the SDL video driver to x11 for displaying Pygame windows in Docker.
        # 도커 환경에서 Pygame 창을 띄우기 위해 그래픽 드라이버를 x11로 강제 설정합니다.

        try:
            pygame.init()
            # Initialize the Pygame system.
            # Pygame 시스템을 초기화합니다.
            self.screen = pygame.display.set_mode((400, 200))
            # Create a small 400x200 window to capture keyboard inputs.
            # 키 입력을 감지하기 위한 400x200 크기의 작은 윈도우 창을 생성합니다.
            pygame.display.set_caption("HRI Controller (Refined Flags & Physics)")
            # Set the title of the created window.
            # 생성된 창의 제목을 설정합니다.
        except pygame.error as e:
            self.get_logger().error(f"Pygame init failed: {e}")
            # Log error if initialization fails.
            # 초기화 실패 시 에러 로그를 남기고 종료합니다.
            sys.exit(1)

        # 1. Publishers Setup (데이터 전송 통로 설정)
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)
        # Channel to send the current joint angles to Rviz.
        # Rviz에 로봇의 현재 관절 각도를 보내는 통로입니다.

        self.imu_wrist_pub = self.create_publisher(Float32MultiArray, 'IMU_Wrist_Data', 10)
        # Channel for transmitting wrist IMU sensor data.
        # 손목 IMU 센서 데이터를 보내는 통로입니다.

        self.imu_arm_pub = self.create_publisher(Float32MultiArray, 'IMU_ARM_Data', 10)
        # Channel for transmitting arm IMU sensor data.
        # 상완 IMU 센서 데이터를 보내는 통로입니다.

        self.mag_pub = self.create_publisher(Float32MultiArray, 'Magnet_Data', 10)
        # Channel for transmitting magnetometer data.
        # 지자계(나침반) 데이터를 보내는 통로입니다.

        self.flex_pub = self.create_publisher(Float32MultiArray, 'Flex_Data', 10)
        # Channel for transmitting flex sensor data.
        # 손가락 굽힘 센서 데이터를 보내는 통로입니다.

        self.elrs_pub = self.create_publisher(UInt8MultiArray, 'elrs_packet', 10)
        # Channel for transmitting binary ELRS packets (Byte array).
        # 바이너리 ELRS 패킷을 전송하기 위한 통로입니다 (바이트 배열).

        # UI Variables
        self.slider_rect = pygame.Rect(50, 100, 300, 20)
        # Define the position and size of the slider bar.
        # 슬라이더 바의 위치와 크기를 정의합니다.
        self.slider_handle_x = 50
        # Initial position of the slider handle.
        # 슬라이더 핸들의 초기 위치입니다.
        self.target_handle_x = 50
        # Target position for the slider handle (mouse position).
        # 슬라이더 핸들이 이동할 목표 위치입니다 (마우스 위치).
        self.dragging = False
        # Flag to indicate if the slider is being dragged.
        # 슬라이더가 드래그 중인지 나타내는 플래그입니다.

        # Gazebo Publishers
        self.pub_cmd_shoulder = self.create_publisher(Float64, '/model/wearable_hand/joint/r_shoulder_yaw/cmd_pos', 10)
        self.pub_cmd_elbow = self.create_publisher(Float64, '/model/wearable_hand/joint/r_elbow_pitch/cmd_pos', 10)
        self.pub_cmd_forearm = self.create_publisher(Float64, '/model/wearable_hand/joint/r_forearm_roll/cmd_pos', 10)
        self.pub_cmd_wrist = self.create_publisher(Float64, '/model/wearable_hand/joint/r_wrist_pitch/cmd_pos', 10)
        self.pub_cmd_index1 = self.create_publisher(Float64, '/model/wearable_hand/joint/r_index_knuckle/cmd_pos', 10)
        self.pub_cmd_index2 = self.create_publisher(Float64, '/model/wearable_hand/joint/r_index_mid/cmd_pos', 10)
        self.pub_cmd_index3 = self.create_publisher(Float64, '/model/wearable_hand/joint/r_index_tip/cmd_pos', 10)

        # 2. Timer & Lock Setup
        self.timer = self.create_timer(TIMER_PERIOD, self.timer_callback)
        # Triggers the main calculation (timer_callback) every 0.01 seconds.
        # 0.01초마다 메인 연산(timer_callback)을 실행합니다.

        self.lock = threading.Lock()
        # Lock mechanism to prevent data corruption from concurrent access.
        # 여러 작업이 동시에 데이터를 수정하여 발생하는 오류를 방지하는 잠금 장치입니다.
        self.tick_count = 0 
        self.seq_counter = 0 
        # Packet sequence counter (0 to 65535).
        # 패킷 시퀀스 카운터입니다 (0부터 65535까지).
        self.sim_time_us = 0 
        # Simulated monotonic clock (microseconds).
        # 시뮬레이션된 단조 증가 시계입니다 (마이크로초 단위).

        # State Variables
        self.target_shoulder_yaw = 0.0; self.curr_shoulder_yaw = 0.0
        self.target_elbow_pitch = 0.0;  self.curr_elbow_pitch = 0.0
        self.target_forearm_roll = 0.0; self.curr_forearm_roll = 0.0
        self.target_wrist_pitch = 0.0;  self.curr_wrist_pitch = 0.0
        
        self.raw_flex_level = FLEX_MIN
        self.noisy_flex_level = FLEX_MIN

        # Separate previous gyro states for wrist and arm.
        # 손목과 팔의 이전 자이로 상태를 분리하여 저장합니다.
        self.prev_shoulder_yaw = 0.0
        self.prev_elbow_pitch = 0.0
        self.prev_forearm_roll = 0.0
        self.prev_wrist_pitch = 0.0
        
        self.prev_wy_wrist = 0.0 # Dedicated previous wy for wrist physics.
        # 손목 물리 계산 전용 이전 각속도 값입니다.
        self.prev_wy_arm = 0.0   # Dedicated previous wy for arm physics.
        # 상완 물리 계산 전용 이전 각속도 값입니다.
        
        self.gyro_bias_x = 0.0; self.gyro_bias_y = 0.0; self.gyro_bias_z = 0.0

        # Persistent Mag data for multi-rate updates.
        # 멀티레이트(저속) 갱신을 위해 지자계 데이터를 유지합니다.
        self.cached_mag = (0.0, 0.0, 0.0)

        print(">>> Mode: Final Physics, Flags & G-Sensitivity <<<")

    def smooth_move(self, curr, target):
        # Interpolation filter for smoothing movement.
        # 움직임을 부드럽게 만드는 보간 필터입니다.
        diff = target - curr
        if abs(diff) < 0.0005: return target
        return curr + diff * 0.15 
    
    def add_noise(self, val, level):
        # Adds random Gaussian noise to sensor data.
        # 센서 데이터에 무작위 가우시안 노이즈(잡음)를 추가합니다.
        return val + random.gauss(0, level)

    def check_and_clamp_int16(self, val):
        # Checks if value exceeds int16 range and clamps it. Returns (clamped_val, is_clipped).
        # 값이 int16 범위를 초과하는지 확인하고 제한합니다. (제한된 값, 클리핑 여부)를 반환합니다.
        is_clipped = False
        if val > 32767 or val < -32768:
            is_clipped = True
        clamped_val = max(-32768, min(32767, int(val)))
        return clamped_val, is_clipped

    def compute_physics(self, roll, pitch, yaw, dr, dp, dy, rad_local, d_shl, elbow_angle, prev_wy):
        # Physics engine calculating acceleration and rotation based on robot motion.
        # 로봇의 움직임에 따른 가속도와 회전 속도를 계산하는 물리 엔진입니다.
        
        gx = 9.81 * math.sin(pitch)
        gy = -9.81 * math.sin(roll) * math.cos(pitch)
        gz = 9.81 * math.cos(roll) * math.cos(pitch)
        # Calculate gravity components based on orientation.
        # 자세에 따른 중력 가속도 성분을 계산합니다.

        wx = dr / TIMER_PERIOD; wy = dp / TIMER_PERIOD; wz = dy / TIMER_PERIOD
        # Calculate angular velocities (rad/s).
        # 각속도를 계산합니다 (라디안/초).
        
        # [FIX] Do NOT update gyro_bias here to avoid double accumulation.
        # [수정] 이중 누적을 방지하기 위해 여기서 자이로 바이어스를 업데이트하지 않습니다.
        
        r_eff = math.sqrt(RADIUS_UPPER**2 + rad_local**2 + 2*RADIUS_UPPER*rad_local*math.cos(elbow_angle))
        # Calculate effective radius considering the arm angle.
        # 팔의 각도를 고려한 유효 회전 반경을 계산합니다.
        
        # Use specific previous wy for angular acceleration calculation.
        # 각가속도 계산에 해당 관절에 맞는 이전 wy 값을 사용합니다.
        alpha_local = (wy - prev_wy) / TIMER_PERIOD
        w_shl = d_shl / TIMER_PERIOD
        
        ax = gx + (alpha_local * rad_local) + (w_shl**2 * r_eff)
        ay = gy 
        az = gz + (wy**2 * rad_local)
        # Compute final linear acceleration values.
        # 최종적인 선형 가속도 값을 산출합니다.

        # Return the NEW wy so caller can update its specific state.
        # 호출자가 각자의 상태를 업데이트할 수 있도록 새로운 wy를 반환합니다.
        return (ax, ay, az), (wx + self.gyro_bias_x, wy + self.gyro_bias_y, wz + self.gyro_bias_z), wy

    def compute_mag(self, r, p, y):
        # Generates magnetometer (compass) sensor values based on robot orientation.
        # 로봇의 방향에 따른 지자계(나침반) 센서 값을 생성합니다.
        mx = math.cos(y)*math.cos(p)
        my = math.sin(y)*math.cos(p)
        mz = math.sin(p)
        return (mx * 45e-6, my * 45e-6, mz * 45e-6) # Units: Tesla

    def publish_array(self, publisher, data_list):
        # Helper function to publish float array data.
        # 실수형 배열 데이터를 발행하는 헬퍼 함수입니다.
        msg = Float32MultiArray()
        msg.data = [float(x) for x in data_list]
        publisher.publish(msg)

    def timer_callback(self):
        # Update monotonic time regardless of packet loss.
        # 패킷 유실 여부와 상관없이 단조 증가 시간을 업데이트합니다.
        self.sim_time_us += DT_US 

        # 1. Pygame Event Handling
        # Pygame 이벤트 처리 로직입니다.
        for event in pygame.event.get():
            if event.type == pygame.QUIT: sys.exit()
            
            if event.type in [pygame.MOUSEBUTTONDOWN, pygame.MOUSEMOTION]:
                if pygame.mouse.get_pressed()[0]: 
                    mouse_x, mouse_y = pygame.mouse.get_pos()
                    if 50 <= mouse_x <= 350 and 80 <= mouse_y <= 130:
                        self.target_handle_x = mouse_x 
                        self.dragging = True
                    else: self.dragging = False

        # Slider Movement Physics (Smoothing)
        # 슬라이더 움직임 물리 처리 (부드럽게 이동).
        diff = self.target_handle_x - self.slider_handle_x
        self.slider_handle_x += diff * MOUSE_FOLLOW_SPEED
        
        # Map slider position to Flex value range (0 ~ 3.5).
        # 슬라이더 위치를 Flex 값 범위(0 ~ 3.5)로 매핑합니다.
        ratio = (self.slider_handle_x - 50) / 300.0
        self.raw_flex_level = max(FLEX_MIN, min(FLEX_USER_LIMIT, FLEX_MIN + ratio * (FLEX_USER_LIMIT - FLEX_MIN)))

        keys = pygame.key.get_pressed()
        # Capture current keyboard state.
        # 현재 키보드 입력 상태를 캡처합니다.

        with self.lock:
            # 2. Control Logic & Noise
            # 제어 로직 및 노이즈 생성 부분입니다.
            if keys[pygame.K_d]: self.target_shoulder_yaw = min(self.target_shoulder_yaw + CONST_STEP, 1.57)
            if keys[pygame.K_a]: self.target_shoulder_yaw = max(self.target_shoulder_yaw - CONST_STEP, -1.57)
            if keys[pygame.K_w]: self.target_elbow_pitch = min(self.target_elbow_pitch + CONST_STEP, 2.0)
            if keys[pygame.K_s]: self.target_elbow_pitch = max(self.target_elbow_pitch - CONST_STEP, 0.0)
            if keys[pygame.K_q]: self.target_forearm_roll = max(self.target_forearm_roll - CONST_STEP, -3.14)
            if keys[pygame.K_e]: self.target_forearm_roll = min(self.target_forearm_roll + CONST_STEP, 3.14)

            # Generate realistic sensor noise (Tremor & Drift).
            # 현실적인 센서 노이즈(떨림 및 드리프트)를 생성합니다.
            tremor = random.gauss(0, NOISE_FLEX_TREMOR)
            fatigue_drift = math.sin(self.tick_count * 0.05) * NOISE_FLEX_DRIFT
            
            # Apply noise to flex sensor.
            # Flex 센서 값에 노이즈를 적용합니다.
            self.noisy_flex_level = max(FLEX_MIN, min(FLEX_USER_LIMIT, self.raw_flex_level + tremor + fatigue_drift))

            # Apply smooth movement to joints.
            # 관절에 부드러운 움직임을 적용합니다.
            self.curr_shoulder_yaw = self.smooth_move(self.curr_shoulder_yaw, self.target_shoulder_yaw)
            self.curr_elbow_pitch = self.smooth_move(self.curr_elbow_pitch, self.target_elbow_pitch)
            self.curr_forearm_roll = self.smooth_move(self.curr_forearm_roll, self.target_forearm_roll)
            self.curr_wrist_pitch = self.smooth_move(self.curr_wrist_pitch, self.target_wrist_pitch)

            # UI Drawing
            # UI 화면 그리기.
            self.screen.fill((30, 30, 30)) 
            pygame.draw.rect(self.screen, (100, 100, 100), self.slider_rect)
            
            # Visualize noisy value with a red line.
            # 노이즈가 섞인 값을 빨간색 선으로 시각화합니다.
            noisy_x = 50 + ((self.noisy_flex_level - FLEX_MIN) / (FLEX_USER_LIMIT - FLEX_MIN)) * 300
            pygame.draw.line(self.screen, (255, 100, 100), (int(noisy_x), 90), (int(noisy_x), 130), 2)
            
            handle_color = (0, 120, 255) if self.dragging else (100, 150, 255)
            pygame.draw.circle(self.screen, handle_color, (int(self.slider_handle_x), 110), 10)
            
            # Calculate ADC value for display (0~4095).
            # 표시를 위해 ADC 값을 계산합니다 (0~4095).
            adc_value = (self.noisy_flex_level - FLEX_MIN) * (ADC_MAX / (FLEX_THEORETICAL_MAX - FLEX_MIN))
            
            font = pygame.font.SysFont(None, 24)
            info_text = f"Flex: {self.noisy_flex_level:.2f} | ADC: {int(adc_value)}"
            img = font.render(info_text, True, (255, 255, 255))
            self.screen.blit(img, (50, 70))
            pygame.display.flip()

            # Gazebo Pub
            # 가제보 명령 발행.
            self.pub_cmd_shoulder.publish(Float64(data=float(self.curr_shoulder_yaw)))
            self.pub_cmd_elbow.publish(Float64(data=float(self.curr_elbow_pitch)))
            self.pub_cmd_forearm.publish(Float64(data=float(self.curr_forearm_roll)))
            self.pub_cmd_wrist.publish(Float64(data=float(self.curr_wrist_pitch)))
            
            # Calculate finger angle based on flex level.
            # Flex 레벨에 따라 손가락 각도를 계산합니다.
            finger_angle = (self.noisy_flex_level - FLEX_MIN) * (1.57 / (FLEX_THEORETICAL_MAX - FLEX_MIN))
            self.pub_cmd_index1.publish(Float64(data=float(finger_angle)))
            self.pub_cmd_index2.publish(Float64(data=float(finger_angle)))
            self.pub_cmd_index3.publish(Float64(data=float(finger_angle)))

        # Rviz Pub
        # Rviz 시각화 데이터 발행.
        j_msg = JointState()
        j_msg.header.stamp = self.get_clock().now().to_msg()
        j_msg.name = ['r_shoulder_yaw', 'r_elbow_pitch', 'r_forearm_roll', 'r_wrist_pitch', 'r_index_knuckle', 'r_index_mid', 'r_index_tip']
        j_msg.position = [self.curr_shoulder_yaw, self.curr_elbow_pitch, self.curr_forearm_roll, self.curr_wrist_pitch, finger_angle, finger_angle, finger_angle]
        self.joint_pub.publish(j_msg)
        
        # -----------------------------------------------------------
        # [ELRS Packet Generation Generation]
        # [ELRS 패킷 생성 로직]
        # -----------------------------------------------------------
        
        if random.random() < LOSS_PROBABILITY: 
            # If packet is lost, skip transmission but tick still increments.
            # 패킷이 유실되면 전송을 건너뛰지만, 틱은 계속 증가합니다.
            self.tick_count += 1
            return 

        # Calculate Deltas (Movement changes)
        # 변화량(델타)을 계산합니다.
        d_s = self.curr_shoulder_yaw - self.prev_shoulder_yaw
        d_e = self.curr_elbow_pitch - self.prev_elbow_pitch
        d_f = self.curr_forearm_roll - self.prev_forearm_roll

        # [FIX] Update gyro bias only ONCE per tick (Avoids double accumulation).
        # [수정] 자이로 바이어스를 틱당 한 번만 업데이트합니다 (이중 누적 방지).
        self.gyro_bias_x += random.gauss(0, DRIFT_STEP)
        self.gyro_bias_y += random.gauss(0, DRIFT_STEP)
        self.gyro_bias_z += random.gauss(0, DRIFT_STEP)

        # 1. Wrist Physics Calculation (Unpack 3 values correctly)
        # 1. 손목 물리 계산 (3개의 반환값을 정확히 언패킹합니다)
        aw, gw, raw_wy_wrist = self.compute_physics(
            self.curr_forearm_roll, self.curr_elbow_pitch, self.curr_shoulder_yaw, 
            d_f, d_e, d_s, RADIUS_WRIST, d_s, self.curr_elbow_pitch, 
            self.prev_wy_wrist 
        )
        self.prev_wy_wrist = raw_wy_wrist # Update wrist state. (손목 상태 갱신)

        # 2. Arm Physics Calculation (For Debug/Arm IMU)
        # 2. 상완 물리 계산 (디버깅/상완 IMU용)
        aa, ga, raw_wy_arm = self.compute_physics(
            self.curr_forearm_roll, self.curr_elbow_pitch, self.curr_shoulder_yaw, 
            d_f, d_e, d_s, RADIUS_FOREARM, d_s, self.curr_elbow_pitch, 
            self.prev_wy_arm 
        )
        self.prev_wy_arm = raw_wy_arm # Update arm state. (상완 상태 갱신)
        
        # [FIX] Multi-rate Magnetometer & Improved Flags
        # [수정] 멀티레이트 지자계 및 개선된 플래그 로직.
        flags = 0
        
        # Bit 0: IMU_VALID (Normally 1 in sim unless crash simulated)
        # 비트 0: IMU_VALID (시뮬레이션에서 크래시가 없으면 보통 1)
        flags |= (1 << 0)

        # Bit 2: FLEX_VALID (Simulating connected sensor)
        # 비트 2: FLEX_VALID (센서 연결 시뮬레이션)
        flags |= (1 << 2)

        if self.tick_count % SLOW_RATE_DIVISOR == 0:
            # Update Magnetometer every N ticks.
            # N 틱마다 지자계 센서를 갱신합니다.
            self.cached_mag = self.compute_mag(self.curr_forearm_roll, self.curr_elbow_pitch, self.curr_shoulder_yaw)
            # Bit 1: MAG_UPDATED
            # 비트 1: MAG_UPDATED (지자계 갱신됨)
            flags |= (1 << 1) 
        
        # Use cached magnetometer value (Hold value if not updated).
        # 캐시된 지자계 값을 사용합니다 (갱신되지 않았으면 값 유지).
        m_val = self.cached_mag

        # Add Noise to Wrist Sensor Data
        # 손목 센서 데이터에 노이즈를 추가합니다.
        ax_raw, ay_raw, az_raw = [self.add_noise(v, NOISE_ACCEL) for v in aw]
        gx_raw, gy_raw, gz_raw = [self.add_noise(v, NOISE_GYRO) for v in gw]
        mx_raw, my_raw, mz_raw = [self.add_noise(v, NOISE_MAG) for v in m_val]
        
        # [FIX] Apply Flex G-Sensitivity (Distortion by Wrist Acceleration).
        # [수정] Flex G-Sensitivity 적용 (손목 가속도에 의한 왜곡).
        # Calculate acceleration magnitude.
        # 가속도 크기를 계산합니다.
        acc_mag = math.sqrt(aw[0]**2 + aw[1]**2 + aw[2]**2)
        # Add dynamic error based on movement intensity.
        # 움직임 강도에 따른 동적 오차를 추가합니다.
        flex_raw = adc_value + (acc_mag * FLEX_G_SENSITIVITY)

        # Add Noise to Arm Sensor Data (For Debug Publish)
        # 상완 센서 데이터에 노이즈를 추가합니다 (디버그 발행용).
        aa_raw, ay_arm_raw, az_arm_raw = [self.add_noise(v, NOISE_ACCEL) for v in aa]
        ga_raw, gy_arm_raw, gz_arm_raw = [self.add_noise(v, NOISE_GYRO) for v in ga]

        # Check and set CLIPPED flag (Bit 3)
        # 클리핑 여부를 확인하고 플래그(비트 3)를 설정합니다.
        clip_detected = False

        # Integer Scaling & Clamping for Binary Packing
        # 바이너리 패킹을 위한 정수 스케일링 및 클램핑을 수행합니다.
        
        # Accel Scaling check
        acc_x, c1 = self.check_and_clamp_int16(ax_raw * 1000)
        acc_y, c2 = self.check_and_clamp_int16(ay_raw * 1000)
        acc_z, c3 = self.check_and_clamp_int16(az_raw * 1000)
        if c1 or c2 or c3: clip_detected = True

        # Gyro Scaling check
        gyro_x, c4 = self.check_and_clamp_int16(gx_raw * 1000)
        gyro_y, c5 = self.check_and_clamp_int16(gy_raw * 1000)
        gyro_z, c6 = self.check_and_clamp_int16(gz_raw * 1000)
        if c4 or c5 or c6: clip_detected = True

        # Mag Scaling check
        mag_x, c7 = self.check_and_clamp_int16(mx_raw * 1e7)
        mag_y, c8 = self.check_and_clamp_int16(my_raw * 1e7)
        mag_z, c9 = self.check_and_clamp_int16(mz_raw * 1e7)
        if c7 or c8 or c9: clip_detected = True

        # Set Bit 3 (CLIPPED) if any value was out of range.
        # 어떤 값이라도 범위를 벗어났다면 비트 3 (CLIPPED)을 설정합니다.
        if clip_detected:
            flags |= (1 << 3)

        # Flex & Time
        flex_adc = int(max(0, min(4095, flex_raw)))
        t_us = self.sim_time_us & 0xFFFFFFFF
        seq = self.seq_counter
        self.seq_counter = (self.seq_counter + 1) % 65536

        # Packing data into C-struct format (27 bytes)
        # 데이터를 C 구조체 포맷으로 패킹합니다 (27바이트).
        packet_data = struct.pack('<hhhhhhhhhHIHB',
                                  acc_x, acc_y, acc_z,
                                  gyro_x, gyro_y, gyro_z,
                                  mag_x, mag_y, mag_z,
                                  flex_adc, t_us, seq, flags)

        # Publish Binary Packet
        # 바이너리 패킷을 발행합니다.
        byte_msg = UInt8MultiArray()
        byte_msg.data = list(packet_data)
        self.elrs_pub.publish(byte_msg)

        # Debug Publish (Float arrays) - Wrist Data
        # 디버그용 실수형 배열 발행 - 손목 데이터
        now = self.get_clock().now()
        timestamp_s = now.nanoseconds / 1e9
        self.publish_array(self.imu_wrist_pub, [ax_raw, ay_raw, az_raw, gx_raw, gy_raw, gz_raw, timestamp_s])
        
        # Debug Publish (Float arrays) - Arm Data (Now separate)
        # 디버그용 실수형 배열 발행 - 상완 데이터 (이제 분리됨)
        self.publish_array(self.imu_arm_pub, [aa_raw, ay_arm_raw, az_arm_raw, ga_raw, gy_arm_raw, gz_arm_raw, timestamp_s])
        
        self.publish_array(self.mag_pub, [mx_raw, my_raw, mz_raw, timestamp_s])
        self.publish_array(self.flex_pub, [flex_raw, timestamp_s])

        # Update previous angles for the next loop.
        # 다음 루프를 위해 현재 각도를 이전 각도 변수에 저장합니다.
        self.prev_shoulder_yaw = self.curr_shoulder_yaw
        self.prev_elbow_pitch = self.curr_elbow_pitch
        self.prev_forearm_roll = self.curr_forearm_roll
        self.prev_wrist_pitch = self.curr_wrist_pitch

        self.tick_count += 1
        # Increment tick counter.
        # 틱 카운터를 증가시킵니다.

def main(args=None):
    rclpy.init(args=args)
    # Initialize the ROS 2 system.
    # ROS 2 시스템을 초기화합니다.
    node = WearableSensorSim()
    # Create the simulation node object.
    # 시뮬레이션 노드 객체를 생성합니다.
    try: 
        rclpy.spin(node)
        # Keeps the node running and handles communication.
        # 노드를 계속 실행하며 통신을 처리합니다.
    except KeyboardInterrupt: pass
    finally:
        pygame.quit()
        # Release Pygame resources on exit.
        # 종료 시 Pygame 리소스를 해제합니다.
        node.destroy_node()
        # Destroy the node.
        # 노드를 파괴합니다.
        if rclpy.ok(): rclpy.shutdown()
        # Shutdown ROS 2 if active.
        # ROS 2가 활성화되어 있다면 종료합니다.

if __name__ == '__main__':
    main()