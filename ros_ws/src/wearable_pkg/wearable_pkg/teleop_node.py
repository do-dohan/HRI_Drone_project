import rclpy
# ROS 2 노드의 생명주기 관리와 통신 처리를 위해 ROS 2 파이썬 클라이언트 라이브러리를 가져옵니다.
# Import the ROS 2 Python client library to handle node lifecycle and communication.

from rclpy.node import Node
# 커스텀 ROS 2 노드를 생성하기 위해 기본 Node 클래스를 가져옵니다.
# Import the base Node class to create a custom ROS 2 node.

from sensor_msgs.msg import JointState
# Rviz에서 로봇 모델의 관절 상태를 시각화하기 위해 JointState 메시지를 가져옵니다.
# Import JointState message for visualizing the robot's joint states in Rviz.

from std_msgs.msg import Float32MultiArray, Float64
# 센서 데이터 배열과 가제보 명령을 전송하기 위해 표준 메시지 타입을 가져옵니다.
# Import standard message types for sensor arrays and Gazebo commands.

import pygame
# 키보드 동시 입력을 처리하고 조종창을 띄우기 위해 Pygame 라이브러리를 사용합니다.
# Use Pygame library to handle simultaneous inputs and display the control window.

import sys, math, random, threading, os
# 시스템 제어, 수학 연산, 난수 생성 및 멀티쓰레딩을 위한 라이브러리입니다.
# Libraries for system control, math, RNG, and multi-threading.

# =============================================================================
# Configuration & Constants (설정 및 상수 정의)
# =============================================================================

TIMER_PERIOD = 0.01 
# 기본 타이머 주기이며 0.01초는 100Hz의 통신 속도를 의미합니다.
# Base timer period; 0.01s corresponds to 100Hz frequency.

SLOW_RATE_DIVISOR = 2
# 느린 센서를 위해 100Hz를 2로 나누어 50Hz로 동작하게 만듭니다.
# Divisor for slower sensors to operate at 50Hz (100Hz / 2).

LOSS_PROBABILITY = 0.05 
# 실제 무선 통신처럼 데이터 패킷이 5% 확률로 유실되는 상황을 가정합니다.
# Assumes a 5% packet loss probability, simulating real-world wireless instability.

RADIUS_UPPER = 0.350
# 로봇 모델(URDF)에 정의된 상완(어깨~팔꿈치)의 물리적 길이입니다.
# Physical length of the upper arm as defined in the URDF.

RADIUS_FOREARM = 0.064 
# 팔꿈치 관절에서 전완 센서가 부착된 위치까지의 거리입니다.
# Distance from the elbow joint to the forearm sensor attachment point.

RADIUS_WRIST = 0.240   
# 팔꿈치 관절에서 손목 센서가 부착된 위치까지의 거리입니다.
# Distance from the elbow joint to the wrist sensor attachment point.

CONST_STEP = 0.01     
# 키보드를 누르고 있을 때 관절 각도가 한 번에 변하는 크기입니다.
# The amount of joint angle change per step while a key is pressed.

TARGET_STEP = 0.05     
# 손가락 관절이 목표 지점까지 움직이는 속도를 결정합니다.
# Determines the speed at which finger joints move toward their target.

KP, KD = 2.5, 0.4
# 손가락의 부드러운 움직임을 위한 PD 제어기의 비례 및 미분 이득값입니다.
# Proportional and Derivative gains for smooth finger movement via PD control.

NOISE_ACCEL, NOISE_GYRO, NOISE_MAG, NOISE_FLEX = 0.05, 0.01, 0.000005, 0.1
# 각 센서 데이터에 섞일 실제와 같은 무작위 잡음(노이즈)의 세기입니다.
# Intensity of realistic random noise added to each sensor data stream.

DRIFT_STEP = 0.0002
# 시간이 지남에 따라 자이로 센서 값이 조금씩 틀어지는 현상을 시뮬레이션합니다.
# Simulates the phenomenon where gyro sensor values drift over time.

FLEX_G_SENSITIVITY = 0.05
# 드론이 빠르게 움직일 때 원심력에 의해 손가락 센서 값이 변하는 정도입니다.
# Sensitivity factor for flex sensor distortion caused by centrifugal force.

class WearableSensorSim(Node):
    """
    ESP32 임베디드 장치와 조종기를 동시에 시뮬레이션하는 ROS 2 노드입니다.
    ROS 2 node simulating both an ESP32 embedded device and a controller.
    """
    def __init__(self):
        super().__init__('wearable_sensor_sim')
        # 부모 클래스인 Node를 초기화하여 'wearable_sensor_sim' 노드를 생성합니다.
        # Initialize the parent Node class to create the 'wearable_sensor_sim' node.
        
        os.environ['SDL_VIDEODRIVER'] = 'x11' 
        # 도커 환경에서 Pygame 창을 띄우기 위해 그래픽 드라이버를 x11로 강제 설정합니다.
        # Force the SDL video driver to x11 for displaying Pygame windows in Docker.

        try:
            pygame.init()
            # Pygame 시스템을 초기화합니다.
            # Initialize the Pygame system.
            self.screen = pygame.display.set_mode((400, 200))
            # 키 입력을 감지하기 위한 400x200 크기의 작은 윈도우 창을 생성합니다.
            # Create a small 400x200 window to capture keyboard inputs.
            pygame.display.set_caption("HRI Drone Controller")
            # 생성된 창의 제목을 설정합니다.
            # Set the title of the created window.
        except pygame.error as e:
            self.get_logger().error(f"Pygame init failed: {e}")
            # 초기화 실패 시 에러 로그를 남기고 프로그램을 안전하게 종료합니다.
            # Log error and safely exit the program if initialization fails.
            sys.exit(1)

        # 1. Publishers Setup (데이터 전송 통로 설정)
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)
        # Rviz에 로봇의 현재 관절 각도를 보내는 통로입니다.
        # Channel to send the current joint angles to Rviz.

        self.imu_wrist_pub = self.create_publisher(Float32MultiArray, 'IMU_Wrist_Data', 10)
        # 손목 IMU 센서 데이터를 보내는 통로입니다.
        # Channel for transmitting wrist IMU sensor data.

        self.imu_arm_pub = self.create_publisher(Float32MultiArray, 'IMU_ARM_Data', 10)
        # 상완 IMU 센서 데이터를 보내는 통로입니다.
        # Channel for transmitting arm IMU sensor data.

        self.mag_pub = self.create_publisher(Float32MultiArray, 'Magnet_Data', 10)
        # 지자계(나침반) 데이터를 보내는 통로입니다.
        # Channel for transmitting magnetometer data.

        self.flex_pub = self.create_publisher(Float32MultiArray, 'Flex_Data', 10)
        # 손가락 굽힘 센서 데이터를 보내는 통로입니다.
        # Channel for transmitting flex sensor data.

        # 가제보(Gazebo)의 모터를 실제로 구동하기 위한 개별 명령 통로들입니다.
        # Individual command channels for driving motors in Gazebo.
        self.pub_cmd_shoulder = self.create_publisher(Float64, '/model/wearable_hand/joint/r_shoulder_yaw/cmd_pos', 10)
        self.pub_cmd_elbow = self.create_publisher(Float64, '/model/wearable_hand/joint/r_elbow_pitch/cmd_pos', 10)
        self.pub_cmd_forearm = self.create_publisher(Float64, '/model/wearable_hand/joint/r_forearm_roll/cmd_pos', 10)
        self.pub_cmd_wrist = self.create_publisher(Float64, '/model/wearable_hand/joint/r_wrist_pitch/cmd_pos', 10)
        self.pub_cmd_index1 = self.create_publisher(Float64, '/model/wearable_hand/joint/r_index_knuckle/cmd_pos', 10)
        self.pub_cmd_index2 = self.create_publisher(Float64, '/model/wearable_hand/joint/r_index_mid/cmd_pos', 10)
        self.pub_cmd_index3 = self.create_publisher(Float64, '/model/wearable_hand/joint/r_index_tip/cmd_pos', 10)

        # 2. Timer & Lock Setup (타이머 및 데이터 보호 설정)
        self.timer = self.create_timer(TIMER_PERIOD, self.timer_callback)
        # 0.01초마다 메인 연산(timer_callback)을 실행합니다.
        # Triggers the main calculation (timer_callback) every 0.01 seconds.

        self.lock = threading.Lock()
        # 여러 작업이 동시에 데이터를 수정하여 발생하는 오류를 방지하는 잠금 장치입니다.
        # Lock mechanism to prevent data corruption from concurrent access.
        self.tick_count = 0 

        # 3. State Variables (로봇 상태 변수)
        self.target_shoulder_yaw = 0.0; self.curr_shoulder_yaw = 0.0
        self.target_elbow_pitch = 0.0;  self.curr_elbow_pitch = 0.0
        self.target_forearm_roll = 0.0; self.curr_forearm_roll = 0.0
        self.target_wrist_pitch = 0.0;  self.curr_wrist_pitch = 0.0
        self.target_flex_level = 1.0;   self.curr_flex_level = 1.0; self.error_prev = 0.0
        # 로봇 관절들의 목표 각도와 현재 각도 정보를 저장합니다.
        # Stores target and current angle information for robot joints.

        self.prev_shoulder_yaw = 0.0; self.prev_gyro_elbow = 0.0
        self.prev_elbow_pitch = 0.0;  self.prev_forearm_roll = 0.0; self.prev_wrist_pitch = 0.0
        self.gyro_bias_x = 0.0; self.gyro_bias_y = 0.0; self.gyro_bias_z = 0.0
        # 물리 엔진 계산을 위해 이전 프레임의 데이터들을 기억합니다.
        # Remembers data from the previous frame for physics engine calculations.

        print(">>> Pygame Controller Active: Focus the window to control! <<<")

    def smooth_move(self, curr, target):
        # 현재 위치에서 목표 위치로 15%씩 부드럽게 이동시키는 필터입니다.
        # Filter that smoothly moves 15% toward the target from current position.
        diff = target - curr
        if abs(diff) < 0.001: return target
        return curr + diff * 0.25
    
    def add_noise(self, val, level):
        # 센서 데이터에 무작위 가우시안 노이즈(잡음)를 추가합니다.
        # Adds random Gaussian noise to sensor data.
        return val + random.gauss(0, level)

    def compute_physics(self, roll, pitch, yaw, dr, dp, dy, rad_local, d_shl, elbow_angle):
        # 로봇의 움직임에 따른 가속도와 회전 속도를 계산하는 물리 엔진입니다.
        # Physics engine calculating acceleration and rotation based on robot motion.
        gx = 9.81 * math.sin(pitch)
        gy = -9.81 * math.sin(roll) * math.cos(pitch)
        gz = 9.81 * math.cos(roll) * math.cos(pitch)
        # 자세에 따른 중력 가속도를 계산합니다. / Calculate gravity acceleration by orientation.

        wx = dr / TIMER_PERIOD; wy = dp / TIMER_PERIOD; wz = dy / TIMER_PERIOD
        # 각속도를 계산합니다. / Calculate angular velocities.
        
        self.gyro_bias_x += random.gauss(0, DRIFT_STEP)
        self.gyro_bias_y += random.gauss(0, DRIFT_STEP)
        self.gyro_bias_z += random.gauss(0, DRIFT_STEP)
        # 센서 오차(드리프트)를 누적시킵니다. / Accumulate sensor drift (bias).
        
        r_eff = math.sqrt(RADIUS_UPPER**2 + rad_local**2 + 2*RADIUS_UPPER*rad_local*math.cos(elbow_angle))
        alpha_local = (wy - self.prev_gyro_elbow) / TIMER_PERIOD
        w_shl = d_shl / TIMER_PERIOD
        # 복합적인 회전 반경과 가속도를 계산합니다. / Calculate complex radius and acceleration.
        
        ax = gx + (alpha_local * rad_local) + (w_shl**2 * r_eff)
        ay = gy 
        az = gz + (wy**2 * rad_local)
        # 최종적인 가속도 값을 산출합니다. / Compute final linear acceleration values.

        self.prev_gyro_elbow = wy
        return (ax, ay, az), (wx + self.gyro_bias_x, wy + self.gyro_bias_y, wz + self.gyro_bias_z)

    def compute_mag(self, r, p, y):
        # 로봇의 방향에 따른 지자계(나침반) 센서 값을 생성합니다.
        # Generates magnetometer (compass) sensor values based on robot orientation.
        mx = math.cos(y)*math.cos(p)
        my = math.sin(y)*math.cos(p)
        mz = math.sin(p)
        return (mx * 45e-6, my * 45e-6, mz * 45e-6)

    def publish_array(self, publisher, data_list):
        # 데이터 리스트를 ROS 메시지 형식으로 변환하여 발행합니다.
        # Converts data list to ROS message format and publishes.
        msg = Float32MultiArray()
        msg.data = [float(x) for x in data_list]
        publisher.publish(msg)

    def timer_callback(self):
        # 100Hz 주기로 실행되는 메인 루프입니다.
        # Main loop running at a 100Hz frequency.
        pygame.event.pump()
        # Pygame의 내부 이벤트 큐를 비워 창이 멈추지 않게 합니다.
        # Pumps Pygame events to keep the window responsive.
        
        keys = pygame.key.get_pressed()
        # 현재 키보드에서 눌린 모든 키의 상태를 한 번에 가져옵니다.
        # Captures the current state of all keyboard keys at once.

        with self.lock:
            # 1. Input Processing (동시 입력 처리)
            # 1. [매핑 변경] 요청하신 대로 키 설정을 수정했습니다.
            # A/D: 어깨 회전 (Shoulder Yaw)
            if keys[pygame.K_d]: self.target_shoulder_yaw = min(self.target_shoulder_yaw + CONST_STEP, 1.57)
            if keys[pygame.K_a]: self.target_shoulder_yaw = max(self.target_shoulder_yaw - CONST_STEP, -1.57)
            
            # W/S: 팔꿈치 피치 (Elbow Pitch)
            if keys[pygame.K_w]: self.target_elbow_pitch = min(self.target_elbow_pitch + CONST_STEP, 2.0)
            if keys[pygame.K_s]: self.target_elbow_pitch = max(self.target_elbow_pitch - CONST_STEP, 0.0)
            
            # Q/E: 상완 롤 회전 (Forearm/Upper Arm Roll)
            if keys[pygame.K_q]: self.target_forearm_roll = max(self.target_forearm_roll - CONST_STEP, -3.14)
            if keys[pygame.K_e]: self.target_forearm_roll = min(self.target_forearm_roll + CONST_STEP, 3.14)
            
            # 손가락 굽힘 (8/9)
            if keys[pygame.K_8]: self.target_flex_level = min(self.target_flex_level + TARGET_STEP, 10.0)
            if keys[pygame.K_9]: self.target_flex_level = max(self.target_flex_level - TARGET_STEP, 1.0)
            # 눌린 키에 따라 목표 관절 각도들을 실시간으로 수정합니다.
            # Updates target joint angles in real-time based on pressed keys.

            # 2. Movement Smoothing (부드러운 움직임 적용)
            self.curr_shoulder_yaw = self.smooth_move(self.curr_shoulder_yaw, self.target_shoulder_yaw)
            self.curr_elbow_pitch = self.smooth_move(self.curr_elbow_pitch, self.target_elbow_pitch)
            self.curr_forearm_roll = self.smooth_move(self.curr_forearm_roll, self.target_forearm_roll)
            self.curr_wrist_pitch = self.smooth_move(self.curr_wrist_pitch, self.target_wrist_pitch)

            # 3. Finger PD Control (손가락 제어)
            err = self.target_flex_level - self.curr_flex_level
            err_d = (err - self.error_prev) / TIMER_PERIOD
            vel = (KP * err) + (KD * err_d)
            self.curr_flex_level = max(1.0, min(10.0, self.curr_flex_level + vel * TIMER_PERIOD))
            self.error_prev = err
            # PD 제어를 통해 손가락이 떨림 없이 부드럽게 굽혀지도록 합니다.
            # Uses PD control for smooth, jitter-free finger flexion.

            # 4. State Update (물리 상태 업데이트)
            d_s = self.curr_shoulder_yaw - self.prev_shoulder_yaw
            d_e = self.curr_elbow_pitch - self.prev_elbow_pitch
            d_f = self.curr_forearm_roll - self.prev_forearm_roll
            self.prev_shoulder_yaw, self.prev_elbow_pitch = self.curr_shoulder_yaw, self.curr_elbow_pitch
            self.prev_forearm_roll, self.prev_wrist_pitch = self.curr_forearm_roll, self.curr_wrist_pitch

            # 5. Gazebo Command Publishing (가제보 명령 발송)
            self.pub_cmd_shoulder.publish(Float64(data=float(self.curr_shoulder_yaw)))
            self.pub_cmd_elbow.publish(Float64(data=float(self.curr_elbow_pitch)))
            self.pub_cmd_forearm.publish(Float64(data=float(self.curr_forearm_roll)))
            self.pub_cmd_wrist.publish(Float64(data=float(self.curr_wrist_pitch)))
            finger_angle = (self.curr_flex_level - 1.0) * (1.57 / 9.0)
            self.pub_cmd_index1.publish(Float64(data=float(finger_angle)))
            self.pub_cmd_index2.publish(Float64(data=float(finger_angle)))
            self.pub_cmd_index3.publish(Float64(data=float(finger_angle)))
            # 계산된 각도들을 가제보 시뮬레이터 속 로봇에게 보냅니다.
            # Sends calculated angles to the robot inside the Gazebo simulator.

        # 6. Rviz Data Publishing (시각화 데이터 발행)
        j_msg = JointState()
        j_msg.header.stamp = self.get_clock().now().to_msg()
        j_msg.name = ['r_shoulder_yaw', 'r_elbow_pitch', 'r_forearm_roll', 'r_wrist_pitch', 'r_index_knuckle', 'r_index_mid', 'r_index_tip']
        j_msg.position = [self.curr_shoulder_yaw, self.curr_elbow_pitch, self.curr_forearm_roll, self.curr_wrist_pitch, finger_angle, finger_angle, finger_angle]
        self.joint_pub.publish(j_msg)
        # Rviz가 로봇을 그릴 수 있도록 현재의 모든 관절 각도를 합쳐서 보냅니다.
        # Combines all joint angles and sends them for Rviz to render the robot.
        
        # 7. Sensor Data Stream (센서 데이터 생성 및 전송)
        if random.random() < LOSS_PROBABILITY: return
        # 유실 확률에 따라 이번 루프의 센서 데이터 전송을 건너뜁니다. / Skips sensor data transmission based on loss probability.

        now = self.get_clock().now()
        timestamp = now.nanoseconds / 1e9
        aw, gw = self.compute_physics(self.curr_forearm_roll, self.curr_elbow_pitch, self.curr_shoulder_yaw, d_f, d_e, d_s, RADIUS_WRIST, d_s, self.curr_elbow_pitch)
        aa, ga = self.compute_physics(self.curr_forearm_roll, self.curr_elbow_pitch, self.curr_shoulder_yaw, d_f, d_e, d_s, RADIUS_FOREARM, d_s, self.curr_elbow_pitch)
        # 물리 엔진을 통해 가짜 센서 값들을 생성합니다. / Generates fake sensor values through the physics engine.

        self.publish_array(self.imu_wrist_pub, [self.add_noise(v, NOISE_ACCEL) for v in aw] + [self.add_noise(v, NOISE_GYRO) for v in gw] + [timestamp])
        self.publish_array(self.imu_arm_pub, [self.add_noise(v, NOISE_ACCEL) for v in aa] + [self.add_noise(v, NOISE_GYRO) for v in ga] + [timestamp])
        # 노이즈를 섞은 IMU 데이터를 100Hz로 전송합니다. / Transmits noisy IMU data at 100Hz.

        if self.tick_count % SLOW_RATE_DIVISOR == 0:
            m_val = self.compute_mag(self.curr_forearm_roll, self.curr_elbow_pitch, self.curr_shoulder_yaw)
            self.publish_array(self.mag_pub, [self.add_noise(v, NOISE_MAG) for v in m_val] + [timestamp])
            acc_mag = math.sqrt(aw[0]**2 + aw[1]**2 + aw[2]**2)
            distorted_flex = self.curr_flex_level + (acc_mag * FLEX_G_SENSITIVITY)
            flex_voltage = self.add_noise((distorted_flex - 1) * (3.3/9.0), NOISE_FLEX)
            self.publish_array(self.flex_pub, [flex_voltage, timestamp])
            # 지자계와 휨 센서 데이터를 50Hz 속도로 전송합니다. / Transmits mag and flex sensor data at 50Hz.

        self.tick_count += 1

def main(args=None):
    rclpy.init(args=args)
    # ROS 2 시스템을 초기화합니다. / Initialize the ROS 2 system.
    node = WearableSensorSim()
    # 시뮬레이션 노드 객체를 생성합니다. / Create the simulation node object.
    try: 
        rclpy.spin(node)
        # 노드를 계속 실행하며 통신을 처리합니다. / Keeps the node running and handles communication.
    except KeyboardInterrupt: pass
    finally:
        pygame.quit()
        # 프로그램 종료 시 Pygame 리소스를 해제합니다. / Release Pygame resources on exit.
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()

if __name__ == '__main__':
    main()