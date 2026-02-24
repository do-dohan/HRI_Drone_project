#!/usr/bin/env python3
# 리눅스 환경에서 Python 3 인터프리터를 사용하여 스크립트를 실행하도록 지정합니다.
# Specifies the use of the Python 3 interpreter to execute the script in a Linux environment.

# Scenario_Driver_node.py
# 사전 정의된 시나리오에 따라 로봇 모델을 제어하고 가상 센서 데이터를 퍼블리시하는 노드입니다.
# Node that controls the robot model and publishes virtual sensor data according to a predefined scenario.

import math
# 삼각함수 연산 및 각도 변환에 필요한 수학 라이브러리를 임포트합니다.
# Imports the math library required for trigonometric operations and angle conversions.

import struct
# 파이썬 데이터 타입을 C 구조체 형식의 바이너리 데이터로 패킹하기 위한 라이브러리를 임포트합니다.
# Imports the library to pack Python data types into C-struct formatted binary data.

import random
# 가우시안 노이즈 생성 및 통신 유실 확률 계산을 위한 난수 생성 라이브러리를 임포트합니다.
# Imports the random number generation library for Gaussian noise and packet loss probability.

from dataclasses import dataclass
# 클래스 생성 시 보일러플레이트 코드를 줄이고 데이터 컨테이너 역할을 수행하도록 데코레이터를 임포트합니다.
# Imports the decorator to reduce boilerplate code and act as a data container when creating classes.

import rclpy
# ROS 2 시스템의 노드 생성 및 통신 관리를 위한 파이썬 클라이언트 라이브러리를 임포트합니다.
# Imports the Python client library for ROS 2 node creation and communication management.

from rclpy.node import Node
# 사용자 정의 ROS 2 노드 작성을 위한 기본 Node 클래스를 임포트합니다.
# Imports the base Node class for writing custom ROS 2 nodes.

from std_msgs.msg import UInt8MultiArray, Float64
# 바이너리 패킷용 UInt8MultiArray와 Gazebo 조인트 제어용 Float64 메시지 타입을 임포트합니다.
# Imports UInt8MultiArray for binary packets and Float64 for Gazebo joint control.


# =============================================================================
# [유동적 변경 가능] 통신 및 물리 노이즈 설정
# 시뮬레이션의 현실성 부여를 위한 노이즈 및 에러 파라미터입니다.
# [Mutable] Configuration for communication and physical noise parameters.
# =============================================================================

LOSS_PROBABILITY = 0.05
# 무선 통신 패킷 유실 확률을 5%로 설정합니다.
# Sets the wireless communication packet loss probability to 5%.

NOISE_ACCEL = 0.02
# 가속도 센서에 적용될 가우시안 노이즈의 표준편차 값입니다.
# Standard deviation of Gaussian noise applied to the accelerometer.

NOISE_GYRO = 0.005
# 자이로스코프 센서에 적용될 가우시안 노이즈의 표준편차 값입니다.
# Standard deviation of Gaussian noise applied to the gyroscope.

NOISE_MAG = 0.000002
# 지자계 센서에 적용될 가우시안 노이즈의 표준편차 값입니다.
# Standard deviation of Gaussian noise applied to the magnetometer.

NOISE_FLEX_TREMOR = 50.0
# 플렉스 센서의 ADC 값에 적용될 고주파 진동(떨림) 노이즈의 강도입니다.
# Intensity of high-frequency tremor noise applied to the flex sensor ADC value.

NOISE_FLEX_DRIFT = 30.0
# 플렉스 센서의 시간에 따른 저주파수 변화(드리프트) 강도입니다.
# Intensity of low-frequency drift applied to the flex sensor over time.

DRIFT_STEP = 0.0002
# 매 틱마다 자이로스코프 바이어스에 누적되는 오차의 크기입니다.
# Magnitude of error accumulated to the gyroscope bias every tick.

FLEX_G_SENSITIVITY = 10.0
# 선형 가속도에 의해 플렉스 센서 값이 왜곡되는 정도를 결정하는 계수입니다.
# Coefficient determining the distortion of the flex sensor value due to linear acceleration.


# =============================================================================
# [변경 금지] 프로토콜 및 스케일링 설정
# 수신부 파서와 동일하게 유지되어야 하는 통신 규약 및 물리 스케일링 값입니다.
# [Immutable] Communication protocol and scaling values that must match the receiver.
# =============================================================================

SOF_1 = 0xAA
# 데이터 프레임 시작을 나타내는 첫 번째 바이트(Start Of Frame)입니다.
# First byte indicating the Start Of Frame.

SOF_2 = 0x55
# 데이터 프레임 시작을 나타내는 두 번째 바이트입니다.
# Second byte indicating the Start Of Frame.

PAYLOAD_LEN = 39
# 전송될 데이터 페이로드의 바이트 단위 길이입니다.
# Length of the transmitted data payload in bytes.

FRAME_LEN = 44
# 헤더, 페이로드, CRC를 모두 포함한 전체 프레임의 바이트 단위 길이입니다.
# Total frame length in bytes including header, payload, and CRC.

HEADER_LEN = 3
# SOF 2바이트와 길이 정보 1바이트를 합친 헤더의 길이입니다.
# Header length combining 2 bytes of SOF and 1 byte of length info.

FLAG_IMU_VALID = 1 << 0
# IMU 데이터의 유효성을 나타내는 비트 플래그입니다. (0번째 비트)
# Bit flag indicating the validity of IMU data. (Bit 0)

FLAG_MAG_UPDATED = 1 << 1
# 지자계 데이터의 갱신 상태를 나타내는 비트 플래그입니다. (1번째 비트)
# Bit flag indicating the update status of magnetometer data. (Bit 1)

FLAG_FLEX_VALID = 1 << 2
# 플렉스 센서 데이터의 유효성을 나타내는 비트 플래그입니다. (2번째 비트)
# Bit flag indicating the validity of flex sensor data. (Bit 2)

ACC_MG_PER_LSB = 0.122
# 가속도 센서의 해상도: LSB당 0.122 밀리중력(mg)을 나타냅니다. (±4g 기준)
# Accelerometer resolution: represents 0.122 milli-g per LSB. (±4g scale)

GYRO_MDPS_PER_LSB = 17.50
# 자이로스코프의 해상도: LSB당 17.5 밀리도(mdps)를 나타냅니다. (±500 dps 기준)
# Gyroscope resolution: represents 17.5 milli-degrees per second per LSB. (±500 dps scale)

ACC_SCALE = ACC_MG_PER_LSB / 1000.0
# 가속도 LSB 값을 중력(G) 단위로 변환하기 위한 상수입니다.
# Constant to convert acceleration LSB values to gravity (G) units.

GYRO_SCALE = (GYRO_MDPS_PER_LSB / 1000.0) * (math.pi / 180.0)
# 자이로스코프 LSB 값을 라디안 퍼 세크(rad/s) 단위로 변환하기 위한 상수입니다.
# Constant to convert gyroscope LSB values to radians per second (rad/s) units.

MAGNET_SCALE = 1.0
# 지자계 센서 데이터 변환을 위한 스케일 상수이며, 현재 원시 값을 그대로 사용합니다.
# Scale constant for magnetometer data conversion, currently passing raw counts.

PAYLOAD_STRUCT = struct.Struct("<15hHIHB")
# 15개의 16비트 정수, 16/32비트 부호 없는 정수 등을 리틀 엔디안으로 패킹하는 구조체입니다.
# Struct packing 15 16-bit integers, unsigned 16/32-bit integers in little-endian format.

assert PAYLOAD_STRUCT.size == 39
# 구조체의 크기가 사전에 정의된 페이로드 길이(39바이트)와 일치하는지 검증합니다.
# Asserts that the struct size strictly matches the predefined payload length (39 bytes).


# =============================================================================
# [변경 금지] 유틸리티 및 수학 함수
# 데이터 처리 및 회전 행렬 연산에 사용되는 기본 함수들입니다.
# [Immutable] Basic functions used for data processing and rotation matrix operations.
# =============================================================================

def add_noise(val, level):
    # 주어진 센서 값에 평균 0, 지정된 표준편차를 갖는 가우시안 노이즈를 합산하여 반환합니다.
    # Returns the sum of the given sensor value and Gaussian noise with mean 0 and specified standard deviation.
    return val + random.gauss(0, level)

def clamp_i16(v: float) -> int:
    # 입력값을 16비트 부호 있는 정수 표현 범위(-32768 ~ 32767) 내로 클램핑(제한)합니다.
    # Clamps the input value within the 16-bit signed integer representation range (-32768 to 32767).
    vi = int(round(v))
    return max(-32768, min(32767, vi))

def acc_g_to_lsb(g: float) -> int:
    # 물리적 가속도(G) 값을 패킷 전송을 위한 16비트 정수형(LSB) 데이터로 변환합니다.
    # Converts physical acceleration (G) to 16-bit integer (LSB) data for packet transmission.
    return clamp_i16(round(g / ACC_SCALE))

def gyro_rad_to_lsb(rad_s: float) -> int:
    # 물리적 각속도(rad/s) 값을 패킷 전송을 위한 16비트 정수형(LSB) 데이터로 변환합니다.
    # Converts physical angular velocity (rad/s) to 16-bit integer (LSB) data for packet transmission.
    return clamp_i16(round(rad_s / GYRO_SCALE))

def crc16_ccitt(data: bytes) -> int:
    # 데이터 무결성 검증을 위해 CCITT 다항식(0x1021)을 사용하여 16비트 CRC를 산출합니다.
    # Calculates a 16-bit CRC using the CCITT polynomial (0x1021) for data integrity verification.
    crc = 0xFFFF
    for b in data:
        crc ^= (b << 8) & 0xFFFF
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc & 0xFFFF

def rot_zyx(roll: float, pitch: float, yaw: float):
    # Z-Y-X 오일러 각(Yaw, Pitch, Roll)을 3x3 직교 회전 행렬 성분으로 변환합니다.
    # Converts Z-Y-X Euler angles (Yaw, Pitch, Roll) into 3x3 orthogonal rotation matrix components.
    cr = math.cos(roll);  sr = math.sin(roll)
    cp = math.cos(pitch); sp = math.sin(pitch)
    cy = math.cos(yaw);   sy = math.sin(yaw)

    r00 = cy*cp
    r01 = cy*sp*sr - sy*cr
    r02 = cy*sp*cr + sy*sr

    r10 = sy*cp
    r11 = sy*sp*sr + cy*cr
    r12 = sy*sp*cr - cy*sr

    r20 = -sp
    r21 = cp*sr
    r22 = cp*cr
    return ((r00, r01, r02), (r10, r11, r12), (r20, r21, r22))

def mat_T_vec(R, v):
    # 회전 행렬 R의 전치(Transpose) 행렬을 벡터 v에 곱하여 좌표계를 변환합니다. (World to Body)
    # Multiplies the transpose of rotation matrix R by vector v for coordinate transformation. (World to Body)
    return (
        R[0][0]*v[0] + R[1][0]*v[1] + R[2][0]*v[2],
        R[0][1]*v[0] + R[1][1]*v[1] + R[2][1]*v[2],
        R[0][2]*v[0] + R[1][2]*v[1] + R[2][2]*v[2],
    )

def mat_mul(A, B):
    # 3x3 행렬 A와 B의 내적 연산을 수행하여 새로운 3x3 행렬을 반환합니다.
    # Performs the dot product operation of 3x3 matrices A and B and returns a new 3x3 matrix.
    return tuple(
        tuple(
            A[i][0]*B[0][j] + A[i][1]*B[1][j] + A[i][2]*B[2][j]
            for j in range(3)
        )
        for i in range(3)
    )

def mat_T(A):
    # 3x3 행렬 A의 행과 열을 교환하여 전치 행렬(Transpose Matrix)을 생성합니다.
    # Swaps rows and columns of 3x3 matrix A to generate a Transpose Matrix.
    return (
        (A[0][0], A[1][0], A[2][0]),
        (A[0][1], A[1][1], A[2][1]),
        (A[0][2], A[1][2], A[2][2]),
    )

def axis_angle_from_R(dR):
    # 회전 변환 행렬 dR로부터 회전 축(Axis) 벡터와 회전 각도(Angle) 스칼라 값을 추출합니다.
    # Extracts the rotation axis vector and rotation angle scalar from the rotation matrix dR.
    tr = dR[0][0] + dR[1][1] + dR[2][2]
    c = (tr - 1.0) * 0.5
    c = max(-1.0, min(1.0, c))
    angle = math.acos(c)
    if angle < 1e-8:
        return (0.0, 0.0, 0.0), 0.0
    s = math.sin(angle)
    ax = (dR[2][1] - dR[1][2]) / (2.0*s)
    ay = (dR[0][2] - dR[2][0]) / (2.0*s)
    az = (dR[1][0] - dR[0][1]) / (2.0*s)
    return (ax, ay, az), angle


@dataclass
# 시나리오의 각 구간을 정의하는 데이터 모델(이름, 시작 시간, 종료 시간) 클래스입니다.
# Data model class defining each segment of the scenario (name, start time, end time).
class Segment:
    name: str
    t0: float
    t1: float


# =============================================================================
# [변경 금지] 시나리오 구간 시작 시간표
# 세그먼트 전환 시 발생하는 불연속성을 억제하기 위한 로컬 시간(u) 매핑 테이블입니다.
# [Immutable] Segment start timetable mapping local time (u) to suppress discontinuity on transitions.
# =============================================================================

SEG_START = {
    "hold_stationary": 0.0,
    "move_slow": 10.0,
    "move_fast": 20.0,
    "hold_after_fast": 30.0,
    "flex_only": 35.0,
    "move_with_flex": 40.0,
}


# =============================================================================
# ROS 2 노드 메인 클래스
# =============================================================================

class ElrsScenarioDriverNode(Node):
    # 시나리오 기반의 운동학적 궤적을 생성하고 센서 데이터를 발행하는 ROS 2 노드입니다.
    # ROS 2 node that generates scenario-based kinematic trajectories and publishes sensor data.

    def __init__(self):
        # 노드 초기화 과정으로 파라미터 로드, 상태 변수 할당 및 퍼블리셔 생성을 수행합니다.
        # Node initialization process: loads parameters, assigns state variables, and creates publishers.
        super().__init__("scenario_driver_node")

        random.seed(1)
        # 실행 간 재현성(Reproducibility)을 보장하기 위해 난수 생성기의 시드 값을 1로 고정합니다.
        # Fixes the random number generator seed to 1 to ensure reproducibility across executions.

        self.rate_hz = int(self.declare_parameter("rate_hz", 50).value)
        # 루프 제어 주파수를 파라미터에서 로드합니다. (기본값: 50Hz)
        # Loads loop control frequency from parameters. (Default: 50Hz)

        self.slow_rate_divisor = int(self.declare_parameter("slow_rate_divisor", 5).value)
        # 지자계 데이터의 갱신 주기 감소율을 정의합니다. (기본값: 5 -> 10Hz 갱신)
        # Defines the update cycle reduction rate for magnetometer data. (Default: 5 -> 10Hz update)

        self.loop_sec = float(self.declare_parameter("loop_sec", 50.0).value)
        # 전체 시나리오의 1회 주기 시간을 50.0초로 설정합니다.
        # Sets the duration of one full scenario cycle to 50.0 seconds.

        # =============================================================================
        # [유동적 변경 가능] 조심스러운 느린 움직임 설정
        # [Mutable] Configuration for slow and precise motion segment.
        # =============================================================================
        self.slow_roll_amp = float(self.declare_parameter("slow_roll_amp_rad", 0.18).value)
        # 저속 제어 구간의 롤 축 진폭 한계입니다.
        # Roll axis amplitude limit for the slow control segment.

        self.slow_pitch_amp = float(self.declare_parameter("slow_pitch_amp_rad", 0.14).value)
        # 저속 제어 구간의 피치 축 진폭 한계입니다.
        # Pitch axis amplitude limit for the slow control segment.

        self.slow_yaw_amp = float(self.declare_parameter("slow_yaw_amp_rad", 0.10).value)
        # 저속 제어 구간의 요 축 진폭 한계입니다.
        # Yaw axis amplitude limit for the slow control segment.

        self.slow_freq = float(self.declare_parameter("slow_freq_hz", 0.20).value)
        # 저속 제어 구간의 궤적 생성 주파수입니다.
        # Trajectory generation frequency for the slow control segment.

        # =============================================================================
        # [유동적 변경 가능] 빠른 움직임 설정
        # [Mutable] Configuration for fast and aggressive motion segment.
        # =============================================================================
        self.fast_roll_amp = float(self.declare_parameter("fast_roll_amp_rad", 0.70).value)
        # 고속 회피 기동 구간의 롤 축 진폭 한계입니다.
        # Roll axis amplitude limit for the fast dodging maneuver segment.

        self.fast_pitch_amp = float(self.declare_parameter("fast_pitch_amp_rad", 0.55).value)
        # 고속 회피 기동 구간의 피치 축 진폭 한계입니다.
        # Pitch axis amplitude limit for the fast dodging maneuver segment.

        self.fast_yaw_amp = float(self.declare_parameter("fast_yaw_amp_rad", 0.26).value)
        # 고속 회피 기동 구간의 요 축 진폭 한계입니다.
        # Yaw axis amplitude limit for the fast dodging maneuver segment.

        self.fast_freq = float(self.declare_parameter("fast_freq_hz", 1.50).value)
        # 고속 회피 기동 구간의 궤적 생성 주파수입니다.
        # Trajectory generation frequency for the fast dodging maneuver segment.

        # =============================================================================
        # [유동적 변경 가능] 플렉스와 함께 움직이는 구간 설정
        # [Mutable] Configuration for simultaneous flex and motion segment.
        # =============================================================================
        self.with_flex_roll_amp = float(self.declare_parameter("with_flex_roll_amp_rad", 0.35).value)
        # 복합 동작 구간의 롤 축 진폭 한계입니다.
        # Roll axis amplitude limit for the complex motion segment.

        self.with_flex_pitch_amp = float(self.declare_parameter("with_flex_pitch_amp_rad", 0.25).value)
        # 복합 동작 구간의 피치 축 진폭 한계입니다.
        # Pitch axis amplitude limit for the complex motion segment.

        self.with_flex_yaw_amp = float(self.declare_parameter("with_flex_yaw_amp_rad", 0.18).value)
        # 복합 동작 구간의 요 축 진폭 한계입니다.
        # Yaw axis amplitude limit for the complex motion segment.

        self.with_flex_freq = float(self.declare_parameter("with_flex_freq_hz", 0.90).value)
        # 복합 동작 구간의 궤적 생성 주파수입니다.
        # Trajectory generation frequency for the complex motion segment.

        # =============================================================================
        # [유동적 변경 가능] 관성 힘(원심력) 설정
        # [Mutable] Configuration for internal inertial (centrifugal) forces.
        # =============================================================================
        self.fast_lin_acc_g = float(self.declare_parameter("fast_lin_acc_g", 0.25).value)
        # 고속 구간 궤적에서 선형 가속도 벡터에 합성되는 관성 스케일 인자입니다.
        # Inertial scale factor synthesized into the linear acceleration vector during the fast segment.

        self.with_flex_lin_acc_g = float(self.declare_parameter("with_flex_lin_acc_g", 0.06).value)
        # 복합 동작 구간 궤적에서 선형 가속도 벡터에 합성되는 관성 스케일 인자입니다.
        # Inertial scale factor synthesized into linear acceleration during the complex motion segment.

        # =============================================================================
        # [변경 금지] 3.3 규약: 총 쏘는 기본 자세 (NP_gun) 오프셋
        # [Immutable] Protocol 3.3: Base offsets defining the NP_gun pose kinematics.
        # =============================================================================
        self.np_shoulder_yaw = 0.0
        # NP_gun 규약에 따른 어깨 요 축 초기 오프셋 값입니다.
        # Initial offset value for shoulder yaw axis according to NP_gun protocol.
        self.np_elbow_pitch = 1.5708
        # NP_gun 규약에 따른 팔꿈치 피치 축 초기 오프셋 값입니다.
        # Initial offset value for elbow pitch axis according to NP_gun protocol.
        self.np_forearm_roll = 0.0
        # NP_gun 규약에 따른 전완 롤 축 초기 오프셋 값입니다.
        # Initial offset value for forearm roll axis according to NP_gun protocol.
        self.np_wrist_pitch = 0.0
        # NP_gun 규약에 따른 손목 피치 축 초기 오프셋 값입니다.
        # Initial offset value for wrist pitch axis according to NP_gun protocol.

        # =============================================================================
        # ROS 2 퍼블리셔 생성
        # ROS 2 publisher instantiation
        # =============================================================================
        self.pub = self.create_publisher(UInt8MultiArray, "elrs_packet", 10)
        # 시리얼 통신을 모사한 바이너리 패킷 데이터 발행을 위한 퍼블리셔를 생성합니다.
        # Creates a publisher for publishing binary packet data mimicking serial communication.

        self.pub_cmd_shoulder = self.create_publisher(Float64, '/model/wearable_hand/joint/r_shoulder_yaw/cmd_pos', 10)
        # Gazebo 로봇 모델의 어깨 요 조인트 제어 명령 퍼블리셔입니다.
        # Publisher for Gazebo robot model's shoulder yaw joint control commands.
        self.pub_cmd_elbow = self.create_publisher(Float64, '/model/wearable_hand/joint/r_elbow_pitch/cmd_pos', 10)
        # Gazebo 로봇 모델의 팔꿈치 피치 조인트 제어 명령 퍼블리셔입니다.
        # Publisher for Gazebo robot model's elbow pitch joint control commands.
        self.pub_cmd_forearm = self.create_publisher(Float64, '/model/wearable_hand/joint/r_forearm_roll/cmd_pos', 10)
        # Gazebo 로봇 모델의 전완 롤 조인트 제어 명령 퍼블리셔입니다.
        # Publisher for Gazebo robot model's forearm roll joint control commands.
        self.pub_cmd_wrist = self.create_publisher(Float64, '/model/wearable_hand/joint/r_wrist_pitch/cmd_pos', 10)
        # Gazebo 로봇 모델의 손목 피치 조인트 제어 명령 퍼블리셔입니다.
        # Publisher for Gazebo robot model's wrist pitch joint control commands.
        self.pub_cmd_index1 = self.create_publisher(Float64, '/model/wearable_hand/joint/r_index_knuckle/cmd_pos', 10)
        # Gazebo 로봇 모델의 검지 첫 번째 관절 제어 명령 퍼블리셔입니다.
        # Publisher for Gazebo robot model's index finger first knuckle control commands.
        self.pub_cmd_index2 = self.create_publisher(Float64, '/model/wearable_hand/joint/r_index_mid/cmd_pos', 10)
        # Gazebo 로봇 모델의 검지 중간 관절 제어 명령 퍼블리셔입니다.
        # Publisher for Gazebo robot model's index finger mid joint control commands.
        self.pub_cmd_index3 = self.create_publisher(Float64, '/model/wearable_hand/joint/r_index_tip/cmd_pos', 10)
        # Gazebo 로봇 모델의 검지 끝 관절 제어 명령 퍼블리셔입니다.
        # Publisher for Gazebo robot model's index finger tip joint control commands.

        # =============================================================================
        # 내부 상태 및 시간 변수 초기화
        # Internal state and time variable initialization
        # =============================================================================
        self.dt = 1.0 / float(self.rate_hz)
        # 주파수 기반의 단위 제어 주기(dt) 스칼라 값을 산출합니다.
        # Calculates the unit control period (dt) scalar value based on frequency.
        self.tick_count = 0
        # 시뮬레이션 루프의 누적 실행 횟수를 추적하는 카운터 변수입니다.
        # Counter variable tracking the accumulated execution count of the simulation loop.
        self.seq = 0
        # 패킷 식별 및 순서 검증을 위한 16비트 시퀀스 번호 변수입니다.
        # 16-bit sequence number variable for packet identification and order verification.
        self.sim_t = 0.0
        # 시뮬레이션 환경 내 누적 경과 시간(초 단위)을 나타내는 변수입니다.
        # Variable representing the accumulated elapsed time (in seconds) in the simulation environment.
        self.sim_t_us = 0
        # 마이크로컨트롤러 호환을 위한 누적 경과 시간(마이크로초 단위) 변수입니다.
        # Variable representing elapsed time (in microseconds) for microcontroller compatibility.

        self.g_world = (0.0, 0.0, 1.0)
        # 월드 좌표계 기준의 중력 벡터 상수를 초기화합니다.
        # Initializes the gravity vector constant based on the world coordinate system.

        self.B_world = (600.0, 50.0, 200.0)
        # 월드 좌표계 기준의 가상 자기장 벡터 원시값을 초기화합니다.
        # Initializes the virtual magnetic field vector raw counts based on the world coordinate system.

        self.cached_mag = (int(self.B_world[0]), int(self.B_world[1]), int(self.B_world[2]))
        # 빈번한 행렬 연산을 방지하기 위한 지자계 데이터 캐시를 할당합니다.
        # Assigns magnetometer data cache to prevent frequent matrix operations.

        self.prev_R_w = rot_zyx(self.np_forearm_roll, self.np_wrist_pitch, 0.0)
        # NP_gun 규약을 준수하는 손목 초기 회전 행렬을 할당합니다.
        # Assigns the initial wrist rotation matrix complying with the NP_gun protocol.
        self.prev_R_a = rot_zyx(0.0, self.np_elbow_pitch, self.np_shoulder_yaw)
        # NP_gun 규약을 준수하는 전완 초기 회전 행렬을 할당합니다.
        # Assigns the initial forearm rotation matrix complying with the NP_gun protocol.

        self.gyro_bias_x = 0.0
        # 자이로스코프 X축 누적 바이어스 상태 변수입니다.
        # Accumulated bias state variable for gyroscope X-axis.
        self.gyro_bias_y = 0.0
        # 자이로스코프 Y축 누적 바이어스 상태 변수입니다.
        # Accumulated bias state variable for gyroscope Y-axis.
        self.gyro_bias_z = 0.0
        # 자이로스코프 Z축 누적 바이어스 상태 변수입니다.
        # Accumulated bias state variable for gyroscope Z-axis.

        # =============================================================================
        # 시나리오 흐름 제어 정의
        # Scenario flow control definitions
        # =============================================================================
        self.segments = [
            Segment("hold_stationary", 0.0, 10.0),
            # 0~10초: 정적 유지 궤적 생성 구간입니다.
            # 0~10s: Static holding trajectory generation segment.
            Segment("move_slow", 10.0, 20.0),
            # 10~20초: 저주파수 모션 궤적 생성 구간입니다.
            # 10~20s: Low-frequency motion trajectory generation segment.
            Segment("move_fast", 20.0, 30.0),
            # 20~30초: 고주파수 모션 궤적 생성 구간입니다.
            # 20~30s: High-frequency motion trajectory generation segment.
            Segment("hold_after_fast", 30.0, 35.0),
            # 30~35초: 동적 궤적 종료 후 정적 유지 구간입니다.
            # 30~35s: Static holding segment after dynamic trajectory completion.
            Segment("flex_only", 35.0, 40.0),
            # 35~40초: 조인트 고정 및 단일 플렉스 값 램프 구간입니다.
            # 35~40s: Joint fixed and single flex value ramp segment.
            Segment("move_with_flex", 40.0, 50.0),
            # 40~50초: 플렉스 센서와 조인트 모션의 동기화 궤적 구간입니다.
            # 40~50s: Synchronized trajectory segment of flex sensor and joint motion.
        ]

        self.current_segment = None
        # 현재 활성화된 세그먼트의 식별자를 저장하는 변수입니다.
        # Variable storing the identifier of the currently active segment.

        self.timer = self.create_timer(self.dt, self.on_tick)
        # 제어 주기(dt)마다 on_tick 콜백을 비동기 호출하는 타이머 객체를 생성합니다.
        # Creates a timer object that asynchronously calls the on_tick callback every control period (dt).
        
        self.get_logger().info(
            f"Scenario driver started: {self.rate_hz}Hz, mag_div={self.slow_rate_divisor}, loop={self.loop_sec}s"
        )
        # 노드 초기화 성공 및 파라미터 상태를 시스템 로그로 출력합니다.
        # Outputs node initialization success and parameter state to the system log.

    def segment_for_time(self, t: float) -> str:
        # 시뮬레이션 타임스탬프 기반 활성 세그먼트 식별자 반환 함수입니다.
        # Active segment identifier return function based on simulation timestamp.
        for seg in self.segments:
            if seg.t0 <= t < seg.t1:
                return seg.name
        return self.segments[-1].name

    def flex_profile(self, t: float) -> int:
        # 시간-플렉스 ADC 매핑 궤적 연산 함수입니다.
        # Time-flex ADC mapping trajectory computation function.

        if t < 35.0:
            # 35초 이전 조건 시 초기 고정값 800을 반환합니다.
            # Returns the initial fixed value 800 if the condition is before 35 seconds.
            return 800

        if t < 40.0:
            # 35초~40초 구간 내 선형 보간을 통한 램프 프로파일을 생성합니다.
            # Generates a ramp profile through linear interpolation within the 35s~40s segment.
            u = (t - 35.0) / 5.0
            v = int(round(800 + u * (3200 - 800)))
            return max(0, min(4095, v))

        # 40초 이후 사인파 형태의 연속 동적 프로파일을 생성합니다.
        # Generates a continuous dynamic profile in sine wave form after 40 seconds.
        u = t - 40.0
        v = int(round(2000 + 1200 * math.sin(2.0 * math.pi * 0.5 * u)))
        return max(0, min(4095, v))

    def euler_profile(self, seg_name: str, t: float):
        # 로컬 시간 변수(u) 매핑을 적용하여 오일러 각 궤적의 불연속성을 제거합니다.
        # Applies local time variable (u) mapping to eliminate discontinuity in the Euler angle trajectory.
        u = t - SEG_START.get(seg_name, 0.0)

        if seg_name in ("hold_stationary", "hold_after_fast", "flex_only"):
            # 정적 상태 세그먼트에 대한 제로-델타 조인트 제어를 선언합니다.
            # Declares zero-delta joint control for static state segments.
            rw = pw = yw = 0.0

        elif seg_name == "move_slow":
            # 저주파수 세그먼트 매개변수를 참조하여 운동학적 궤적을 할당합니다.
            # Allocates kinematic trajectories referencing low-frequency segment parameters.
            f = self.slow_freq
            rw = self.slow_roll_amp * math.sin(2.0 * math.pi * f * u)
            pw = self.slow_pitch_amp * math.sin(2.0 * math.pi * f * u)
            yw = self.slow_yaw_amp * math.sin(2.0 * math.pi * 0.5 * f * u)

        elif seg_name == "move_fast":
            # 고주파수 세그먼트 매개변수를 참조하여 운동학적 궤적을 할당합니다.
            # Allocates kinematic trajectories referencing high-frequency segment parameters.
            f = self.fast_freq
            rw = self.fast_roll_amp * math.sin(2.0 * math.pi * f * u)
            pw = self.fast_pitch_amp * math.sin(2.0 * math.pi * f * u)
            yw = self.fast_yaw_amp * math.sin(2.0 * math.pi * 0.5 * f * u)

        else:  
            # 복합 모션 세그먼트에 대한 모션 스케일 할당 로직입니다.
            # Motion scale allocation logic for the complex motion segment.
            f = self.with_flex_freq
            rw = self.with_flex_roll_amp * math.sin(2.0 * math.pi * f * u)
            pw = self.with_flex_pitch_amp * math.sin(2.0 * math.pi * f * u)
            yw = self.with_flex_yaw_amp * math.sin(2.0 * math.pi * 0.5 * f * u)

        # 근골격계 동역학 근사를 위해 상완 조인트 각도 변위를 손목 대비 55%로 축소 연산합니다.
        # Scales arm joint angular displacement to 55% of wrist to approximate musculoskeletal dynamics.
        ra = 0.55 * rw
        pa = 0.55 * pw
        ya = 0.55 * yw

        # 산출된 로컬 조인트 변위 데이터에 NP_gun 기준 절대 오프셋 상수를 합산 처리합니다.
        # Adds NP_gun reference absolute offset constants to the calculated local joint displacement data.
        rw_final = rw + self.np_forearm_roll
        pw_final = pw + self.np_wrist_pitch
        yw_final = yw

        ra_final = ra
        pa_final = pa + self.np_elbow_pitch
        ya_final = ya + self.np_shoulder_yaw

        return (rw_final, pw_final, yw_final), (ra_final, pa_final, ya_final)
        # 연산이 완료된 3D 손목/전완 오일러 각 튜플 데이터를 반환합니다.
        # Returns the computed 3D wrist/forearm Euler angle tuple data.

    def compute_gyro_from_R(self, R_prev, R_cur) -> tuple:
        # 역방향 회전 행렬 연산을 통한 델타 회전 행렬 도출을 수행합니다.
        # Derives the delta rotation matrix through reverse rotation matrix operations.
        dR = mat_mul(mat_T(R_prev), R_cur)
        # 델타 회전 행렬을 기반으로 회전 축 벡터 성분과 각도 스칼라를 연산합니다.
        # Computes rotation axis vector components and angle scalar based on the delta rotation matrix.
        axis, angle = axis_angle_from_R(dR)
        if angle <= 1e-8:
            # 짐벌 락(Gimbal lock) 및 수치 불안정성 방지를 위한 임계값 하한 필터링입니다.
            # Threshold lower bound filtering to prevent gimbal lock and numerical instability.
            return (0.0, 0.0, 0.0)
        # 회전 변위를 주기 시간(dt)으로 나누어 로컬 프레임 각속도(rad/s)를 도출합니다.
        # Derives local frame angular velocity (rad/s) by dividing rotational displacement by cycle time (dt).
        wx = axis[0] * (angle / self.dt)
        wy = axis[1] * (angle / self.dt)
        wz = axis[2] * (angle / self.dt)
        return (wx, wy, wz)

    def on_tick(self):
        # 타이머 인터럽트에 의해 구동되는 시뮬레이션 상태 업데이트 및 발행 루틴입니다.
        # Simulation state update and publishing routine driven by the timer interrupt.

        self.sim_t += self.dt
        # 초 단위 시뮬레이션 절대 누적 시간을 연산합니다.
        # Computes absolute accumulated simulation time in seconds.
        self.sim_t_us += int(round(self.dt * 1e6))
        # 패킷 구조체 호환을 위한 마이크로초 단위 시간 변환 및 누적 연산입니다.
        # Microsecond unit time conversion and accumulation operation for packet struct compatibility.

        if self.sim_t >= self.loop_sec:
            # 설정된 시나리오 루프 타임 초과 판정 시 환경 변수 초기화 블록을 실행합니다.
            # Executes environment variable reset block upon checking configured scenario loop time excess.
            self.sim_t = 0.0
            self.seq = 0
            self.tick_count = 0
            self.prev_R_w = rot_zyx(self.np_forearm_roll, self.np_wrist_pitch, 0.0)
            self.prev_R_a = rot_zyx(0.0, self.np_elbow_pitch, self.np_shoulder_yaw)

        seg = self.segment_for_time(self.sim_t)
        # 시뮬레이션 타임스탬프와 일치하는 상태기(State Machine) 세그먼트를 쿼리합니다.
        # Queries the state machine segment matching the simulation timestamp.
        if seg != self.current_segment:
            # 세그먼트 상태 전이 발생 시 콘솔 로그 인쇄 이벤트를 트리거합니다.
            # Triggers console log print event upon segment state transition occurrence.
            self.current_segment = seg
            self.get_logger().info(f"[scenario] {seg}")

        flags = 0
        # 비트 연산을 위해 패킷 상태 플래그 레지스터를 초기화합니다.
        # Initializes packet status flag register for bitwise operations.
        flags |= FLAG_IMU_VALID
        # IMU 데이터 무결성 승인 플래그(비트 0)를 설정합니다.
        # Sets the IMU data integrity approval flag (Bit 0).
        flags |= FLAG_FLEX_VALID
        # 플렉스 데이터 무결성 승인 플래그(비트 2)를 설정합니다.
        # Sets the Flex data integrity approval flag (Bit 2).

        (rw, pw, yw), (ra, pa, ya) = self.euler_profile(seg, self.sim_t)
        # 로컬 시간 변수를 참조하여 타겟 모션 오일러 각 튜플을 산출합니다.
        # Computes target motion Euler angle tuples referencing the local time variable.
        Rw = rot_zyx(rw, pw, yw)
        # 생성된 손목 조인트 각도를 직교 회전 행렬로 변환 매핑합니다.
        # Converts the generated wrist joint angles into orthogonal rotation matrix mapping.
        Ra = rot_zyx(ra, pa, ya)
        # 생성된 상완 조인트 각도를 직교 회전 행렬로 변환 매핑합니다.
        # Converts the generated arm joint angles into orthogonal rotation matrix mapping.

        w_w = self.compute_gyro_from_R(self.prev_R_w, Rw)
        # 직전 프레임과의 역행렬 연산을 통해 손목 자이로스코프 출력 벡터를 추정합니다.
        # Estimates wrist gyroscope output vector via inverse matrix operations with the previous frame.
        w_a = self.compute_gyro_from_R(self.prev_R_a, Ra)
        # 직전 프레임과의 역행렬 연산을 통해 상완 자이로스코프 출력 벡터를 추정합니다.
        # Estimates arm gyroscope output vector via inverse matrix operations with the previous frame.
        self.prev_R_w = Rw
        # 차기 루프 차분 연산을 위해 현재 손목 행렬 상태를 할당 및 저장합니다.
        # Assigns and stores current wrist matrix state for next loop differential operations.
        self.prev_R_a = Ra
        # 차기 루프 차분 연산을 위해 현재 상완 행렬 상태를 할당 및 저장합니다.
        # Assigns and stores current arm matrix state for next loop differential operations.

        a_w = mat_T_vec(Rw, self.g_world)
        # 전치 행렬 연산을 활용하여 지구 중력 벡터를 손목 로컬 프레임으로 투영합니다.
        # Projects Earth's gravity vector onto the wrist local frame utilizing transpose matrix operations.
        a_a = mat_T_vec(Ra, self.g_world)
        # 전치 행렬 연산을 활용하여 지구 중력 벡터를 상완 로컬 프레임으로 투영합니다.
        # Projects Earth's gravity vector onto the arm local frame utilizing transpose matrix operations.

        u_local = self.sim_t - SEG_START.get(seg, 0.0)
        # 세그먼트 내 위상(Phase) 일관성 보장을 위한 순수 델타 시간(u_local)을 산정합니다.
        # Calculates pure delta time (u_local) ensuring phase consistency within the segment.

        if seg == "move_fast":
            # 가상 관성력을 부여하기 위해 조인트 진동 가속도와 선형 스케일 팩터를 내적 합산합니다.
            # Sums dot products of joint oscillation acceleration and linear scale factors to impart virtual inertia.
            k = self.fast_lin_acc_g
            a_w = (a_w[0] + k * math.sin(2.0 * math.pi * self.fast_freq * u_local),
                   a_w[1] + k * math.cos(2.0 * math.pi * self.fast_freq * u_local),
                   a_w[2])
            a_a = (a_a[0] + 0.8 * k * math.sin(2.0 * math.pi * self.fast_freq * u_local),
                   a_a[1] + 0.8 * k * math.cos(2.0 * math.pi * self.fast_freq * u_local),
                   a_a[2])

        if seg == "move_with_flex":
            # 복합 동작 궤적 상 가상 관성력을 부여하기 위한 가속도 벡터 합산 로직입니다.
            # Acceleration vector summation logic to impart virtual inertia on the complex motion trajectory.
            k = self.with_flex_lin_acc_g
            a_w = (a_w[0] + k * math.sin(2.0 * math.pi * self.with_flex_freq * u_local),
                   a_w[1] + k * math.cos(2.0 * math.pi * self.with_flex_freq * u_local),
                   a_w[2])
            a_a = (a_a[0] + 0.8 * k * math.sin(2.0 * math.pi * self.with_flex_freq * u_local),
                   a_a[1] + 0.8 * k * math.cos(2.0 * math.pi * self.with_flex_freq * u_local),
                   a_a[2])

        if (self.tick_count % self.slow_rate_divisor) == 0:
            # 연산 자원 최적화를 위해 지정된 나눗셈 주기에 도달 시에만 지자계 행렬을 변환합니다.
            # Transforms magnetometer matrix only upon reaching the specified divisor cycle to optimize resources.
            flags |= FLAG_MAG_UPDATED
            # 지자계 데이터 갱신 알림 비트 플래그를 할당합니다.
            # Assigns magnetometer data update notification bit flag.
            m_body = mat_T_vec(Rw, self.B_world)
            self.cached_mag = (int(round(m_body[0])), int(round(m_body[1])), int(round(m_body[2])))

        mx, my, mz = self.cached_mag
        # 비-업데이트 사이클 시 메모리에 상주하는 캐시 데이터 값을 호출합니다.
        # Recalls cache data values residing in memory during non-update cycles.
        flex_adc = self.flex_profile(self.sim_t)
        # 산출된 아날로그 범위의 프로파일 데이터를 획득합니다.
        # Obtains computed profile data within the analog range.

        # =============================================================================
        # 가제보 화면 속 마리오네트 로봇 인형극 강제 조종선
        # Force-control Gazebo puppet joints
        # =============================================================================
        self.pub_cmd_shoulder.publish(Float64(data=float(ya)))
        # Gazebo 브릿지로 상완 요 조인트의 라디안 제어 명령을 다이렉트 전송합니다.
        # Directly transmits radian control commands of arm yaw joint to the Gazebo bridge.
        self.pub_cmd_elbow.publish(Float64(data=float(pa)))
        # Gazebo 브릿지로 상완 피치 조인트의 제어 명령을 전송합니다.
        # Transmits control commands of arm pitch joint to the Gazebo bridge.
        self.pub_cmd_forearm.publish(Float64(data=float(rw)))
        # Gazebo 브릿지로 전완 롤 조인트의 제어 명령을 전송합니다.
        # Transmits control commands of forearm roll joint to the Gazebo bridge.
        self.pub_cmd_wrist.publish(Float64(data=float(pw)))
        # Gazebo 브릿지로 손목 피치 조인트의 제어 명령을 전송합니다.
        # Transmits control commands of wrist pitch joint to the Gazebo bridge.

        finger_angle = (flex_adc - 800) * (1.57 / (4095 - 800))
        # 12비트 분해능 ADC 스케일을 Gazebo 구동 한계인 0.0~1.57 라디안 구간으로 스케일 매핑합니다.
        # Maps 12-bit resolution ADC scale to the 0.0~1.57 radian range of Gazebo limits.
        finger_angle = max(0.0, min(1.57, finger_angle))
        # 조인트 변위 한계 이탈 방지를 위한 클램핑 연산을 수행합니다.
        # Performs clamping operations to prevent joint displacement from exceeding limits.
        self.pub_cmd_index1.publish(Float64(data=float(finger_angle)))
        # 매핑된 라디안 데이터를 검지 제 1관절의 목표 각도로 발행합니다.
        # Publishes the mapped radian data as the target angle for the index finger's first joint.
        self.pub_cmd_index2.publish(Float64(data=float(finger_angle)))
        # 매핑된 라디안 데이터를 검지 제 2관절의 목표 각도로 발행합니다.
        # Publishes mapped radian data as target angle for index second joint.
        self.pub_cmd_index3.publish(Float64(data=float(finger_angle)))
        # 매핑된 라디안 데이터를 검지 제 3관절의 목표 각도로 발행합니다.
        # Publishes mapped radian data as target angle for index third joint.

        # =============================================================================
        # 노이즈 주입 및 센서 데이터 처리부
        # Noise injection and sensor data processing module
        # =============================================================================
        ax, ay, az = a_w
        # 손목 가속도 튜플 객체를 X, Y, Z 단일 스칼라 변수로 해제(Unpack)합니다.
        # Unpacks wrist acceleration tuple object into X, Y, Z single scalar variables.
        gx, gy, gz = w_w
        # 손목 각속도 튜플 객체를 X, Y, Z 단일 변수로 해제합니다.
        # Unpacks wrist angular velocity tuple object into X, Y, Z single variables.
        axa, aya, aza = a_a
        # 전완 가속도 튜플 객체를 단일 변수로 해제합니다.
        # Unpacks arm acceleration tuple object into single variables.
        gxa, gya, gza = w_a
        # 전완 각속도 튜플 객체를 단일 변수로 해제합니다.
        # Unpacks arm angular velocity tuple object into single variables.

        self.gyro_bias_x += random.gauss(0, DRIFT_STEP)
        # 저주파수 적분 오차(바이어스 워크)를 모사하기 위해 누적 드리프트를 덧셈합니다.
        # Accumulates drift addition to mimic low-frequency integration errors (bias walk).
        self.gyro_bias_y += random.gauss(0, DRIFT_STEP)
        # Y축 누적 드리프트 덧셈입니다.
        # Y-axis accumulated drift addition.
        self.gyro_bias_z += random.gauss(0, DRIFT_STEP)
        # Z축 누적 드리프트 덧셈입니다.
        # Z-axis accumulated drift addition.

        ax_raw = add_noise(ax, NOISE_ACCEL)
        # 이상적 운동학 가속도 벡터에 가우시안 백색 잡음을 합산합니다.
        # Sums Gaussian white noise to ideal kinematic acceleration vectors.
        ay_raw = add_noise(ay, NOISE_ACCEL)
        # Y축 가속도 잡음 합산입니다.
        # Y-axis acceleration noise addition.
        az_raw = add_noise(az, NOISE_ACCEL)
        # Z축 가속도 잡음 합산입니다.
        # Z-axis acceleration noise addition.

        gx_raw = add_noise(gx, NOISE_GYRO) + self.gyro_bias_x
        # 이상적 각속도에 고주파 백색 잡음 및 누적 저주파 바이어스를 동시 주입합니다.
        # Simultaneously injects high-frequency white noise and low-frequency bias to ideal angular velocities.
        gy_raw = add_noise(gy, NOISE_GYRO) + self.gyro_bias_y
        # Y축 각속도 잡음 및 바이어스 합산입니다.
        # Y-axis angular velocity noise and bias addition.
        gz_raw = add_noise(gz, NOISE_GYRO) + self.gyro_bias_z
        # Z축 각속도 잡음 및 바이어스 합산입니다.
        # Z-axis angular velocity noise and bias addition.

        axa_raw = add_noise(axa, NOISE_ACCEL)
        # 전완 프레임 X축 가속도 잡음 합산입니다.
        # Arm frame X-axis acceleration noise addition.
        aya_raw = add_noise(aya, NOISE_ACCEL)
        # 전완 프레임 Y축 가속도 잡음 합산입니다.
        # Arm frame Y-axis acceleration noise addition.
        aza_raw = add_noise(aza, NOISE_ACCEL)
        # 전완 프레임 Z축 가속도 잡음 합산입니다.
        # Arm frame Z-axis acceleration noise addition.

        gxa_raw = add_noise(gxa, NOISE_GYRO) + self.gyro_bias_x
        # 전완 프레임 X축 각속도 잡음 및 바이어스 합산입니다.
        # Arm frame X-axis angular velocity noise and bias addition.
        gya_raw = add_noise(gya, NOISE_GYRO) + self.gyro_bias_y
        # 전완 프레임 Y축 각속도 잡음 및 바이어스 합산입니다.
        # Arm frame Y-axis angular velocity noise and bias addition.
        gza_raw = add_noise(gza, NOISE_GYRO) + self.gyro_bias_z
        # 전완 프레임 Z축 각속도 잡음 및 바이어스 합산입니다.
        # Arm frame Z-axis angular velocity noise and bias addition.

        mx_raw = add_noise(mx, NOISE_MAG)
        # 지자계 원시 벡터에 저레벨 환경 자기장 노이즈를 합산합니다.
        # Sums low-level environmental magnetic noise to raw magnetometer vectors.
        my_raw = add_noise(my, NOISE_MAG)
        # 지자계 Y축 자기장 노이즈 합산입니다.
        # Magnetometer Y-axis magnetic noise addition.
        mz_raw = add_noise(mz, NOISE_MAG)
        # 지자계 Z축 자기장 노이즈 합산입니다.
        # Magnetometer Z-axis magnetic noise addition.

        acc_magnitude = math.sqrt(ax_raw**2 + ay_raw**2 + az_raw**2)
        # 선형 가속도 벡터의 스칼라 놈(Norm)을 통해 전체 관성 합력을 추정합니다.
        # Estimates total inertial resultant force through the scalar norm of linear acceleration vectors.
        acc_dyn = max(0.0, acc_magnitude - 1.0)
        # 정지 상태 중력 1G를 소거하여 순수 동적 가속 성분만을 추출합니다.
        # Extracts only pure dynamic acceleration components by eliminating 1G static gravity.

        flex_noise = random.gauss(0, NOISE_FLEX_TREMOR)
        # 저항 기반 플렉스 센서의 기계적 파생 노이즈를 가우시안 생성합니다.
        # Generates Gaussian mechanical derivation noise for resistance-based flex sensors.
        flex_drift = math.sin(self.tick_count * 0.05) * NOISE_FLEX_DRIFT
        # 저항 가열에 따른 측정점 저주파 드리프트의 주기 모델을 구축합니다.
        # Builds periodic model of low-frequency measurement point drift due to resistive heating.
        flex_raw = flex_adc + flex_noise + flex_drift + (acc_dyn * FLEX_G_SENSITIVITY)
        # 베이스 프로파일, 진동, 드리프트 및 가속 동역학 성분을 일괄 합산 적용합니다.
        # Consolidates base profile, tremor, drift, and acceleration dynamics components.

        acc_raw_int = (acc_g_to_lsb(ax_raw), acc_g_to_lsb(ay_raw), acc_g_to_lsb(az_raw))
        # 실수형 손목 가속도 데이터를 LSB 단위 통신 정수로 스케일 클램핑 변환합니다.
        # Converts and scale-clamps float wrist accel data into LSB unit communication integers.
        gyro_raw_int = (gyro_rad_to_lsb(gx_raw), gyro_rad_to_lsb(gy_raw), gyro_rad_to_lsb(gz_raw))
        # 실수형 손목 각속도 데이터를 LSB 정수로 변환 매핑합니다.
        # Converts float wrist gyro data into LSB integers mapping.
        acc_arm_raw_int = (acc_g_to_lsb(axa_raw), acc_g_to_lsb(aya_raw), acc_g_to_lsb(aza_raw))
        # 실수형 전완 가속도 데이터를 통신 전송용 LSB 정수로 변환합니다.
        # Converts float arm accel data into LSB integers for comm transmission.
        gyro_arm_raw_int = (gyro_rad_to_lsb(gxa_raw), gyro_rad_to_lsb(gya_raw), gyro_rad_to_lsb(gza_raw))
        # 실수형 전완 각속도 데이터를 정수 형태로 다운캐스팅 변환합니다.
        # Down-cast converts float arm gyro data into integer formats.
        mag_raw_int = (clamp_i16(mx_raw), clamp_i16(my_raw), clamp_i16(mz_raw))
        # 지자계 데이터를 16비트 오버플로우 방지 구역 내로 클램프 제한 정수화합니다.
        # Clamp-limits and integerizes mag data within 16-bit overflow prevention zone.

        final_flex = int(max(0, min(4095, flex_raw)))
        # 혼합된 플렉스 데이터 레벨을 12비트 하드웨어 ADC의 물리적 경계(0~4095) 내로 컷오프합니다.
        # Cuts off mixed flex data levels within physical boundaries (0~4095) of 12-bit hardware ADC.

        seq_u16 = self.seq & 0xFFFF
        # 모듈로 마스킹 비트 연산을 적용하여 통신 시퀀스 번호를 uint16 규격으로 제한합니다.
        # Limits communication sequence number to uint16 specs applying modulo masking bitwise ops.
        t_us_u32 = self.sim_t_us & 0xFFFFFFFF
        # 마이크로초 타임스탬프 데이터를 오버플로우를 막기 위해 32비트 uint 규격으로 마스킹합니다.
        # Masks microsecond timestamp data into 32-bit uint specs to prevent overflow.
        self.seq = (self.seq + 1) & 0xFFFF
        # 패킷 증분 루틴을 수행하며, 65535를 넘으면 안전하게 0으로 순환(Round-Robin)합니다.
        # Executes packet increment routine, safely round-robins to 0 if passing 65535.

        drop = (random.random() < LOSS_PROBABILITY)
        # 패킷 조립 전, 가상 네트워크 인터페이스의 드랍 스위치를 활성화할 확률 평가를 진행합니다.
        # Evaluates probability to trigger virtual network interface drop switch before packet assembly.

        payload = PAYLOAD_STRUCT.pack(
            # 정의된 39바이트 리틀엔디안 구조체 형식에 맞추어 18개의 패킷 세그먼트를 묶음 포장합니다.
            # Batch-packs 18 packet segments matching defined 39-byte little-endian struct format.
            acc_raw_int[0], acc_raw_int[1], acc_raw_int[2],
            gyro_raw_int[0], gyro_raw_int[1], gyro_raw_int[2],
            acc_arm_raw_int[0], acc_arm_raw_int[1], acc_arm_raw_int[2],
            gyro_arm_raw_int[0], gyro_arm_raw_int[1], gyro_arm_raw_int[2],
            mag_raw_int[0], mag_raw_int[1], mag_raw_int[2],
            final_flex & 0xFFFF,
            int(t_us_u32),
            int(seq_u16),
            int(flags) & 0xFF
        )
        assert len(payload) == PAYLOAD_LEN
        # C-Byte 배열 구조체가 사전에 정의된 크기 검증을 통과하지 못하면 런타임 강제 종료 처리합니다.
        # Runtime crash processing if C-Byte array struct fails predefined size verification.

        frame = bytearray(FRAME_LEN)
        # 패킷 완전체 전송을 위해 통신 스펙인 44바이트 배열 메모리 블록을 사전 할당합니다.
        # Pre-allocates 44-byte array memory block comm spec for complete packet transmission.
        frame[0] = SOF_1
        # 시작 프레임 마커(SOF) 1을 패킷 버퍼 초기 주소 0에 입력 할당합니다.
        # Inputs Start of Frame (SOF) marker 1 at packet buffer initial address 0.
        frame[1] = SOF_2
        # 시작 프레임 마커(SOF) 2를 패킷 버퍼 주소 1에 입력 할당합니다.
        # Inputs Start of Frame (SOF) marker 2 at packet buffer address 1.
        frame[2] = PAYLOAD_LEN
        # 바디 페이로드 39의 길이 지표를 패킷 버퍼 주소 2에 입력 할당합니다.
        # Inputs body payload length indicator 39 at packet buffer address 2.
        frame[3:3 + PAYLOAD_LEN] = payload
        # 3번 인덱스 메모리부터 페이로드 길이 끝단까지 구조체 바이너리 데이터를 일괄 덮어씁니다.
        # Batch-overwrites struct binary data from index 3 memory to payload length end.

        crc = crc16_ccitt(bytes(frame[2:FRAME_LEN - 2]))
        # 순환 중복 검사 로직(CRC)을 페이로드 및 길이 바이트 블록 단위로 계산합니다.
        # Calculates Cyclic Redundancy Check (CRC) logic by payload and length byte block units.
        frame[FRAME_LEN - 2] = crc & 0xFF
        # 분리된 16비트 검증 스칼라의 우측 8비트 성분을 배열의 끝에서 2번째 슬롯에 입력합니다.
        # Inputs right 8-bit component of split 16-bit verification scalar into 2nd-to-last slot of array.
        frame[FRAME_LEN - 1] = (crc >> 8) & 0xFF
        # 시프트 연산 처리된 검증 스칼라의 좌측 8비트 성분을 최종 배열 인덱스에 입력 마감합니다.
        # Inputs left 8-bit component of shift-operated verification scalar into final array index closing it.

        msg = UInt8MultiArray()
        # ROS 2 토픽 발행망을 사용하기 위한 데이터 컨테이너 객체의 인스턴스를 소환합니다.
        # Summons instance of data container object to use ROS 2 topic publishing network.
        msg.data = list(frame)
        # 바이트 어레이의 원시 프레임을 파이썬 표준 리스트로 캐스팅하여 데이터 객체에 입력합니다.
        # Casts raw frame of bytearray into python standard list and inputs to data object.

        if not drop:
            # 의도적 통신 중단 플래그가 거짓으로 확인된 논리 상태 시에만 패킷 토픽을 퍼블리싱합니다.
            # Publishes packet topic only during logical state confirming intentional comm drop flag is false.
            self.pub.publish(msg)

        self.tick_count += 1
        # 모든 시퀀스가 결함 없이 루프를 완료 시 전체 누적 사이클 연산 카운트를 1 증가시킵니다.
        # Increments total accumulated cycle calc count by 1 upon completing loop without defects in all sequences.


def main():
    # ROS 2 프레임워크 런타임 초기화 과정 및 노드 라이프사이클 관리를 수행하는 엔트리 포인트입니다.
    # Entry point executing ROS 2 framework runtime initialization and node lifecycle management.
    rclpy.init()
    # 데몬 엔진을 부팅하여 노드 간의 메시징 네트워크를 개방합니다.
    # Boots daemon engine to open messaging network between nodes.
    node = ElrsScenarioDriverNode()
    # 정의된 스크립트 모듈 클래스의 메모리 객체 인스턴스화 작업을 실행합니다.
    # Executes memory object instantiation operation of defined script module class.
    try:
        rclpy.spin(node)
        # 스레드 차단 형태의 이벤트 감지 루프(Event Loop)를 활성화하여 콜백 처리를 무한 유지합니다.
        # Activates thread-blocking style event detection loop to maintain callback processing infinitely.
    except KeyboardInterrupt:
        pass
        # 시스템 시그널(Ctrl+C) 감지 시 프로그램 크래시를 방지하기 위해 예외 패스를 트리거합니다.
        # Triggers exception pass to prevent program crash upon detecting system signal (Ctrl+C).
    node.destroy_node()
    # 메모리 누수 방지 차원에서 할당된 노드 객체 인스턴스를 강제 폐기 소멸합니다.
    # Forcefully discards and destroys allocated node object instance to prevent memory leaks.
    rclpy.shutdown()
    # 부팅된 프로세스의 데몬 엔진 연결을 최종 셧다운 파괴하며 스크립트 사이클을 종료합니다.
    # Finally destroys shutdown daemon engine connection of booted process and ends script cycle.


if __name__ == "__main__":
    main()
    # 임포트가 아닌 시스템 CLI 단독 실행 명령어 입력 시 엔트리 포인트를 활성화하는 예약 조건문입니다.
    # Reserved conditional activating entry point upon inputting standalone system CLI execution command not import.