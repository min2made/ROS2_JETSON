#!/usr/bin/env python3
"""
gazebo_room_env.py
==================
ROS2 Humble + Gazebo  ←→  Gymnasium 커스텀 환경
AWS RoboMaker Bookstore 맵 전용 PPO 강화학습 환경

로봇  : 메카넘 휠 RC카 (Planar Move 플러그인)
센서  : 360° 2D LiDAR (/scan), Odometry (/odom)
제어  : /cmd_vel (vx, vy, wz)
"""

import math
import time
import threading
from typing import Optional, Tuple, Dict, Any

import numpy as np
import gymnasium as gym
from gymnasium import spaces

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from gazebo_msgs.srv import SetEntityState
from gazebo_msgs.msg import EntityState
from std_srvs.srv import Empty
from geometry_msgs.msg import Pose, Point, Quaternion

# ──────────────────────────────────────────────────────────────
# 상수 정의
# ──────────────────────────────────────────────────────────────
LIDAR_DIM       = 360
LIDAR_MIN       = 0.12
LIDAR_MAX       = 12.0

COLLISION_DIST  = 0.35   # [m] 충돌 판정 거리 (로봇 실제 반경 + 여유)
GOAL_RADIUS     = 0.40   # [m] 목적지 도달 반경
STEP_DT         = 0.1    # [s] 타임스텝 주기
MAX_STEPS       = 1000   # 에피소드 최대 스텝

# 보상 상수
R_GOAL          =  100.0
R_COLLISION     = -100.0
R_TIMEOUT       =   -0.1
R_HEADING_SCALE =    0.3
R_CENTER_SCALE  =    0.1
R_SPIN_PENALTY  =   -0.2
DIAGONAL_PENALTY =   0.05  # Phase 2: |vy| 단위당 스텝 페널티 (메카넘 에너지 비용)

# 속도 클리핑
VX_MAX = 0.4
VY_MAX = 0.4
WZ_MAX = 0.8
MAX_LINEAR = VX_MAX  # Method A: 방향 무관 최대 선속도 (대각선 이점 제거)

# AWS Bookstore 안전 스폰 영역 (x_min, x_max, y_min, y_max)
# 맵 중앙 통로 및 개방 구역 위주로 수동 설정
SAFE_ZONES = [
    (-1.5,  1.5, -3.0,  3.0),   # 중앙 메인 통로
    ( 2.0,  4.0, -2.0,  2.0),   # 우측 개방구역
    (-4.0, -2.0, -2.0,  2.0),   # 좌측 개방구역
    (-1.0,  1.0,  3.0,  5.0),   # 상단 공간
    (-1.0,  1.0, -5.0, -3.0),   # 하단 공간
]


# ──────────────────────────────────────────────────────────────
# ROS2 센서 노드
# ──────────────────────────────────────────────────────────────
class RobotSensorNode(Node):
    """
    센서 구독 + 제어 퍼블리시 전담 ROS2 노드.
    Gym 환경과 스레드 안전하게 데이터 공유.
    """

    def __init__(self):
        super().__init__('rl_sensor_node')

        # use_sim_time 강제 설정
        self.set_parameters([
            rclpy.parameter.Parameter(
                'use_sim_time',
                rclpy.Parameter.Type.BOOL,
                True
            )
        ])

        # QoS 프로파일
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.VOLATILE,
        )
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # 콜백 그룹 (센서 / 서비스 분리)
        self.sensor_cb_group  = ReentrantCallbackGroup()
        self.service_cb_group = MutuallyExclusiveCallbackGroup()

        # ── 구독자 ──────────────────────────────────────────
        self.sub_scan = self.create_subscription(
            LaserScan, '/scan',
            self._cb_scan, sensor_qos,
            callback_group=self.sensor_cb_group,
        )
        self.sub_odom = self.create_subscription(
            Odometry, '/odom',
            self._cb_odom, sensor_qos,
            callback_group=self.sensor_cb_group,
        )

        # ── 퍼블리셔 ────────────────────────────────────────
        self.pub_cmd = self.create_publisher(
            Twist, '/cmd_vel', reliable_qos
        )

        # ── 서비스 클라이언트 ───────────────────────────────
        self.cli_set_entity = self.create_client(
            SetEntityState, '/gazebo/set_entity_state',
            callback_group=self.service_cb_group,
        )

        # ── 공유 상태 (스레드 락 보호) ──────────────────────
        self._lock       = threading.Lock()
        self._scan_data  = np.full(LIDAR_DIM, LIDAR_MAX, dtype=np.float32)
        self._pos_x      = 0.0
        self._pos_y      = 0.0
        self._yaw        = 0.0
        self._vel_x      = 0.0
        self._vel_y      = 0.0
        self._vel_wz     = 0.0
        self._scan_ready = False
        self._odom_ready = False
        self._scan_seq   = 0   # 스캔 수신 카운터 (신선도 확인용)

    # ── 콜백 ──────────────────────────────────────────────────
    def _cb_scan(self, msg: LaserScan):
        ranges = np.array(msg.ranges, dtype=np.float32)
        ranges = np.where(np.isfinite(ranges), ranges, LIDAR_MAX)
        ranges = np.clip(ranges, LIDAR_MIN, LIDAR_MAX)
        # 360개로 맞추기 (센서 해상도 차이 보정)
        if len(ranges) != LIDAR_DIM:
            indices = np.linspace(0, len(ranges) - 1, LIDAR_DIM).astype(int)
            ranges  = ranges[indices]
        with self._lock:
            self._scan_data  = ranges
            self._scan_ready = True
            self._scan_seq  += 1

    def _cb_odom(self, msg: Odometry):
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        vel = msg.twist.twist
        # 쿼터니언 → yaw
        siny = 2.0 * (ori.w * ori.z + ori.x * ori.y)
        cosy = 1.0 - 2.0 * (ori.y * ori.y + ori.z * ori.z)
        yaw  = math.atan2(siny, cosy)
        with self._lock:
            self._pos_x      = pos.x
            self._pos_y      = pos.y
            self._yaw        = yaw
            self._vel_x      = vel.linear.x
            self._vel_y      = vel.linear.y
            self._vel_wz     = vel.angular.z
            self._odom_ready = True

    # ── 외부 접근자 ───────────────────────────────────────────
    def get_scan(self) -> np.ndarray:
        with self._lock:
            return self._scan_data.copy()

    def get_scan_seq(self) -> int:
        with self._lock:
            return self._scan_seq

    def wait_fresh_scan(self, timeout: float = 1.0):
        """텔포 후 새 스캔이 도착할 때까지 대기."""
        seq_before = self.get_scan_seq()
        deadline = time.time() + timeout
        while time.time() < deadline:
            if self.get_scan_seq() != seq_before:
                return
            time.sleep(0.02)
        self.get_logger().warn('wait_fresh_scan: 타임아웃 — stale 스캔 사용 가능성')

    def get_pose(self) -> Tuple[float, float, float]:
        with self._lock:
            return self._pos_x, self._pos_y, self._yaw

    def get_velocity(self) -> Tuple[float, float, float]:
        with self._lock:
            return self._vel_x, self._vel_y, self._vel_wz

    def is_ready(self) -> bool:
        with self._lock:
            return self._scan_ready and self._odom_ready

    # ── 제어 ──────────────────────────────────────────────────
    def publish_cmd(self, vx: float, vy: float, wz: float):
        msg = Twist()
        msg.linear.x  = float(np.clip(vx, -VX_MAX, VX_MAX))
        msg.linear.y  = float(np.clip(vy, -VY_MAX, VY_MAX))
        msg.angular.z = float(np.clip(wz, -WZ_MAX, WZ_MAX))
        self.pub_cmd.publish(msg)

    def stop_robot(self):
        self.publish_cmd(0.0, 0.0, 0.0)

    # ── Gazebo 서비스 ─────────────────────────────────────────
    def set_robot_pose(self, x: float, y: float, yaw: float, timeout: float = 5.0) -> bool:
        """로봇 위치를 Gazebo 서비스로 즉시 재배치."""
        if not self.cli_set_entity.service_is_ready():
            self.get_logger().warn('set_entity_state 서비스 미준비')
            return False

        q = _yaw_to_quaternion(yaw)
        state = EntityState()
        state.name = 'rccar'
        state.pose = Pose(
            position=Point(x=x, y=y, z=0.05),
            orientation=Quaternion(x=q[0], y=q[1], z=q[2], w=q[3]),
        )
        state.twist = Twist()

        req = SetEntityState.Request()
        req.state = state

        done_event = threading.Event()
        result_holder: list = [None]

        def _done_cb(future):
            try:
                result_holder[0] = future.result()
            except Exception as e:
                self.get_logger().error(f'set_entity_state 오류: {e}')
            done_event.set()

        future = self.cli_set_entity.call_async(req)
        future.add_done_callback(_done_cb)

        if not done_event.wait(timeout=timeout):
            self.get_logger().warn('set_entity_state 응답 타임아웃')
            return False

        res = result_holder[0]
        return res.success if res is not None else False


# ──────────────────────────────────────────────────────────────
# 유틸리티
# ──────────────────────────────────────────────────────────────
def _yaw_to_quaternion(yaw: float) -> Tuple[float, float, float, float]:
    """yaw → (x, y, z, w) 쿼터니언"""
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def _sample_safe_pose(rng: np.random.Generator) -> Tuple[float, float, float]:
    """안전 구역에서 무작위 (x, y, yaw) 샘플링."""
    zone = SAFE_ZONES[rng.integers(0, len(SAFE_ZONES))]
    x   = rng.uniform(zone[0], zone[1])
    y   = rng.uniform(zone[2], zone[3])
    yaw = rng.uniform(-math.pi, math.pi)
    return x, y, yaw


def _goal_obs(robot_x, robot_y, robot_yaw, goal_x, goal_y) -> np.ndarray:
    """
    목적지 관측값 계산.
    [정규화된 거리(0~1), 상대 각도 cos, 상대 각도 sin]
    """
    dx   = goal_x - robot_x
    dy   = goal_y - robot_y
    dist = math.hypot(dx, dy)
    angle_to_goal = math.atan2(dy, dx)
    rel_angle     = angle_to_goal - robot_yaw
    # [-π, π] 정규화
    rel_angle = math.atan2(math.sin(rel_angle), math.cos(rel_angle))
    dist_norm = min(dist / 10.0, 1.0)
    return np.array([dist_norm, math.cos(rel_angle), math.sin(rel_angle)],
                    dtype=np.float32)


# ──────────────────────────────────────────────────────────────
# Gymnasium 커스텀 환경
# ──────────────────────────────────────────────────────────────
class GazeboRoomEnv(gym.Env):
    """
    AWS RoboMaker Bookstore 맵 전용 Gymnasium 환경.

    Observation (Dict):
        lidar    : (360,)  정규화 LiDAR [0, 1]
        goal     : (3,)    [dist_norm, cos(rel_angle), sin(rel_angle)]
        velocity : (3,)    [vx, vy, wz]  — 추후 YOLO obs 확장 가능

    Action (Box, continuous):
        [vx, vy, wz]  각각 [-0.4, 0.4], [-0.4, 0.4], [-0.8, 0.8]
    """

    metadata = {'render_modes': []}

    def __init__(self, node: RobotSensorNode, render_mode=None, phase: int = 1):
        super().__init__()
        self.node  = node
        self.rng   = np.random.default_rng()
        self.phase = phase  # 1=A+C 기초학습, 2=A+B 심화학습

        # ── Observation Space ──────────────────────────────
        # Dict 구조로 설계 → 추후 'yolo' 키 추가만으로 확장 가능
        self.observation_space = spaces.Dict({
            'lidar': spaces.Box(
                low=0.0, high=1.0,
                shape=(LIDAR_DIM,), dtype=np.float32,
            ),
            'goal': spaces.Box(
                low=np.array([0.0, -1.0, -1.0], dtype=np.float32),
                high=np.array([1.0,  1.0,  1.0], dtype=np.float32),
                dtype=np.float32,
            ),
            'velocity': spaces.Box(
                low=np.array([-VX_MAX, -VY_MAX, -WZ_MAX], dtype=np.float32),
                high=np.array([ VX_MAX,  VY_MAX,  WZ_MAX], dtype=np.float32),
                dtype=np.float32,
            ),
            # ── 향후 YOLO 확장 예시 (현재 비활성) ────────────
            # 'yolo': spaces.Box(
            #     low=0.0, high=1.0,
            #     shape=(MAX_DETECTIONS, 6), dtype=np.float32,
            # ),
        })

        # ── Action Space ───────────────────────────────────
        self.action_space = spaces.Box(
            low=np.array([-VX_MAX, -VY_MAX, -WZ_MAX], dtype=np.float32),
            high=np.array([ VX_MAX,  VY_MAX,  WZ_MAX], dtype=np.float32),
            dtype=np.float32,
        )

        # ── 내부 상태 ──────────────────────────────────────
        self._goal_x: float = 0.0
        self._goal_y: float = 0.0
        self._step_count: int = 0
        self._prev_dist: float = 0.0
        self._episode_count: int = 0
        
    # ── 핵심 메서드 ───────────────────────────────────────────
    def reset(
        self,
        *,
        seed: Optional[int] = None,
        options: Optional[Dict] = None,
    ) -> Tuple[Dict[str, np.ndarray], Dict[str, Any]]:

        super().reset(seed=seed)
        if seed is not None:
            self.rng = np.random.default_rng(seed)

        self._episode_count += 1
        self._step_count    = 0
        self.node.stop_robot()

        # ── 스폰 위치 검증 루프 ───────────────────────────
        # 가구/벽 안에 스폰되면 즉시 충돌 판정 → 의미없는 에피소드 반복 방지
        SPAWN_MIN_CLEARANCE = COLLISION_DIST * 2.0   # 0.5m 여유
        MAX_SPAWN_ATTEMPTS  = 8

        for attempt in range(MAX_SPAWN_ATTEMPTS):
            robot_x, robot_y, robot_yaw = _sample_safe_pose(self.rng)
            ok = self.node.set_robot_pose(robot_x, robot_y, robot_yaw)
            if not ok:
                continue
            # 텔포 후 새 스캔이 도착할 때까지 대기 (stale 스캔 방지)
            self.node.wait_fresh_scan(timeout=1.0)
            if np.min(self.node.get_scan()) >= SPAWN_MIN_CLEARANCE:
                break          # 유효한 스폰 위치 확보
            if attempt < MAX_SPAWN_ATTEMPTS - 1:
                self.node.get_logger().warn(
                    f'[reset] 스폰 위치 장애물 충돌, 재시도 {attempt + 1}/{MAX_SPAWN_ATTEMPTS}'
                )

        # ── 목적지 샘플링 (로봇과 최소 2m 이상 떨어진 곳) ─
        for _ in range(50):
            gx, gy, _ = _sample_safe_pose(self.rng)
            if math.hypot(gx - robot_x, gy - robot_y) >= 2.0:
                break
        self._goal_x, self._goal_y = gx, gy

        # 추가 물리 안정화
        time.sleep(0.1)

        # ── 초기 거리 기록 ────────────────────────────────
        obs = self._get_obs()
        self._prev_dist = math.hypot(
            self._goal_x - robot_x,
            self._goal_y - robot_y,
        )

        info = {
            'robot_pos' : (robot_x, robot_y, robot_yaw),
            'goal_pos'  : (self._goal_x, self._goal_y),
            'episode'   : self._episode_count,
        }
        return obs, info

    def step(
        self, action: np.ndarray
    ) -> Tuple[Dict[str, np.ndarray], float, bool, bool, Dict[str, Any]]:

        vx, vy, wz = float(action[0]), float(action[1]), float(action[2])

        # Phase 1 — Method C: 횡 이동(vy) 비활성화
        if self.phase == 1:
            vy = 0.0

        # Method A: 선속도 벡터 정규화 (대각선 이동 시 속도 이점 제거)
        # vx=0.4, vy=0.4 → sqrt(0.32)≈0.566 → 0.4로 클리핑
        linear = math.hypot(vx, vy)
        if linear > MAX_LINEAR:
            scale = MAX_LINEAR / linear
            vx, vy = vx * scale, vy * scale

        self.node.publish_cmd(vx, vy, wz)

        # 제어 주기 대기 (시뮬 타임 동기화)
        time.sleep(STEP_DT)
        self._step_count += 1

        obs      = self._get_obs()
        scan     = self.node.get_scan()
        rx, ry, ryaw = self.node.get_pose()

        # ── 종료 조건 판정 ────────────────────────────────
        dist_to_goal = math.hypot(self._goal_x - rx, self._goal_y - ry)
        min_scan     = float(np.min(scan))

        terminated = False
        truncated  = False
        info: Dict[str, Any] = {
            'dist_to_goal': dist_to_goal,
            'min_scan'    : min_scan,
            'step'        : self._step_count,
        }

        reward = self._compute_reward(
            rx, ry, ryaw, vx, vy, wz,
            scan, dist_to_goal, min_scan, info,
        )

        if info.get('goal_reached', False):
            terminated = True
            info['result'] = 'success'
        elif info.get('collision', False):
            terminated = True
            info['result'] = 'collision'
            self.node.stop_robot()
        elif self._step_count >= MAX_STEPS:
            truncated = True
            info['result'] = 'timeout'
            self.node.stop_robot()

        self._prev_dist = dist_to_goal
        return obs, reward, terminated, truncated, info

    def close(self):
        self.node.stop_robot()

    # ── 관측값 구성 ──────────────────────────────────────────
    def _get_obs(self) -> Dict[str, np.ndarray]:
        scan         = self.node.get_scan()
        rx, ry, ryaw = self.node.get_pose()
        vx, vy, vwz  = self.node.get_velocity()

        # LiDAR 정규화 [0, 1]
        lidar_norm = (scan - LIDAR_MIN) / (LIDAR_MAX - LIDAR_MIN)
        lidar_norm = np.clip(lidar_norm, 0.0, 1.0).astype(np.float32)

        goal_vec  = _goal_obs(rx, ry, ryaw, self._goal_x, self._goal_y)
        vel_vec   = np.array([vx, vy, vwz], dtype=np.float32)

        return {
            'lidar'   : lidar_norm,
            'goal'    : goal_vec,
            'velocity': vel_vec,
        }

    # ── 보상 함수 ─────────────────────────────────────────────
    def _compute_reward(
        self,
        rx: float, ry: float, ryaw: float,
        vx: float, vy: float, wz: float,
        scan: np.ndarray,
        dist_to_goal: float,
        min_scan: float,
        info: Dict,
    ) -> float:

        reward = 0.0

        # 1) 목적지 도달 ──────────────────────────────────────
        if dist_to_goal < GOAL_RADIUS:
            reward += R_GOAL
            info['goal_reached'] = True
            return reward
        info['goal_reached'] = False

        # 2) 충돌 ─────────────────────────────────────────────
        if min_scan < COLLISION_DIST:
            reward += R_COLLISION
            info['collision'] = True
            return reward
        info['collision'] = False

        # 3) 목적지 방향 접근 보상 (거리 감소량 비례) ─────────
        dist_delta = self._prev_dist - dist_to_goal
        reward += dist_delta * 2.0   # 1cm 접근 = +0.02

        # 4) 헤딩 정렬 보상 ───────────────────────────────────
        angle_to_goal = math.atan2(
            self._goal_y - ry, self._goal_x - rx
        )
        rel_heading = math.atan2(
            math.sin(angle_to_goal - ryaw),
            math.cos(angle_to_goal - ryaw),
        )
        # cos: 정면 정렬 시 1, 반대 방향 시 -1
        heading_reward = math.cos(rel_heading) * R_HEADING_SCALE
        reward += heading_reward

        # 5) 통로 중앙 정렬 보상 ──────────────────────────────
        # 좌측(90°) ↔ 우측(270°) 거리 균형
        left_d  = float(scan[90])    # 로봇 왼쪽
        right_d = float(scan[270])   # 로봇 오른쪽

        # 좌우 균형 (통로 중앙에 있을수록 0에 가까움)
        balance = abs(left_d - right_d)
        # 통로 폭이 좁을 때만 보상 (좁은 책장 통로 통과 유도)
        if (left_d + right_d) < 2.0:
            center_reward = max(0.0, R_CENTER_SCALE - balance * 0.1)
            reward += center_reward

        # 6) 제자리 회전 페널티 ───────────────────────────────
        # |wz| 크고 |vx|+|vy| 작을 때 페널티
        linear_speed = math.hypot(vx, vy)
        if abs(wz) > 0.3 and linear_speed < 0.05:
            reward += R_SPIN_PENALTY

        # 7) 장애물 근접 경고 (선형 페널티) ──────────────────
        # 0.25~0.5m 구간: 거리 반비례 페널티
        danger_dist = 0.5
        if min_scan < danger_dist:
            proximity_pen = -0.5 * (danger_dist - min_scan) / danger_dist
            reward += proximity_pen

        # 8) 타임 아웃 페널티 (매 스텝) ─────────────────────
        reward += R_TIMEOUT

        # 9) Phase 2 — Method B: 횡 이동 페널티 ──────────────
        # 실제 메카넘휠은 vy 사용 시 에너지 소비 증가.
        # timeout(-0.1)의 20% 수준으로 시작 → 얼어붙지 않는 선에서 vy 억제.
        if self.phase == 2:
            reward -= DIAGONAL_PENALTY * abs(vy)

        return float(reward)
