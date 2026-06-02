#!/usr/bin/env python3
"""
train_rl.py
===========
PPO (Stable-Baselines3) 훈련 스크립트
AWS RoboMaker Bookstore 맵 / 메카넘 RC카

실행:
    python3 train_rl.py [--timesteps 2000000] [--resume]
"""

import argparse
import os
import time
import threading

import numpy as np
import rclpy
from rclpy.executors import MultiThreadedExecutor

from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import (
    CheckpointCallback,
    EvalCallback,
    CallbackList,
)
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.vec_env import DummyVecEnv, VecNormalize
from stable_baselines3.common.torch_layers import BaseFeaturesExtractor

import torch
import torch.nn as nn
import gymnasium as gym
from gymnasium import spaces

from .gazebo_room_env import GazeboRoomEnv, RobotSensorNode

# ──────────────────────────────────────────────────────────────
# 경로 설정
# ──────────────────────────────────────────────────────────────
BASE_DIR     = os.path.expanduser('~/rl_training')
LOG_DIR      = os.path.join(BASE_DIR, 'logs')
MODEL_DIR    = os.path.join(BASE_DIR, 'models')
BEST_DIR     = os.path.join(BASE_DIR, 'best_model')
VECNORM_PATH = os.path.join(BASE_DIR, 'vec_normalize.pkl')

os.makedirs(LOG_DIR,   exist_ok=True)
os.makedirs(MODEL_DIR, exist_ok=True)
os.makedirs(BEST_DIR,  exist_ok=True)


# ──────────────────────────────────────────────────────────────
# 커스텀 특징 추출기 (Dict Obs → PPO 입력)
# ──────────────────────────────────────────────────────────────
class BookstoreFeatureExtractor(BaseFeaturesExtractor):
    """
    Dict 관측값을 처리하는 커스텀 네트워크.

    ┌─ LiDAR (360) ──→ 1D-Conv × 3 ──→ Flatten → 256
    │
    ├─ goal (3) ──────────────────────────────────→ 32
    │                                                  } → Cat → 320 → out_dim
    └─ velocity (3) ──────────────────────────────→ 32

    YOLO obs 추가 시: 새 브랜치 추가 후 out_features 조정만 하면 됨.
    """

    def __init__(self, observation_space: spaces.Dict, features_dim: int = 320):
        super().__init__(observation_space, features_dim)

        lidar_dim = observation_space['lidar'].shape[0]   # 360

        # LiDAR 처리: 1D CNN (국소 장애물 패턴 추출)
        self.lidar_cnn = nn.Sequential(
            nn.Conv1d(1, 32, kernel_size=11, stride=2, padding=5),
            nn.ReLU(),
            nn.Conv1d(32, 64, kernel_size=7, stride=2, padding=3),
            nn.ReLU(),
            nn.Conv1d(64, 128, kernel_size=5, stride=2, padding=2),
            nn.ReLU(),
            nn.Flatten(),
        )
        # CNN 출력 차원 계산
        with torch.no_grad():
            dummy = torch.zeros(1, 1, lidar_dim)
            cnn_out = self.lidar_cnn(dummy).shape[1]

        self.lidar_fc = nn.Sequential(
            nn.Linear(cnn_out, 256),
            nn.ReLU(),
        )

        # 목적지 벡터 처리
        self.goal_fc = nn.Sequential(
            nn.Linear(3, 32),
            nn.ReLU(),
        )

        # 속도 벡터 처리
        self.vel_fc = nn.Sequential(
            nn.Linear(3, 32),
            nn.ReLU(),
        )

        # 최종 결합 차원 확인
        total = 256 + 32 + 32
        assert total == features_dim, (
            f'features_dim 불일치: {total} != {features_dim}. '
            f'features_dim={total} 으로 맞춰주세요.'
        )

    def forward(self, obs: dict) -> torch.Tensor:
        lidar = obs['lidar'].unsqueeze(1)           # (B, 1, 360)
        lidar_feat = self.lidar_fc(self.lidar_cnn(lidar))
        goal_feat  = self.goal_fc(obs['goal'])
        vel_feat   = self.vel_fc(obs['velocity'])
        return torch.cat([lidar_feat, goal_feat, vel_feat], dim=1)


# ──────────────────────────────────────────────────────────────
# PPO 하이퍼파라미터
# ──────────────────────────────────────────────────────────────
PPO_KWARGS = dict(
    policy              = 'MultiInputPolicy',
    learning_rate       = 3e-4,
    n_steps             = 2048,
    batch_size          = 256,
    n_epochs            = 10,
    gamma               = 0.99,
    gae_lambda          = 0.95,
    clip_range          = 0.2,
    ent_coef            = 0.01,       # 탐험 장려
    vf_coef             = 0.5,
    max_grad_norm       = 0.5,
    use_sde             = True,       # 에피소드 내 일관된 방향성 탐험 (덜덜떨기 방지)
    sde_sample_freq     = 64,         # 64스텝마다 노이즈 갱신
    tensorboard_log     = LOG_DIR,
    verbose             = 1,
    policy_kwargs       = dict(
        features_extractor_class  = BookstoreFeatureExtractor,
        features_extractor_kwargs = dict(features_dim=320),
        net_arch                  = dict(
            pi=[256, 256],   # Actor
            vf=[256, 256],   # Critic
        ),
        activation_fn  = nn.ReLU,
        log_std_init   = 0.0,         # 초기 탐험 std=1.0 → 큰 액션부터 시도
    ),
)


# ──────────────────────────────────────────────────────────────
# 에피소드 로깅 콜백
# ──────────────────────────────────────────────────────────────
from stable_baselines3.common.callbacks import BaseCallback

class EpisodeLogCallback(BaseCallback):
    """에피소드별 성공/충돌/타임아웃 통계 출력."""

    def __init__(self, verbose=0):
        super().__init__(verbose)
        self.episode_rewards: list = []
        self.results: dict = {'success': 0, 'collision': 0, 'timeout': 0}

    def _on_step(self) -> bool:
        infos = self.locals.get('infos', [])
        for info in infos:
            result = info.get('result')
            if result in self.results:
                self.results[result] += 1

            ep_info = info.get('episode')
            if ep_info is not None:
                r = ep_info['r']
                self.episode_rewards.append(r)
                total_ep = sum(self.results.values())
                if total_ep % 10 == 0 and total_ep > 0:
                    print(
                        f"[Episode {total_ep:4d}] "
                        f"reward={r:7.1f} | "
                        f"success={self.results['success']} "
                        f"collision={self.results['collision']} "
                        f"timeout={self.results['timeout']}"
                    )
        return True


# ──────────────────────────────────────────────────────────────
# 환경 팩토리
# ──────────────────────────────────────────────────────────────
def make_env(node: RobotSensorNode, phase: int = 1):
    """Monitor 감싸기로 에피소드 통계 자동 기록."""
    def _init():
        env = GazeboRoomEnv(node=node, phase=phase)
        env = Monitor(env, filename=os.path.join(LOG_DIR, f'monitor_p{phase}'))
        return env
    return _init


# ──────────────────────────────────────────────────────────────
# 메인
# ──────────────────────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--timesteps', type=int, default=2_000_000,
                        help='총 학습 타임스텝 수 (기본 2M)')
    parser.add_argument('--resume', action='store_true',
                        help='마지막 저장 모델부터 이어서 학습')
    parser.add_argument('--checkpoint', type=str, default=None,
                        help='재개할 체크포인트 경로 (.zip)')
    parser.add_argument('--phase', type=int, default=1, choices=[1, 2],
                        help='학습 페이즈: 1=A+C 기초(기본), 2=A+B 심화')
    args = parser.parse_args()

    phase = args.phase

    # Phase 2는 반드시 Phase 1 체크포인트에서 시작 (자동 resume)
    if phase == 2 and not args.resume:
        print('[train_rl] Phase 2: Phase 1 체크포인트에서 자동 재개')
        args.resume = True

    # ── ROS2 초기화 ────────────────────────────────────────
    rclpy.init()
    sensor_node = RobotSensorNode()

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(sensor_node)

    # 별도 스레드에서 ROS2 스핀 (Gym 루프와 분리)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    # 센서 데이터 수신 대기
    print('[train_rl] 센서 데이터 수신 대기 중...')
    timeout_count = 0
    while not sensor_node.is_ready():
        time.sleep(0.5)
        timeout_count += 1
        if timeout_count > 20:
            print('[train_rl] 경고: 센서 타임아웃 — 로봇/시뮬레이터 상태를 확인하세요.')
            break
    print('[train_rl] 센서 준비 완료.')

    # ── 환경 구성 ──────────────────────────────────────────
    env = DummyVecEnv([make_env(sensor_node, phase=phase)])

    vecnorm_path = os.path.join(BASE_DIR, f'vec_normalize_p{phase}.pkl')

    # VecNormalize: reward 정규화
    # Phase 2는 보상 분포가 달라지므로 항상 새로 생성 (Phase 1 통계 재사용 안 함)
    if args.resume and phase == 1 and os.path.exists(vecnorm_path):
        print(f'[train_rl] VecNormalize 로드: {vecnorm_path}')
        env = VecNormalize.load(vecnorm_path, env)
        env.training    = True
        env.norm_reward = True
    else:
        if phase == 2:
            print('[train_rl] Phase 2: VecNormalize 새로 생성 (보상 분포 변경)')
        env = VecNormalize(env, norm_obs=False, norm_reward=True, clip_reward=10.0)

    # ── 모델 생성 / 로드 ───────────────────────────────────
    if args.resume:
        ckpt = args.checkpoint
        if ckpt is None:
            # 페이즈별 체크포인트 우선 탐색, 없으면 전체에서 최신
            prefix = f'ppo_bookstore_p{phase}'
            phase_files = sorted([
                f for f in os.listdir(MODEL_DIR)
                if f.startswith(prefix) and f.endswith('.zip')
            ])
            if phase_files:
                ckpt = os.path.join(MODEL_DIR, phase_files[-1])
            else:
                # Phase 2 최초 시작: Phase 1 체크포인트 탐색
                all_files = sorted([
                    f for f in os.listdir(MODEL_DIR) if f.endswith('.zip')
                ])
                if all_files:
                    ckpt = os.path.join(MODEL_DIR, all_files[-1])

        if ckpt and os.path.exists(ckpt):
            print(f'[train_rl] 체크포인트 로드: {ckpt}')
            model = PPO.load(ckpt, env=env, **{
                k: v for k, v in PPO_KWARGS.items()
                if k not in ('policy', 'tensorboard_log', 'verbose', 'policy_kwargs')
            })
        else:
            print('[train_rl] 체크포인트 없음 — 새 모델 생성')
            model = PPO(env=env, **PPO_KWARGS)
    else:
        model = PPO(env=env, **PPO_KWARGS)

    # ── 콜백 설정 ──────────────────────────────────────────
    checkpoint_cb = CheckpointCallback(
        save_freq         = 20_000,
        save_path         = MODEL_DIR,
        name_prefix       = f'ppo_bookstore_p{phase}',
        save_vecnormalize = True,
    )

    best_dir_phase = os.path.join(BASE_DIR, f'best_model_p{phase}')
    os.makedirs(best_dir_phase, exist_ok=True)

    eval_vec_env = DummyVecEnv([make_env(sensor_node, phase=phase)])
    eval_vec_env = VecNormalize(eval_vec_env, norm_obs=False, norm_reward=True,
                                clip_reward=10.0, training=False)

    eval_cb = EvalCallback(
        eval_env             = eval_vec_env,
        best_model_save_path = best_dir_phase,
        log_path             = LOG_DIR,
        eval_freq            = 50_000,
        n_eval_episodes      = 5,
        deterministic        = True,
        render               = False,
        warn                 = False,
    )

    episode_log_cb = EpisodeLogCallback()

    callback_list = CallbackList([
        checkpoint_cb,
        eval_cb,
        episode_log_cb,
    ])

    # ── 학습 시작 ──────────────────────────────────────────
    phase_desc = 'A+C 기초 (vy 비활성)' if phase == 1 else 'A+B 심화 (vy 페널티)'
    print(f'[train_rl] PPO 학습 시작 — Phase {phase} ({phase_desc})')
    print(f'           총 스텝: {args.timesteps:,}')
    print(f'           모델 저장: {MODEL_DIR}')
    print(f'           로그:      {LOG_DIR}')
    print()

    try:
        model.learn(
            total_timesteps     = args.timesteps,
            callback            = callback_list,
            reset_num_timesteps = not args.resume,
            tb_log_name         = f'PPO_bookstore_p{phase}',
            progress_bar        = True,
        )
    except KeyboardInterrupt:
        print('\n[train_rl] 학습 중단 — 모델 저장 중...')

    # ── 최종 저장 ──────────────────────────────────────────
    final_path = os.path.join(MODEL_DIR, f'ppo_bookstore_p{phase}_final')
    model.save(final_path)
    env.save(vecnorm_path)
    print(f'[train_rl] 최종 모델 저장: {final_path}.zip')
    print(f'[train_rl] VecNormalize 저장: {vecnorm_path}')

    # ── 정리 ───────────────────────────────────────────────
    sensor_node.stop_robot()
    env.close()
    executor.shutdown()
    rclpy.shutdown()
    print('[train_rl] 완료.')


if __name__ == '__main__':
    main()
