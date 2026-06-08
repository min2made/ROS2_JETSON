# ROS2 서빙 로봇 프로젝트

## 프로젝트 목표 (단계별)

1. **현재** — Gazebo 시뮬에서 LIDAR 기반 PPO 강화학습으로 장애물 회피
2. 실제 환경에서도 RL 학습
3. 카메라 SLAM으로 실제 환경 지도 생성 → 지도 기반 RL
4. 바닥 QR코드로 자율주행 보조
5. 미학습 장애물 실시간 회피
6. 멀티 로봇: 가상환경에 여러 서빙 로봇 배치, 테이블별 충돌 없이 이동
7. Phase 2 — LIDAR + YOLOv8 카메라로 사람 우선 회피

## 하드웨어 / 소프트웨어 스택

- **로봇**: 메카넘휠 RC카 (실제 하드웨어 + Jetson)
- **OS**: Ubuntu 22.04, ROS2 Humble, Gazebo Classic 11
- **학습**: Stable Baselines3 PPO + Gymnasium, CPU 학습 (느림 — 스텝당 ~0.1s)
- **시뮬 맵**: AWS RoboMaker Bookstore World

## 주요 명령어

```bash
make sim              # Gazebo + 로봇 스폰
make rl-train         # Phase 1 신규 학습 (A+C, 2M 스텝)
make rl-train-resume  # Phase 1 이어서
make rl-train-phase2  # Phase 2 학습 (A+B, Phase 1 체크포인트에서 자동 재개)
make rl-monitor       # 웹 대시보드(8765) + TensorBoard(6006)
make build            # 전체 패키지 빌드
```

## 학습 파일 경로

```
~/rl_training/
├── models/               # 체크포인트 (20K 스텝마다 자동 저장)
│   ├── ppo_bookstore_p1_*.zip
│   └── ppo_bookstore_p2_*.zip
├── best_model_p1/        # Phase 1 최고 성능 모델
├── best_model_p2/        # Phase 2 최고 성능 모델
├── logs/                 # TensorBoard 로그
│   ├── PPO_bookstore_p1_*/
│   └── monitor_p1.monitor.csv
├── vec_normalize_p1.pkl  # Phase 1 보상 정규화 통계
└── vec_normalize_p2.pkl
```

학습 초기화 시:
```bash
rm -rf ~/rl_training && mkdir -p ~/rl_training/{models,best_model_p1,best_model_p2,logs}
```

## 핵심 파일

| 파일 | 역할 |
|------|------|
| `src/rccar_rl/rccar_rl/gazebo_room_env.py` | Gymnasium 환경 (보상함수, 리셋, 관측) |
| `src/rccar_rl/rccar_rl/train_rl.py` | PPO 학습 스크립트 |
| `src/rccar_rl/rccar_rl/monitor_training.py` | 웹 모니터링 서버 |
| `src/rccar_gazebo/worlds/bookstore.world` | Gazebo 월드 |
| `src/rccar_description/urdf/rccar_sim.urdf.xacro` | 시뮬 URDF |
| `src/rccar_bringup/launch/sim.launch.py` | 시뮬 런치 |

## 학습 설계 (2페이즈)

### Phase 1 — A+C (기초)
- **액션**: `[vx, wz]` — vy=0 강제 (횡이동 비활성)
- **Method A**: 선속도 정규화 `sqrt(vx²+vy²) ≤ 0.4` — 대각선 속도 이점 제거
- **목적**: vx+wz만으로 기본 회피/목표 도달 학습

### Phase 2 — A+B (심화)
- **액션**: `[vx, vy, wz]` — vy 재활성
- **Method B**: `reward -= 0.05 × |vy|` — 횡이동 에너지 비용 학습
- **목적**: 메카넘 특성 이해 후 효율적 경로 탐색

## 보상 함수 설계 원칙

| 항목 | 값 | 비고 |
|------|-----|------|
| 목표 도달 | +100 | |
| 충돌 | -100 | LiDAR min < 0.35m |
| 거리 접근 | +2.0×Δdist | 매 스텝 |
| 헤딩 정렬 | cos(θ)×0.3×(속도/최대속도) | **속도 비례** — 멈추면 0 |
| 제자리 회전 | -0.2 | wz>0.1 AND 속도<0.05 |
| 근접 경고 | 최대 -0.25 | 0.35~0.5m 구간 |
| 타임아웃 | -0.1/스텝 | 매 스텝 |

**헤딩 보상을 속도 비례로 설계한 이유**: 정지 상태에서 목표 방향만 봐도 +0.2/스텝 양수 보상을 받는 reward hacking 방지.

## 알려진 이슈 및 해결책

### Gazebo set_entity_state 서비스 타임아웃
- **원인**: 월드 파일에 `libgazebo_ros_state.so` 플러그인 미설치
- **해결**: `bookstore.world`에 `<plugin name="gazebo_ros_state">` 추가 (완료)
- `rclpy`의 `wait_for_service()`는 executor spin 중 메인 스레드에서 호출 시 충돌. `service_is_ready()` polling으로 대체

### 스폰 위치 즉시 충돌
- **원인**: SAFE_ZONES 수동 정의 — 가구 위치와 불일치
- **해결**: 스폰 후 `wait_fresh_scan()` → min_scan < 0.7m이면 재시도 (최대 8회)
- **향후**: SLAM 지도 기반 스폰 위치 필터링으로 교체 예정

### Eval/Train VecNormalize 타입 불일치
- **원인**: EvalCallback의 eval_env가 VecNormalize 미적용
- **해결**: eval_env도 `VecNormalize(training=False)`로 래핑

## SAFE_ZONES (임시 — SLAM 지도로 교체 예정)

현재 수동 정의된 직사각형 구역. 실제 가구 위치와 정확히 일치하지 않을 수 있음.
향후 SLAM 지도 (PGM 포맷) 생성 후 빈 셀에서만 랜덤 스폰하는 방식으로 교체 계획.

## 관측 공간 구조

```python
{
  'lidar'   : (360,)  # 정규화 LiDAR [0,1]
  'goal'    : (3,)    # [dist_norm, cos(θ), sin(θ)]
  'velocity': (3,)    # [vx, vy, wz] 실제 속도
}
```

## 향후 확장 계획

- YOLO obs 추가: `'yolo': (N, 6)` 키 추가만으로 확장 가능하게 설계됨
- 멀티 로봇: 환경 인스턴스 여러 개 + 각 로봇 별 네임스페이스
- 실제 환경 전이: SLAM 지도 + 동일 관측 공간 유지
