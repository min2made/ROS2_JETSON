# RC-Car RL Training Guide

## 🚀 Quick Start

### 1. First-time Setup
```bash
cd /home/work/project/ROS2_JETSON

# Build the project
make build

# Install RL dependencies
pip install -r requirements-rl.txt
```

### 2. Start Training

**Terminal 1: Launch Gazebo Simulation**
```bash
make sim
```

**Terminal 2: Start RL Training**
```bash
make rl-train          # Fresh training
# or
make rl-train-resume   # Resume from checkpoint
```

**Terminal 3: Monitor Training (Optional)**
```bash
make rl-monitor
```

Then open:
- Web Dashboard: http://localhost:8765
- TensorBoard: http://localhost:6006

### 3. Interactive Menu
```bash
make rl-ui   # Full menu with all options
```

---

## 📚 File Organization

```
src/rccar_rl/                    # New RL package
├── rccar_rl/
│   ├── gazebo_room_env.py      # Gymnasium environment
│   ├── train_rl.py             # PPO training script
│   └── monitor_training.py     # Monitoring dashboard
└── launch/

~/rl_training/                   # Training outputs (auto-created)
├── logs/                        # TensorBoard logs + monitor.csv
├── models/                      # Checkpoints (every 20k steps)
└── best_model/                  # Best eval model
```

---

## 🎮 Training Commands

### Start Fresh Training
```bash
python3 -m rccar_rl.train_rl --timesteps 2000000
```

### Resume Training
```bash
python3 -m rccar_rl.train_rl --resume --timesteps 3000000
```

### Monitor Only (No Training)
```bash
python3 -m rccar_rl.monitor_training --port 8765
```

---

## 📊 Monitoring Dashboard

The monitoring dashboard provides:
- **Latest Episode Stats**: Reward, length, timestep
- **Recent 10-Episode Average**: Average performance
- **Model History**: Recently saved models with size/timestamp
- **System Info**: Log directory, last update time

Access: http://localhost:8765

---

## 🔧 Customization

### Adjust Training Hyperparameters
Edit `src/rccar_rl/rccar_rl/train_rl.py`:

```python
PPO_KWARGS = dict(
    ent_coef = 0.005,      # Higher = more exploration
    learning_rate = 3e-4,  # Lower = more stable
    n_steps = 2048,        # Larger = more samples per update
    ...
)
```

### Modify Reward Function
Edit `src/rccar_rl/rccar_rl/gazebo_room_env.py` in `GazeboRoomEnv._compute_reward()`:

- `R_GOAL`: Reward for reaching goal (+100)
- `R_COLLISION`: Penalty for collision (-100)
- `R_HEADING_SCALE`: Heading alignment reward (0.3)
- `R_CENTER_SCALE`: Path centering reward (0.1)

### Add New Observation (e.g., YOLO)
The observation space is Dict-based for easy extension:

```python
self.observation_space = spaces.Dict({
    'lidar': spaces.Box(...),      # Existing
    'goal': spaces.Box(...),       # Existing
    'velocity': spaces.Box(...),   # Existing
    'yolo': spaces.Box(...),       # New: Add here
})
```

Then add a branch in `BookstoreFeatureExtractor`:
```python
self.yolo_fc = nn.Sequential(
    nn.Linear(yolo_dim, 32),
    nn.ReLU(),
)
```

---

## 💡 Tips & Troubleshooting

### Training Starts Slow (High Collision Rate)
- Expected in first ~200k steps. The agent learns corridor structure.
- Solution: Keep `ent_coef=0.005` for exploration.

### Model Not Converging
1. Check reward scaling in `_compute_reward()`
2. Try lower learning rate: `learning_rate=1e-4`
3. Increase n_steps: `n_steps=4096`

### GPU Not Used
```bash
python3 -c "import torch; print(torch.cuda.is_available())"
```

If False, install GPU PyTorch:
```bash
pip install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu118
```

### Gazebo Not Found
```bash
make setup    # Install dependencies
make build    # Rebuild packages
```

---

## 📈 Performance Expectations

| Stage | Timesteps | Success Rate | Notes |
|-------|-----------|--------------|-------|
| Exploration | 0-200k | 0-30% | Learning corridor navigation |
| Learning | 200k-1M | 30-70% | Policy refinement |
| Convergence | 1M-2M | 70-90%+ | Fine-tuning |

---

## 🛑 Stopping Training

- **Ctrl+C**: Graceful shutdown (saves current model + VecNormalize)
- **Emergency Stop**: Kill the process (latest checkpoint from `n_steps` saved)

---

## 📖 Architecture Overview

### GazeboRoomEnv (Gymnasium)
- **Observation**: LiDAR(360) + Goal(3) + Velocity(3)
- **Action**: [vx, vy, wz] continuous control
- **Reward**: 8-component reward function
- **Safe Spawning**: 5 safe zones for random episode starts

### RobotSensorNode (ROS2)
- **ReentrantCallbackGroup**: LiDAR/Odom callbacks (concurrent)
- **MutuallyExclusiveCallbackGroup**: Gazebo services (no race)
- **Thread-safe Data**: Shared scan/pose/velocity via locks

### BookstoreFeatureExtractor (CNN)
- **LiDAR**: 1D-CNN (11→7→5 kernels) → 256D features
- **Goal**: FC(3→32) features
- **Velocity**: FC(3→32) features
- **Output**: 320D concatenated features → PPO

---

## 🔗 Related Commands

```bash
make list-packages    # Show all packages
make clean            # Clean build/install
make info             # Workspace info
```
