# RC-Car Serving Robot - ROS2 Project

**Advanced Autonomous Serving Robot with Simulation, Reinforcement Learning, and Vision**

## 📋 Project Overview

This is a comprehensive ROS2 project for an autonomous serving robot with:
- **Mecanum wheel drive** (omnidirectional movement)
- **Multi-directional cameras** (front, rear, left, right)
- **Gazebo simulation** with realistic physics
- **Gamepad control** (Xbox/Wireless controllers)
- **Hardware integration** ready for real robot deployment
- **RL training** framework with sim-to-real transfer

## 📁 Directory Structure

```
ROS2_JETSON/
├── src/
│   ├── rccar_description/          # URDF and mesh files
│   │   ├── urdf/
│   │   │   ├── rccar.urdf.xacro    # Main robot description
│   │   │   └── rccar_sim.urdf.xacro # Simulation extensions
│   │   └── meshes/
│   │
│   ├── rccar_gazebo/               # Gazebo simulation
│   │   ├── launch/
│   │   │   └── gazebo.launch.py    # Gazebo server/client
│   │   └── worlds/
│   │       └── restaurant.world    # Restaurant environment
│   │
│   ├── rccar_control/              # Control nodes
│   │   ├── rccar_control/
│   │   │   ├── gamepad_node.py     # Joystick → cmd_vel
│   │   │   └── mecanum_controller.py
│   │   └── launch/
│   │       └── control.launch.py
│   │
│   └── rccar_bringup/              # Unified bringup
│       └── launch/
│           ├── sim.launch.py       # Full simulation
│           └── real.launch.py      # Real hardware
│
├── Makefile                        # Menu interface (make help)
└── README.md                       # This file
```

## 🚀 Quick Start

### 1. **First Time Setup**
```bash
cd /home/work/project/ROS2_JETSON
make setup      # Install dependencies
make build      # Compile all packages
```

### 2. **Run Gazebo Simulation**
```bash
make sim        # Launch Gazebo with robot
```

### 3. **Test Gamepad Control**
```bash
# In another terminal:
make gamepad-control    # Launch joy_node + gamepad controller
```

## 📦 Available Make Commands

Run `make help` to see all commands:

| Command | Description |
|---------|-------------|
| `make setup` | Install ROS2 dependencies |
| `make build` | Build all packages (colcon) |
| `make source` | Source setup.bash |
| `make clean` | Remove build/install dirs |
| `make sim` | Launch Gazebo simulation |
| `make sim-gamepad` | Sim + gamepad control |
| `make joy-test` | Test joystick input |
| `make gamepad-control` | Launch gamepad controller |
| `make check-deps` | Verify dependencies |
| `make list-packages` | List all packages |
| `make info` | Show workspace info |

## 🎮 Gamepad Controls

**Xbox / Wireless Controller Layout:**

```
         Y (X)
         /  \
    LB /    \ RB
    [  \   /  ]  
    [   \ /    ]
  LT[ \   /  ]RT
    [  [LStick]  ]
    [      |      ]
    [  [RStick]   ]
    [      |      ]
    [   [DPad]    ]
```

| Input | Action |
|-------|--------|
| **Left Stick (X,Y)** | Linear motion (forward/back, strafe left/right) |
| **Right Stick (X)** | Rotation (turn left/right) |
| **LT + RT** | Fine control (reduce speed by 50%) |
| **A Button** | Reserved for future |
| **B Button** | Reserved for future |

## 🎯 Next Steps

### Phase 1: Validation (Current)
- [x] URDF completed
- [x] ROS2 package structure
- [x] Gazebo integration
- [x] Gamepad control framework
- [ ] **NEXT:** Test simulation with gamepad

### Phase 2: Hardware Integration
- [ ] Motor driver ROS2 node
- [ ] Sensor integration (IMU, encoders)
- [ ] Real robot control testing

### Phase 3: Reinforcement Learning
- [ ] Gym/Stable-baselines3 environment
- [ ] Sim training pipeline
- [ ] Real world training

### Phase 4: Advanced Features
- [ ] Multi-robot simulation
- [ ] Camera-based SLAM/mapping
- [ ] QR code navigation
- [ ] Dynamic obstacle avoidance
- [ ] Nav2 integration

## 📊 Robot Specifications

| Feature | Value |
|---------|-------|
| **Platform** | Mecanum wheels (omnidirectional) |
| **Size** | 400mm (L) × 300mm (W) × 60mm (H) |
| **Wheels** | 80mm diameter, 4× mecanum |
| **Cameras** | 4× RGB (front, rear, left, right) |
| **Sensors** | IMU, wheel encoders (via Gazebo) |
| **Max Speed** | 1.0 m/s (configurable) |
| **Control** | Xbox/Wireless gamepad via joy_node |

## 🛠️ Package Dependencies

**Core ROS2:**
- `rclpy`, `rclcpp`
- `std_msgs`, `geometry_msgs`, `sensor_msgs`

**Simulation:**
- `gazebo-ros2-control`
- `gazebo-plugins`
- `robot-state-publisher`

**Control:**
- `joy` (joystick input)
- `xacro` (URDF processing)

## 📝 Configuration Files

### Launch Files
- **sim.launch.py** - Full simulation (Gazebo + robot + control)
- **gazebo.launch.py** - Gazebo only
- **control.launch.py** - Gamepad control nodes

### URDF/Xacro
- **rccar.urdf.xacro** - Robot model (geometry, physics)
- **rccar_sim.urdf.xacro** - Simulation plugins (cameras, sensors)

### Gazebo World
- **restaurant.world** - Restaurant environment with tables/walls

## 🔧 Customization

### Change Controller Limits
Edit `src/rccar_control/rccar_control/gamepad_node.py`:
```python
self.max_linear_vel = 1.0    # m/s
self.max_angular_vel = 2.0   # rad/s
```

### Change Gazebo Physics
Edit `src/rccar_gazebo/worlds/restaurant.world`:
```xml
<physics name="default_physics" type="ode">
  <max_step_size>0.001</max_step_size>
  <gravity>0 0 -9.81</gravity>
</physics>
```

### Change Robot Dimensions
Edit `src/rccar_description/urdf/rccar.urdf.xacro`:
```xml
<xacro:property name="body_l"  value="0.400"/>  <!-- Length -->
<xacro:property name="body_w"  value="0.300"/>  <!-- Width -->
<xacro:property name="wheel_r" value="0.040"/>  <!-- Wheel radius -->
```

## 🐛 Troubleshooting

### Gazebo won't start
```bash
make check-deps      # Verify gazebo-ros is installed
make setup          # Reinstall dependencies
```

### Gamepad not detected
```bash
ls /dev/input/js*   # Check joystick device
make joy-test       # Test with joy_node directly
```

### Build errors
```bash
make clean          # Clean build
make build          # Rebuild from scratch
```

### Environment not sourced
```bash
source install/setup.bash   # Manual source
# OR
make source         # Using Makefile
```

## 📚 Useful ROS2 Commands

```bash
# List topics
ros2 topic list

# Show message on topic
ros2 topic echo /cmd_vel

# Test camera data
ros2 topic echo /camera/front/image_raw

# Check TF tree
ros2 run rqt_tf_tree rqt_tf_tree

# Monitor node activity
ros2 node list

# Check launched nodes
ros2 lifecycle list
```

## 🎓 Learning Resources

- [ROS2 Official Documentation](https://docs.ros.org/en/humble/)
- [Gazebo Integration](https://github.com/gazebosim/ros_gazebo_bridge)
- [Mecanum Wheel Kinematics](https://www.ros.org/RoboWiki/Robots/Mecanum%20Wheels)
- [Reinforcement Learning with ROS2](https://github.com/ros-reinforcement-learning)

## 📞 Support

For issues or questions:
1. Check the troubleshooting section
2. Review ROS2 logs: `ros2 topic echo /rosout`
3. Use `make info` to verify workspace setup

---

**Last Updated:** 2024  
**ROS2 Version:** Humble  
**License:** Apache 2.0
