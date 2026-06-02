.PHONY: help build clean setup sim sim-gamepad check-deps list-packages info

# Colors for output
RED := \033[0;31m
GREEN := \033[0;32m
YELLOW := \033[1;33m
BLUE := \033[0;34m
NC := \033[0m # No Color

# Project directories
PROJECT_ROOT := $(shell pwd)
SRC_DIR := $(PROJECT_ROOT)/src
BUILD_DIR := $(PROJECT_ROOT)/build
INSTALL_DIR := $(PROJECT_ROOT)/install

# Help message
help:
	@echo "$(BLUE)╔═══════════════════════════════════════════════════════════╗$(NC)"
	@echo "$(BLUE)║     RC-Car Serving Robot - ROS2 Development Menu         ║$(NC)"
	@echo "$(BLUE)╚═══════════════════════════════════════════════════════════╝$(NC)"
	@echo ""
	@echo "$(YELLOW)📦 BUILD & SETUP:$(NC)"
	@echo "  $(GREEN)make setup$(NC)           - Install dependencies and setup workspace"
	@echo "  $(GREEN)make build$(NC)           - Build all ROS2 packages (colcon build)"
	@echo "  $(GREEN)make source$(NC)          - Source the installation (install/setup.bash)"
	@echo "  $(GREEN)make clean$(NC)           - Clean build and install directories"
	@echo ""
	@echo "$(YELLOW)🎮 SIMULATION:$(NC)"
	@echo "  $(GREEN)make sim$(NC)             - Launch Gazebo + Robot simulation"
	@echo "  $(GREEN)make sim-gamepad$(NC)     - Launch sim + gamepad control nodes"
	@echo "  $(GREEN)make gazebo-only$(NC)     - Launch Gazebo server only (no GUI)"
	@echo ""
	@echo "$(YELLOW)🕹️  GAMEPAD TEST:$(NC)"
	@echo "  $(GREEN)make joy-test$(NC)        - Test joystick/gamepad input (joy_node)"
	@echo "  $(GREEN)make gamepad-control$(NC) - Test gamepad controller node"
	@echo ""
	@echo "$(YELLOW)🤖 REINFORCEMENT LEARNING:$(NC)"
	@echo "  $(GREEN)make rl-train$(NC)        - Start PPO training (2M timesteps)"
	@echo "  $(GREEN)make rl-train-resume$(NC) - Resume training from last checkpoint"
	@echo "  $(GREEN)make rl-monitor$(NC)      - Start monitoring dashboard & TensorBoard"
	@echo "  $(GREEN)make rl-ui$(NC)           - Interactive RL menu (Gazebo + Training)"
	@echo ""
	@echo "$(YELLOW)🔍 UTILITIES:$(NC)"
	@echo "  $(GREEN)make check-deps$(NC)      - Check required dependencies"
	@echo "  $(GREEN)make list-packages$(NC)   - List all ROS2 packages in workspace"
	@echo "  $(GREEN)make info$(NC)            - Show workspace structure and info"
	@echo "  $(GREEN)make help$(NC)            - Show this help message"
	@echo ""
	@echo "$(YELLOW)📝 NOTES:$(NC)"
	@echo "  - Run '$(GREEN)make setup$(NC)' first time to install dependencies"
	@echo "  - Run '$(GREEN)make build$(NC)' after code changes"
	@echo "  - Run '$(GREEN)make source$(NC)' before running any launch/executable"
	@echo "  - Gamepad: Xbox (wired) or Wireless game pad on /dev/input/js0"
	@echo ""

# Setup workspace (install dependencies)
setup:
	@echo "$(BLUE)Setting up ROS2 workspace...$(NC)"
	@echo "$(YELLOW)Installing required packages...$(NC)"
	sudo apt update
	sudo apt install -y python3-rosdep python3-colcon-common-extensions
	rosdep install --from-paths src --ignore-src -r -y
	@echo "$(GREEN)✓ Setup complete$(NC)"

# Build all packages
build:
	@echo "$(BLUE)Building ROS2 packages...$(NC)"
	@mkdir -p $(BUILD_DIR) $(INSTALL_DIR)
	cd $(PROJECT_ROOT) && colcon build --symlink-install
	@echo "$(GREEN)✓ Build complete$(NC)"
	@echo "$(YELLOW)Next: run 'make source' to setup environment$(NC)"

# Source installation
source:
	@bash -c 'source $(INSTALL_DIR)/setup.bash && echo "$(GREEN)✓ Environment sourced$(NC)"'

# Clean build/install
clean:
	@echo "$(RED)Cleaning build and install directories...$(NC)"
	rm -rf $(BUILD_DIR) $(INSTALL_DIR)
	find $(SRC_DIR) -name "__pycache__" -type d -exec rm -rf {} + 2>/dev/null || true
	@echo "$(GREEN)✓ Clean complete$(NC)"

# Gazebo simulation (with spawn)
sim:
	@echo "$(BLUE)Launching Gazebo simulation...$(NC)"
	@bash -c 'source $(INSTALL_DIR)/setup.bash && ros2 launch rccar_bringup sim.launch.py'

# Gazebo simulation + gamepad control
sim-gamepad:
	@echo "$(BLUE)Launching Gazebo simulation + Gamepad control...$(NC)"
	@bash -c 'source $(INSTALL_DIR)/setup.bash && ros2 launch rccar_bringup sim.launch.py'

# Gazebo server only (no GUI, for testing)
gazebo-only:
	@echo "$(BLUE)Launching Gazebo server only (no client)...$(NC)"
	@bash -c 'source $(INSTALL_DIR)/setup.bash && ros2 launch rccar_gazebo gazebo.launch.py gui:=false'

# Test joystick input
joy-test:
	@echo "$(BLUE)Testing joystick input (joy_node)...$(NC)"
	@echo "$(YELLOW)Connect your gamepad/joystick and press any button...$(NC)"
	@bash -c 'source $(INSTALL_DIR)/setup.bash && ros2 run joy joy_node'

# Test gamepad controller
gamepad-control:
	@echo "$(BLUE)Testing gamepad controller...$(NC)"
	@bash -c 'source $(INSTALL_DIR)/setup.bash && ros2 launch rccar_control control.launch.py'

# Check dependencies
check-deps:
	@echo "$(BLUE)Checking ROS2 dependencies...$(NC)"
	@which ros2 > /dev/null && echo "$(GREEN)✓ ROS2 installed$(NC)" || echo "$(RED)✗ ROS2 not found$(NC)"
	@which colcon > /dev/null && echo "$(GREEN)✓ colcon installed$(NC)" || echo "$(RED)✗ colcon not found$(NC)"
	@which gazebo > /dev/null && echo "$(GREEN)✓ Gazebo installed$(NC)" || echo "$(RED)✗ Gazebo not found$(NC)"
	@dpkg -l | grep -q gazebo-ros && echo "$(GREEN)✓ gazebo-ros installed$(NC)" || echo "$(RED)✗ gazebo-ros not found$(NC)"
	@dpkg -l | grep -q ros-$(ROS_DISTRO)-joy && echo "$(GREEN)✓ ros-joy installed$(NC)" || echo "$(RED)✗ ros-joy not found$(NC)"
	@echo ""
	@echo "$(YELLOW)Missing dependencies? Run: make setup$(NC)"

# List all packages
list-packages:
	@echo "$(BLUE)ROS2 Packages in workspace:$(NC)"
	@cd $(PROJECT_ROOT) && colcon list --names-only 2>/dev/null || find src -maxdepth 2 -name "package.xml" -exec dirname {} \; | sed 's|.*src/||'

# Show workspace info
info:
	@echo "$(BLUE)╔═══════════════════════════════════════════════════════════╗$(NC)"
	@echo "$(BLUE)║           Workspace Information                          ║$(NC)"
	@echo "$(BLUE)╚═══════════════════════════════════════════════════════════╝$(NC)"
	@echo ""
	@echo "$(YELLOW)Project Root:$(NC) $(PROJECT_ROOT)"
	@echo "$(YELLOW)ROS Distro:$(NC)   $(shell . /opt/ros/humble/setup.sh > /dev/null 2>&1 && echo 'humble' || echo 'unknown')"
	@echo ""
	@echo "$(YELLOW)📁 Directory Structure:$(NC)"
	@tree -L 2 -a $(PROJECT_ROOT) 2>/dev/null || find $(SRC_DIR) -maxdepth 2 -type d | sed 's|$(PROJECT_ROOT)|.|' | head -20
	@echo ""
	@echo "$(YELLOW)📦 Packages:$(NC)"
	@ls -d $(SRC_DIR)/*/ 2>/dev/null | sed 's|.*src/||' | sed 's|/||' | nl
	@echo ""
	@echo "$(YELLOW)🔑 Key Files:$(NC)"
	@echo "  URDF:           src/rccar_description/urdf/"
	@echo "  Gazebo World:   src/rccar_gazebo/worlds/restaurant.world"
	@echo "  Launch Files:   src/rccar_*/launch/"
	@echo "  Control Nodes:  src/rccar_control/rccar_control/"
	@echo ""

# ──────────────────────────────────────────────────────────────
# Reinforcement Learning Targets
# ──────────────────────────────────────────────────────────────

# RL training start (fresh)
rl-train:
	@echo "$(BLUE)╔═══════════════════════════════════════════════════════════╗$(NC)"
	@echo "$(BLUE)║     RC-Car RL Training - PPO (Fresh)                     ║$(NC)"
	@echo "$(BLUE)╚═══════════════════════════════════════════════════════════╝$(NC)"
	@echo ""
	@echo "$(YELLOW)Prerequisites:$(NC)"
	@echo "  1. Gazebo simulation running: $(GREEN)make sim$(NC)"
	@echo "  2. ROS2 environment sourced"
	@echo ""
	@echo "$(YELLOW)Training starts in 5 seconds... (Ctrl+C to cancel)$(NC)"
	@sleep 5
	@bash -c 'source $(INSTALL_DIR)/setup.bash && \
		python3 -m rccar_rl.train_rl --timesteps 2000000'

# RL training resume (Phase 1 이어서)
rl-train-resume:
	@echo "$(BLUE)╔═══════════════════════════════════════════════════════════╗$(NC)"
	@echo "$(BLUE)║     RC-Car RL Training - PPO Phase 1 (Resume)            ║$(NC)"
	@echo "$(BLUE)╚═══════════════════════════════════════════════════════════╝$(NC)"
	@echo ""
	@echo "$(YELLOW)Prerequisites:$(NC)"
	@echo "  1. Gazebo simulation running: $(GREEN)make sim$(NC)"
	@echo "  2. Previous checkpoint exists in ~/rl_training/models/"
	@echo ""
	@echo "$(YELLOW)Resuming Phase 1 in 5 seconds... (Ctrl+C to cancel)$(NC)"
	@sleep 5
	@bash -c 'source $(INSTALL_DIR)/setup.bash && \
		python3 -m rccar_rl.train_rl --phase 1 --resume --timesteps 2000000'

# RL training Phase 2: A+B 심화 (Phase 1 완료 후 실행)
rl-train-phase2:
	@echo "$(BLUE)╔═══════════════════════════════════════════════════════════╗$(NC)"
	@echo "$(BLUE)║     RC-Car RL Training - PPO Phase 2 (A+B 심화)          ║$(NC)"
	@echo "$(BLUE)╚═══════════════════════════════════════════════════════════╝$(NC)"
	@echo ""
	@echo "$(YELLOW)Phase 2: 속도 정규화 + 횡 이동 페널티$(NC)"
	@echo "$(YELLOW)Prerequisites:$(NC)"
	@echo "  1. Gazebo simulation running: $(GREEN)make sim$(NC)"
	@echo "  2. Phase 1 완료 (~/rl_training/models/ 에 체크포인트 존재)"
	@echo ""
	@echo "$(YELLOW)Phase 2 시작 (5초 후)... (Ctrl+C to cancel)$(NC)"
	@sleep 5
	@bash -c 'source $(INSTALL_DIR)/setup.bash && \
		python3 -m rccar_rl.train_rl --phase 2 --timesteps 2000000'

# RL monitoring dashboard
rl-monitor:
	@echo "$(BLUE)╔═══════════════════════════════════════════════════════════╗$(NC)"
	@echo "$(BLUE)║     RC-Car RL Monitoring Dashboard                       ║$(NC)"
	@echo "$(BLUE)╚═══════════════════════════════════════════════════════════╝$(NC)"
	@echo ""
	@echo "$(GREEN)✓ Web Dashboard:$(NC)  http://localhost:8765"
	@echo "$(GREEN)✓ TensorBoard:$(NC)     http://localhost:6006"
	@echo ""
	@echo "$(YELLOW)Monitoring started. Refresh dashboards in browser.$(NC)"
	@echo "$(YELLOW)Training files: ~/rl_training/$(NC)"
	@echo ""
	@bash -c 'python3 -m rccar_rl.monitor_training --port 8765 --tensorboard-port 6006'

# Interactive RL UI menu
rl-ui:
	@echo "$(BLUE)╔═══════════════════════════════════════════════════════════╗$(NC)"
	@echo "$(BLUE)║     RC-Car RL Interactive Menu                           ║$(NC)"
	@echo "$(BLUE)╚═══════════════════════════════════════════════════════════╝$(NC)"
	@bash $(PROJECT_ROOT)/scripts/rl_menu.sh

# Default target
.DEFAULT_GOAL := help

