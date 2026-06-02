#!/bin/bash
#
# rl_menu.sh - Interactive RC-Car RL Training Menu
#

set -e

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

# Paths
PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
INSTALL_DIR="$PROJECT_ROOT/install"
RL_TRAINING_DIR=~/rl_training
LOG_DIR=$RL_TRAINING_DIR/logs
MODEL_DIR=$RL_TRAINING_DIR/models

# Check if environment is set up
if [ ! -f "$INSTALL_DIR/setup.bash" ]; then
    echo -e "${RED}✗ ROS2 environment not built. Run: make build && make source${NC}"
    exit 1
fi

source "$INSTALL_DIR/setup.bash"

# Print header
print_header() {
    echo -e "${BLUE)╔═══════════════════════════════════════════════════════════╗${NC}"
    echo -e "${BLUE}║  🤖 RC-Car Reinforcement Learning Training Menu          ║${NC}"
    echo -e "${BLUE}║     AWS RoboMaker Bookstore Map · PPO · Gymnasium        ║${NC}"
    echo -e "${BLUE}╚═══════════════════════════════════════════════════════════╝${NC}"
    echo ""
}

# Print training status
print_status() {
    echo -e "${CYAN}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
    echo -e "${CYAN}📊 Training Status${NC}"
    echo -e "${CYAN}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
    
    if [ -d "$MODEL_DIR" ]; then
        model_count=$(ls -1 "$MODEL_DIR"/*.zip 2>/dev/null | wc -l)
        echo -e "  Models saved:        ${GREEN}$model_count${NC}"
    else
        echo -e "  Models saved:        ${RED}0${NC}"
    fi
    
    if [ -d "$LOG_DIR" ] && [ -f "$LOG_DIR/monitor.csv" ]; then
        total_episodes=$(tail -1 "$LOG_DIR/monitor.csv" 2>/dev/null | cut -d',' -f1 | head -c 10 || echo "?")
        echo -e "  Total episodes:      ${GREEN}$total_episodes${NC}"
    else
        echo -e "  Total episodes:      ${RED}0${NC}"
    fi
    
    echo -e "  Training dir:        ${YELLOW}$RL_TRAINING_DIR${NC}"
    echo ""
}

# Menu options
show_menu() {
    echo -e "${YELLOW}Select an option:${NC}"
    echo ""
    echo -e "  ${GREEN}1${NC}) Start fresh training (2M timesteps)"
    echo -e "  ${GREEN}2${NC}) Resume training from checkpoint (3M timesteps)"
    echo -e "  ${GREEN}3${NC}) Start monitoring dashboard (TensorBoard + Web UI)"
    echo -e "  ${GREEN}4${NC}) Launch Gazebo simulation (in new terminal)"
    echo -e "  ${GREEN}5${NC}) View training logs"
    echo -e "  ${GREEN}6${NC}) Check GPU/CUDA status"
    echo -e "  ${GREEN}7${NC}) Quick install RL dependencies"
    echo -e "  ${GREEN}8${NC}) View best model info"
    echo -e "  ${GREEN}0${NC}) Exit"
    echo ""
    read -p "Enter choice [0-8]: " choice
}

# Train function
train_fresh() {
    echo ""
    echo -e "${YELLOW}Starting fresh training...${NC}"
    echo -e "${YELLOW}Note: Gazebo simulation should be running!${NC}"
    echo ""
    read -p "Continue? (y/n): " confirm
    if [ "$confirm" != "y" ]; then
        return
    fi
    
    echo -e "${GREEN}Training started. Press Ctrl+C to stop and save.${NC}"
    python3 -m rccar_rl.train_rl --timesteps 2000000
}

# Resume training
train_resume() {
    echo ""
    echo -e "${YELLOW}Resuming training from checkpoint...${NC}"
    echo -e "${YELLOW}Note: Gazebo simulation should be running!${NC}"
    echo ""
    
    if [ ! -d "$MODEL_DIR" ] || [ -z "$(ls -1 "$MODEL_DIR"/*.zip 2>/dev/null)" ]; then
        echo -e "${RED}✗ No checkpoints found in $MODEL_DIR${NC}"
        read -p "Press Enter to continue..."
        return
    fi
    
    echo -e "${CYAN}Available checkpoints:${NC}"
    ls -lh "$MODEL_DIR"/*.zip 2>/dev/null | awk '{print "  " $9}' | tail -5
    echo ""
    read -p "Continue with latest checkpoint? (y/n): " confirm
    if [ "$confirm" != "y" ]; then
        return
    fi
    
    echo -e "${GREEN}Training resumed. Press Ctrl+C to stop and save.${NC}"
    python3 -m rccar_rl.train_rl --resume --timesteps 3000000
}

# Start monitoring
start_monitoring() {
    echo ""
    echo -e "${GREEN}Monitoring dashboard starting...${NC}"
    echo -e "${GREEN}  Web UI:     http://localhost:8765${NC}"
    echo -e "${GREEN}  TensorBoard: http://localhost:6006${NC}"
    echo ""
    python3 -m rccar_rl.monitor_training --port 8765 --tensorboard-port 6006
}

# Launch Gazebo
launch_gazebo() {
    echo ""
    echo -e "${YELLOW}Launching Gazebo in new terminal...${NC}"
    gnome-terminal -- bash -c "source $INSTALL_DIR/setup.bash && \
        echo 'Gazebo Simulation' && \
        ros2 launch rccar_bringup sim.launch.py && \
        bash" 2>/dev/null || \
    xterm -e "source $INSTALL_DIR/setup.bash && \
        echo 'Gazebo Simulation' && \
        ros2 launch rccar_bringup sim.launch.py" 2>/dev/null || \
    (echo -e "${YELLOW}Please run manually: source $INSTALL_DIR/setup.bash && make sim${NC}")
}

# View logs
view_logs() {
    echo ""
    if [ ! -f "$LOG_DIR/monitor.csv" ]; then
        echo -e "${RED}✗ No training logs found${NC}"
        read -p "Press Enter to continue..."
        return
    fi
    
    echo -e "${CYAN}Last 20 episodes:${NC}"
    tail -20 "$LOG_DIR/monitor.csv"
    echo ""
    read -p "Press Enter to continue..."
}

# Check GPU
check_gpu() {
    echo ""
    echo -e "${CYAN}GPU/CUDA Status:${NC}"
    echo ""
    
    if command -v nvidia-smi &> /dev/null; then
        nvidia-smi --query-gpu=index,name,driver_version,memory.total,memory.free --format=csv,noheader
    else
        echo -e "${YELLOW}NVIDIA GPU not detected. CPU mode will be used.${NC}"
    fi
    
    echo ""
    python3 -c "import torch; print(f'PyTorch CUDA available: {torch.cuda.is_available()}'); print(f'CUDA version: {torch.version.cuda}')" 2>/dev/null || true
    echo ""
    read -p "Press Enter to continue..."
}

# Install dependencies
install_deps() {
    echo ""
    echo -e "${YELLOW}Installing RL dependencies...${NC}"
    echo ""
    
    pip install --upgrade pip
    pip install gymnasium stable-baselines3 tensorboard
    
    echo -e "${GREEN}✓ Dependencies installed${NC}"
    read -p "Press Enter to continue..."
}

# View best model
view_best_model() {
    echo ""
    echo -e "${CYAN}Best Model Information:${NC}"
    
    best_model_dir=~/rl_training/best_model
    if [ ! -d "$best_model_dir" ] || [ -z "$(ls -1 "$best_model_dir"/*.zip 2>/dev/null)" ]; then
        echo -e "${YELLOW}No best model saved yet. It will be created during training.${NC}"
        read -p "Press Enter to continue..."
        return
    fi
    
    echo ""
    ls -lh "$best_model_dir"/*.zip 2>/dev/null | awk '{print "  Size: " $5 "  Modified: " $6" "$7" "$8}'
    echo ""
    read -p "Press Enter to continue..."
}

# Main loop
main_loop() {
    while true; do
        print_header
        print_status
        show_menu
        
        case $choice in
            1) train_fresh ;;
            2) train_resume ;;
            3) start_monitoring ;;
            4) launch_gazebo ;;
            5) view_logs ;;
            6) check_gpu ;;
            7) install_deps ;;
            8) view_best_model ;;
            0) 
                echo -e "${YELLOW}Exiting...${NC}"
                exit 0
                ;;
            *)
                echo -e "${RED}Invalid option${NC}"
                read -p "Press Enter to continue..."
                ;;
        esac
    done
}

# Run main loop
main_loop
