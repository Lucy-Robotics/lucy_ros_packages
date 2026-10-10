#!/usr/bin/zsh
# Copyright 2026 Lucy Robotics Team
# Launch script for Lucy Robot System using tmux

set -e  # Exit on error

SESSION_NAME="lucy"
_SCRIPT_DIR="${0:A:h}"
source "${_SCRIPT_DIR}/lucy_workspace.zsh.inc"
WORKSPACE="${LUCY_WORKSPACE}"
WEB_DIR="${LUCY_CONTROL_PANEL_DIR}"

# Colors for output
GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}🤖 Launching Lucy Robot System...${NC}"
echo -e "${GREEN}========================================${NC}"

# Check if tmux is installed
if ! command -v tmux &> /dev/null; then
    echo -e "${RED}❌ tmux is not installed!${NC}"
    echo "Install with: sudo apt install tmux"
    exit 1
fi

# Check if tmux session already exists
if tmux has-session -t $SESSION_NAME 2>/dev/null; then
    echo -e "${YELLOW}⚠️  Session '$SESSION_NAME' already exists!${NC}"
    echo ""
    echo "Options:"
    echo "  1. Attach to existing session:"
    echo -e "     ${BLUE}tmux attach -t $SESSION_NAME${NC}"
    echo ""
    echo "  2. Kill and recreate:"
    echo -e "     ${BLUE}tmux kill-session -t $SESSION_NAME${NC}"
    echo -e "     ${BLUE}./launch_lucy.sh${NC}"
    echo ""
    echo "  3. Use the stop script:"
    echo -e "     ${BLUE}~/stop_lucy.sh${NC}"
    exit 1
fi

# Check if workspace exists
if [ ! -d "$WORKSPACE" ]; then
    echo -e "${RED}❌ Workspace not found: $WORKSPACE${NC}"
    exit 1
fi

# Check if web directory exists
if [ ! -d "$WEB_DIR" ]; then
    echo -e "${YELLOW}⚠️  Control panel dir not found: $WEB_DIR${NC}"
    echo "Web interface will not be started."
    WEB_AVAILABLE=false
else
    WEB_AVAILABLE=true
fi

# Source ROS2 workspace
echo -e "${BLUE}📦 Sourcing ROS2 workspace...${NC}"
source $WORKSPACE/install/setup.zsh

# Create new tmux session with 3 panes
echo -e "${BLUE}🖥️  Creating tmux session with 3 panes...${NC}"

# Create session with first pane
tmux new-session -d -s $SESSION_NAME -n "Lucy"

# Split vertically (top/bottom)
tmux split-window -v -t $SESSION_NAME

# Split the bottom pane horizontally (left/right)
tmux split-window -h -t $SESSION_NAME:0.1

# Adjust pane sizes (pane 0 gets 60% height, pane 1 gets 20 cols, pane 2 gets the rest)
tmux resize-pane -t $SESSION_NAME:0.0 -y 30
tmux resize-pane -t $SESSION_NAME:0.1 -x 20

# ============================================
# PANE 0 (Top - 60%): ROS2 Launch File
# ============================================
echo -e "${BLUE}🤖 Setting up ROS2 nodes pane...${NC}"
tmux send-keys -t $SESSION_NAME:0.0 "source $WORKSPACE/install/setup.zsh" C-m
tmux send-keys -t $SESSION_NAME:0.0 "ros2 launch lucy_bringup lucy.launch.py real:=true" C-m

# ============================================
# PANE 1 (Bottom-Left): Web Interface
# ============================================
if [ "$WEB_AVAILABLE" = true ]; then
    echo -e "${BLUE}🌐 Setting up web interface pane...${NC}"
    tmux send-keys -t $SESSION_NAME:0.1 "cd $WEB_DIR" C-m
    tmux send-keys -t $SESSION_NAME:0.1 "yarn dev --host" C-m

else
    tmux send-keys -t $SESSION_NAME:0.1 "clear" C-m
    tmux send-keys -t $SESSION_NAME:0.1 "echo '⚠️  Web interface directory not found'" C-m
    tmux send-keys -t $SESSION_NAME:0.1 "echo 'Expected: $WEB_DIR'" C-m
fi

# ============================================
# PANE 2 (Bottom-Right): Debug Terminal
# ============================================
echo -e "${BLUE}📟 Setting up debug terminal pane...${NC}"
tmux send-keys -t $SESSION_NAME:0.2 "source $WORKSPACE/install/setup.zsh" C-m
tmux send-keys -t $SESSION_NAME:0.2 "clear" C-m

# Select the ROS pane as default
tmux select-pane -t $SESSION_NAME:0.0

echo ""
echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}✅ Lucy system started successfully!${NC}"
echo -e "${GREEN}========================================${NC}"
echo ""
echo -e "${BLUE}Session Control:${NC}"
echo "  Attach to session:  tmux attach -t $SESSION_NAME"
echo "  Detach (keep running): Ctrl+B then D"
echo "  Stop system:        ~/stop_lucy.sh"
echo ""
echo -e "${BLUE}Pane Navigation:${NC}"
echo "  Switch panes:       Ctrl+B then arrow keys"
echo "  Cycle panes:        Ctrl+B then O"
echo "  Zoom pane:          Ctrl+B then Z (toggle fullscreen)"
echo "  Scroll/search:      Ctrl+B then [ (then q to exit)"
echo ""
echo -e "${BLUE}Pane Layout:${NC}"
echo "  Pane 0 (Top 60%):        ROS2 nodes"
echo "  Pane 1 (Bottom-L ~12%):  Web interface"
echo "  Pane 2 (Bottom-R ~88%):  Debug terminal"
echo ""
echo -e "${YELLOW}Attaching to session in 2 seconds...${NC}"
sleep 2

# Auto-attach to the session
tmux attach -t $SESSION_NAME

