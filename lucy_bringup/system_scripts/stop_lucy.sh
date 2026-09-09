#!/usr/bin/zsh
# Copyright 2024 Sentience Robotics Team
# Stop script for Lucy Robot System

SESSION_NAME="lucy"
_SCRIPT_DIR="${0:A:h}"
source "${_SCRIPT_DIR}/lucy_workspace.zsh.inc"
WORKSPACE="${LUCY_WORKSPACE}"

# Colors for output
GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}🛑 Stopping Lucy Robot System...${NC}"
echo -e "${BLUE}========================================${NC}"

# Check if tmux session exists
if ! tmux has-session -t $SESSION_NAME 2>/dev/null; then
    echo -e "${YELLOW}⚠️  Session '$SESSION_NAME' is not running${NC}"
    exit 0
fi

echo -e "${BLUE}📋 Gracefully stopping processes...${NC}"

# Source ROS2 workspace for service calls
source "${WORKSPACE}/install/setup.zsh" 2>/dev/null || true

# Stop camera streaming via service (with timeout)
echo -e "${BLUE}  → Stopping camera streaming...${NC}"
if ros2 service list 2>/dev/null | grep -q "/camera_publisher/stop_streaming"; then
    timeout 2 ros2 service call /camera_publisher/stop_streaming std_srvs/srv/SetBool "{data: true}" 2>/dev/null || true
else
    echo -e "${YELLOW}    ⚠️  Camera service not available${NC}"
fi
sleep 1

# Stop ROS nodes (pane 0): ros2 launch often needs a second SIGINT to tear down nested
# launches (e.g. rosbridge spawned via ExecuteProcess/shell).
echo -e "${BLUE}  → Stopping ROS2 nodes...${NC}"
tmux send-keys -t $SESSION_NAME:0.0 C-c 2>/dev/null || true
sleep 3
tmux send-keys -t $SESSION_NAME:0.0 C-c 2>/dev/null || true
sleep 2

# Stop web interface (pane 1)
echo -e "${BLUE}  → Stopping web interface...${NC}"
tmux send-keys -t $SESSION_NAME:0.1 C-c 2>/dev/null || true
sleep 1

# Kill the tmux session (sends SIGHUP to remaining pane processes)
echo -e "${BLUE}  → Terminating tmux session...${NC}"
tmux kill-session -t $SESSION_NAME 2>/dev/null || true
sleep 1

# Fallback: processes launched under shell wrappers sometimes survive tmux teardown.
echo -e "${BLUE}  → Cleaning up orphaned Lucy launch helpers (if any)...${NC}"
pkill -TERM -f rosbridge_websocket 2>/dev/null || true
pkill -TERM -f rosbridge_websocket_launch 2>/dev/null || true
pkill -TERM -f lucy_modbus_bridge 2>/dev/null || true
pkill -TERM -f modbus_bridge_node 2>/dev/null || true
sleep 2
pkill -KILL -f rosbridge_websocket 2>/dev/null || true
pkill -KILL -f rosbridge_websocket_launch 2>/dev/null || true

echo ""
echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}✅ Lucy system stopped successfully!${NC}"
echo -e "${GREEN}========================================${NC}"

