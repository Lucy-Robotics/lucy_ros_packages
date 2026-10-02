#!/usr/bin/zsh
# Copyright 2025 Sentience Robotics Team
# Health check script for Lucy Robot System

SESSION_NAME="lucy"
_SCRIPT_DIR="${0:A:h}"
source "${_SCRIPT_DIR}/lucy_workspace.zsh.inc"
WORKSPACE="${LUCY_WORKSPACE}"
CONTROL_PANEL_DIR="${LUCY_CONTROL_PANEL_DIR}"

# Colors for output
GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}🔍 Checking Lucy Robot System Status...${NC}"
echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}Workspace:${NC} ${WORKSPACE}"
echo -e "${BLUE}Control panel:${NC} ${CONTROL_PANEL_DIR}"
echo ""

# Check if tmux session exists
echo -n "tmux session: "
if tmux has-session -t $SESSION_NAME 2>/dev/null; then
    echo -e "${GREEN}✅ Running${NC}"
    TMUX_RUNNING=true
else
    echo -e "${RED}❌ Not running${NC}"
    TMUX_RUNNING=false
    echo ""
    echo "Start with: ~/launch_lucy.sh"
    exit 1
fi

# Source ROS2 workspace
source $WORKSPACE/install/setup.zsh 2>/dev/null || true

echo ""
echo -e "${BLUE}ROS2 Nodes:${NC}"

# Check Modbus bridge nodes (one per flashed board with serial_id)
echo -n "  Modbus bridges:          "
BRIDGE_COUNT=$(ros2 node list 2>/dev/null | grep -c "modbus_bridge" || echo "0")
if [ "$BRIDGE_COUNT" -ge 1 ]; then
    echo -e "${GREEN}✅ Active ($BRIDGE_COUNT)${NC}"
else
    echo -e "${YELLOW}⚠️  None (ok if real:=false)${NC}"
fi

# Check camera node
echo -n "  Camera Publisher:        "
if ros2 node list 2>/dev/null | grep -q "camera_publisher"; then
    echo -e "${GREEN}✅ Active${NC}"
else
    echo -e "${RED}❌ Not found${NC}"
fi

# Check camera stream controller
echo -n "  Camera Stream Controller: "
if ros2 node list 2>/dev/null | grep -q "camera_stream_controller"; then
    echo -e "${GREEN}✅ Active${NC}"
else
    echo -e "${RED}❌ Not found${NC}"
fi

# Check rosbridge
echo -n "  ROSBridge Server:        "
if ros2 node list 2>/dev/null | grep -q "rosbridge"; then
    echo -e "${GREEN}✅ Active${NC}"
else
    echo -e "${RED}❌ Not found${NC}"
fi

# Hardware config pipeline (from lucy_bringup lucy.launch.py)
echo -n "  Config services (YAML): "
if ros2 service list 2>/dev/null | grep -Fq "/config/get"; then
    echo -e "${GREEN}✅ config/get OK${NC}"
else
    echo -e "${RED}❌ /config/get missing${NC}"
fi

echo -n "  Config pipeline node: "
if ros2 node list 2>/dev/null | grep -Fq "/lucy_config_pipeline"; then
    echo -e "${GREEN}✅ lucy_config_pipeline${NC}"
else
    echo -e "${YELLOW}⚠️  lucy_config_pipeline not in node list${NC}"
fi

echo -n "  ConfigurePipeline action: "
if ros2 action list 2>/dev/null | grep -Fq "/configure_pipeline"; then
    echo -e "${GREEN}✅ /configure_pipeline${NC}"
else
    echo -e "${YELLOW}⚠️  /configure_pipeline not listed${NC}"
fi

# Check realsense node
echo -n "  Realsense Camera:         "
if ros2 node list 2>/dev/null | grep -q "realsense2_camera"; then
    echo -e "${GREEN}✅ Active${NC}"
else
    echo -e "${RED}❌ Not found${NC}"
fi

# Check audio nodes
echo -n "  Audio Capturer:          "
if ros2 node list 2>/dev/null | grep -q "audio_capturer"; then
    echo -e "${GREEN}✅ Active${NC}"
else
    echo -e "${RED}❌ Not found${NC}"
fi

echo -n "  Audio Player:            "
if ros2 node list 2>/dev/null | grep -q "audio_player"; then
    echo -e "${GREEN}✅ Active${NC}"
else
    echo -e "${RED}❌ Not found${NC}"
fi

echo ""
echo -e "${BLUE}Control panel (Vite):${NC}"

echo -n "  Repository on disk:      "
if [[ -d "${CONTROL_PANEL_DIR}" ]]; then
    echo -e "${GREEN}✅ ${CONTROL_PANEL_DIR}${NC}"
else
    echo -e "${RED}❌ Missing (expected under src/lucy_control_panel)${NC}"
fi

# Match vite.config default (VITE_PORT or 5000)
_VITE_PORT="${VITE_PORT:-5000}"
echo -n "  Dev server (port ${_VITE_PORT}): "
if command -v curl >/dev/null 2>&1; then
    _code="$(curl -s -o /dev/null -w '%{http_code}' --connect-timeout 1 "http://127.0.0.1:${_VITE_PORT}/" 2>/dev/null)" || _code=""
    [[ -z "${_code}" ]] && _code="000"
    if [[ "${_code}" == 2* ]] || [[ "${_code}" == 3* ]]; then
        echo -e "${GREEN}✅ HTTP ${_code} (pane 1 / yarn dev)${NC}"
    else
        echo -e "${YELLOW}⚠️  No response (HTTP ${_code}) — start via launch pane 1 if needed${NC}"
    fi
else
    if command -v ss >/dev/null 2>&1 && ss -tln 2>/dev/null | grep -q ":${_VITE_PORT}\\b"; then
        echo -e "${GREEN}✅ Port ${_VITE_PORT} listening${NC}"
    else
        echo -e "${YELLOW}⚠️  Install curl or ss to probe; port ${_VITE_PORT} not detected listening${NC}"
    fi
fi
unset _VITE_PORT _code

echo ""
echo -e "${BLUE}Camera Services:${NC}"

# Check camera services
for service in "/camera_publisher/start_streaming" "/camera_publisher/stop_streaming"; do
    echo -n "  $service: "
    if ros2 service list 2>/dev/null | grep -q "$service"; then
        echo -e "${GREEN}✅ Available${NC}"
    else
        echo -e "${YELLOW}⚠️  Not found${NC}"
    fi
done

echo ""
echo -e "${BLUE}Key Topics:${NC}"

# Check important topics
for topic in "/actuators/right_arm" "/actuators/left_arm" "/trace_publisher" "/ext_camera/jpg" "/audio"; do
    echo -n "  $topic: "
    if ros2 topic list 2>/dev/null | grep -q "$topic"; then
        echo -e "${GREEN}✅ Available${NC}"
    else
        echo -e "${YELLOW}⚠️  Not found${NC}"
    fi
done

echo ""
echo -e "${BLUE}Serial Devices:${NC}"

# Check serial devices
for device in "/dev/ttyACM0" "/dev/ttyACM1"; do
    echo -n "  $device: "
    if [ -e "$device" ]; then
        echo -e "${GREEN}✅ Connected${NC}"
    else
        echo -e "${RED}❌ Not found${NC}"
    fi
done

echo ""
echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}Commands:${NC}"
echo "  Attach:  tmux attach -t $SESSION_NAME"
echo "  Stop:    ~/stop_lucy.sh"
echo -e "${BLUE}========================================${NC}"

