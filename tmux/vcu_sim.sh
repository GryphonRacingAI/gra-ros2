#!/bin/bash

# Purpose: Test VCU path (ackermann_can + vcu_sim.py)
#   - vcu_sim.py emulates the VCU on virtual CAN
#   - ackermann_can bridges /ackermann_cmd (and brake flags) to CAN AI2VCU frames
#   - Use the VCU teleop config so sliders publish AckermannDrive + Bool flags
#   - Observe steering/speed changes in the vcu_sim window stdout (steer= field)
#
# Prereq (one time):
#   sudo ip link add dev vcan0 type vcan 2>/dev/null || true
#   sudo ip link set vcan0 up
#
# After changes to launch/configs: colcon build --packages-select simulation

SESSION="fsai"
TMUX_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=_ws.sh
source "$TMUX_DIR/_ws.sh"

source_overlay() {
	echo "source /opt/ros/jazzy/setup.bash 2>/dev/null || source /opt/ros/humble/setup.bash; source \"$WS/install/setup.bash\""
}

# Kill old session
tmux kill-session -t $SESSION 2>/dev/null || true


start_component() {
	local win_name="$1"
	local command="$3"
	local use_venv="$4"
	local run="$2"
	tmux new-window -t $SESSION -n "$win_name"
	tmux send-keys  -t $SESSION:"$win_name" "cd $WS" C-m
	tmux send-keys  -t $SESSION:"$win_name" "$(source_overlay)" C-m
	if [ -n "$use_venv" ]; then
		tmux send-keys -t $SESSION:"$win_name" "source ros_venv/bin/activate || true" C-m
	fi

	if [ -n "$run" ]; then
		tmux send-keys  -t $SESSION:"$win_name" "$command" C-m
	else
		tmux send-keys  -t $SESSION:"$win_name" "$command"
	fi
}

tmux new-session -d -s $SESSION -n "monitor"
tmux send-keys -t $SESSION:monitor "cd $WS" C-m
tmux send-keys -t $SESSION:monitor "$(source_overlay)" C-m

# Second arg "1" auto-runs the command (sends Enter). Empty leaves it typed for manual start.
# Gazebo + planner + CAN is ./tmux/startup.sh (no --auto-drive; arm VCU in the vcu pane).
# This script is the VCU bench: virtual CAN + teleop, no Gazebo. --auto-drive is OK here.
start_component "vcu_sim"       1  "python3 $WS/src/fsai_api/scripts/vcu_sim.py vcan0 --auto-drive"			""
start_component "ackermann_can" 1  "ros2 run fsai_api ackermann_can vcan0"			""
# NOTE: wheel_speed_controller listens on /ackermann_cmd_controller and publishes to /ackermann_cmd.
# For direct testing of ackermann_can we publish to /ackermann_cmd from teleop and do not run the
# speed controller (it would overwrite commands). If you want PI speed control, publish to
# /ackermann_cmd_controller instead (or temporarily comment the teleop_vcu line below and use wheels).
# start_component "wheels" 1 "ros2 run fsai_api wheel_speed_controller.py"		""
# VCU-aware teleop: publishes directly to /ackermann_cmd + /brake /emergency_brake /chequered_flag
VCU_TELEOP_CFG="$WS/install/simulation/share/simulation/config/teleop_vcu.yaml"
start_component "teleop"	1  "ros2 launch simulation teleop.launch config:=${VCU_TELEOP_CFG}"		""


tmux attach -t $SESSION
