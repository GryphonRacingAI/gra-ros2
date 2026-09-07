#!/bin/bash
LOGFILE=$HOME/startup_tmux.log
echo "[$(date)] tmux startup script starting..." >> "$LOGFILE"

# Log Secure Boot status (silently fail if mokutil is not available or fails)
# If Secure Boot is enabled, certain Nvidia drivers/modules may not load
echo "[$(date)] Secure Boot status:" >> "$LOGFILE"
mokutil --sb-state >> "$LOGFILE" 2>/dev/null || echo "[$(date)] mokutil not available or failed to run." >> "$LOGFILE"

# Setup CAN
if ip link show can0 > /dev/null 2>&1; then
    echo "[$(date)] can0 found, bringing it up..." >> "$LOGFILE"
    sudo ip link set can0 up type can bitrate 500000 >> "$LOGFILE" 2>&1
    sudo ifconfig can0 up >> "$LOGFILE" 2>&1
fi

sleep 5

SESSION_NAME="ros_startup"

# Check if tmux session exists, kill if it does (optional)
if tmux has-session -t $SESSION_NAME 2>/dev/null; then
    tmux kill-session -t $SESSION_NAME
fi

# Start a new tmux session, detached
tmux new-session -d -s $SESSION_NAME

# Run ackermann_can in first pane (pane 0)
tmux send-keys -t $SESSION_NAME "source $HOME/virtual_env/bin/activate; ros2 run fsai_api ackermann_can can0" C-m

# Split window vertically for second pane and immediately run command
tmux split-window -v -t $SESSION_NAME
tmux send-keys -t $SESSION_NAME "source $HOME/zed_env/bin/activate; ros2 launch bringup sensor_bringup.launch.py" C-m

# Split the top pane horizontally and immediately run command
tmux split-window -h -t $SESSION_NAME:0.0
tmux send-keys -t $SESSION_NAME "source $HOME/virtual_env/bin/activate; ros2 launch mission_supervisor mission.launch.py" C-m

# Select top-left pane (optional)
tmux select-pane -t $SESSION_NAME:0.0

echo "[$(date)] tmux session $SESSION_NAME started with 3 panes." >> "$LOGFILE"
echo "[$(date)] Pane 0: ackermann_can" >> "$LOGFILE"
echo "[$(date)] Pane 1: sensor_bringup.launch.py" >> "$LOGFILE"
echo "[$(date)] Pane 2: mission.launch.py" >> "$LOGFILE"
