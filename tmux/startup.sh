#!/bin/bash
#
# Gazebo + virtual CAN + path + one controller.
#
# Usage:
#   ./tmux/startup.sh
#   CONTROLLER=pp EVENT=mppi_track CONES=yolo ./tmux/startup.sh
#   CONTROLLER=mppi CONES=perfect ./tmux/startup.sh
#   CAN=0 ./tmux/startup.sh
#   DETACH=1 ./tmux/startup.sh
#
# Env:
#   CONTROLLER  mppi | pp          (default: pp)
#   EVENT       trackdrive | mppi_track | acceleration | skidpad | autocross
#               (default: mppi_track)
#   CONES       perfect | yolo     (default: yolo — /cone_array → pathfinder)
#   CAN         1 | 0              (default: 1)  vcan0 + vcu_sim + ackermann_can
#   CAN_IFACE   SocketCAN iface    (default: vcan0)
#   WS          colcon overlay     (default: parent of src/ or of tmux/)
#   SESSION     tmux session name  (default: fsai)
#   DETACH      1 = stay detached
#   VIZ         1 = also start rviz2
#   LOGFILE     orchestrator log (default: $LOG_DIR/startup.log)
#
# CAN path (matches the car):
#   controller  -> /ackermann_cmd_planner
#   mission_supervisor forwards that to /ackermann_cmd_controller only in AS_DRIVING
#   wheel_speed -> /ackermann_cmd  -> Gazebo  and  ackermann_can -> vcan -> vcu_sim
#   scripted AMI (5/6/7) is driven by the supervisor; planner cmds are ignored
#
# VCU: vcu_sim starts in AS_OFF. Arm it in the vcu pane:
#   a = ASMS, t = TSMS, 4 = trackdrive AMI, wait 5 s in AS_READY, g = RES go
#
# YOLO always uses device:=cuda:0 (no CPU fallback).
# CUDA preflight writes $LOG_DIR/gpu.log. If nvidia_uvm is wedged, the
# launcher stops and that file has the sudo rmmod/modprobe recipe.
#
# Logs: source tmux/log.sh → log_setup; each pane runs
#   source log.sh && run_logged <name> <cmd>
#   → stdbuf -oL cmd | python stamp | tee $LOG_DIR/<name>.log
#
# Cleanup (AGENTS.md): after Gazebo tests kill leftover processes so
#   ros2 topic list  shows only ~2 baseline topics before re-running.

set -euo pipefail

SESSION="${SESSION:-fsai}"
TMUX_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=log.sh
source "$TMUX_DIR/log.sh"
# shellcheck source=gpu.sh
source "$TMUX_DIR/gpu.sh"
if [ -z "${WS:-}" ]; then
	if [ -f "$TMUX_DIR/../install/setup.bash" ]; then
		WS="$(cd "$TMUX_DIR/.." && pwd)"
	elif [ -f "$TMUX_DIR/../../install/setup.bash" ]; then
		WS="$(cd "$TMUX_DIR/../.." && pwd)"
	else
		WS="$HOME/colcon_ws"
	fi
	export WS
fi
CONTROLLER="${CONTROLLER:-pp}"
EVENT="${EVENT:-mppi_track}"
CAN="${CAN:-1}"
CAN_IFACE="${CAN_IFACE:-vcan0}"
DISPLAY_VAR="${DISPLAY:-:0}"
CONES="${CONES:-yolo}"
CLOCK_TIMEOUT_S="${CLOCK_TIMEOUT_S:-45}"

if [ -z "${LOG_DIR:-}" ]; then
	log_setup
fi
export LOG_DIR
LOGFILE="${LOGFILE:-$LOG_DIR/startup.log}"

log() {
	echo "[$(date)] $*" | tee -a "$LOGFILE" >&2
}

source_overlay() {
	echo "source /opt/ros/jazzy/setup.bash 2>/dev/null || source /opt/ros/humble/setup.bash; source \"$WS/install/setup.bash\""
}

source_overlay_here() {
	# ROS setup.bash reads optional AMENT_* vars; nounset would abort.
	set +u
	# shellcheck disable=SC1091
	source /opt/ros/jazzy/setup.bash 2>/dev/null || source /opt/ros/humble/setup.bash
	# shellcheck disable=SC1091
	source "$WS/install/setup.bash"
	set -u
}

case "$CONTROLLER" in
	mppi|pp) ;;
	*)
		echo "CONTROLLER must be mppi or pp (got: $CONTROLLER)" >&2
		exit 1
		;;
esac

case "$CONES" in
	perfect|yolo) ;;
	*)
		echo "CONES must be perfect or yolo (got: $CONES)" >&2
		exit 1
		;;
esac

if [ "$CONES" = "perfect" ] && [ "$CONTROLLER" = "pp" ]; then
	log "WARNING: CONES=perfect publishes an odom-frame path; pure pursuit is local-frame only. Prefer CONTROLLER=mppi."
fi

ensure_vcan() {
	if ip link show "$CAN_IFACE" >/dev/null 2>&1; then
		log "$CAN_IFACE is up"
		return 0
	fi
	cat >&2 <<EOF
Virtual CAN interface ${CAN_IFACE} is not up. Grok cannot run sudo.

In your own terminal:

  cd $WS
  sudo modprobe vcan
  sudo ip link add dev ${CAN_IFACE} type vcan 2>/dev/null || true
  sudo ip link set ${CAN_IFACE} up

Then re-run this script.
EOF
	log "ERROR: $CAN_IFACE missing"
	exit 1
}

wait_for_topic() {
	local topic="$1"
	local timeout_s="$2"
	local i=0
	log "Waiting up to ${timeout_s}s for $topic"
	while [ "$i" -lt "$timeout_s" ]; do
		if ros2 topic list 2>/dev/null | grep -qx "$topic"; then
			log "Saw $topic"
			return 0
		fi
		sleep 1
		i=$((i + 1))
	done
	log "ERROR: timed out waiting for $topic"
	return 1
}

if [ "$CAN" != "0" ]; then
	ensure_vcan
fi

if [ "$CONES" = "yolo" ] && [ "${GPU_SKIP:-0}" != "1" ]; then
	log "CUDA preflight (YOLO device:=cuda:0)"
	if ! gpu_require; then
		log "ERROR: CUDA not usable. See $LOG_DIR/gpu.log and $TMUX_DIR/reload_nvidia_uvm.sh"
		echo "YOLO needs a working GPU. Reload nvidia_uvm then re-run. Recipe: $LOG_DIR/gpu.log" >&2
		exit 1
	fi
	log "CUDA preflight ok"
elif [ "$CONES" = "yolo" ]; then
	log "GPU_SKIP=1 — CUDA preflight skipped; YOLO still device:=cuda:0 (respawn until GPU works)"
fi

log "tmux startup CONTROLLER=$CONTROLLER EVENT=$EVENT CONES=$CONES CAN=$CAN IFACE=$CAN_IFACE SESSION=$SESSION LOG_DIR=$LOG_DIR"

# Tear down a previous Gazebo / stack so this launch is the only one.
if [ -x "$WS/clean_gz.sh" ]; then
	log "Running clean_gz.sh"
	"$WS/clean_gz.sh" >>"$LOGFILE" 2>&1 || true
fi
tmux kill-session -t "$SESSION" 2>/dev/null || true

{
	echo "CONTROLLER=$CONTROLLER"
	echo "EVENT=$EVENT"
	echo "CONES=$CONES"
	echo "CAN=$CAN"
	echo "CAN_IFACE=$CAN_IFACE"
	echo "SESSION=$SESSION"
	echo "WS=$WS"
	echo "started=$(date -Iseconds)"
} >"$LOG_DIR/session.env"

start_component() {
	local win_name="$1"
	local run="$2"
	local command="$3"
	local use_venv="${4:-}"
	local venv_part=""
	if [ -n "$use_venv" ]; then
		venv_part="source ros_venv/bin/activate && "
	fi
	tmux new-window -t "$SESSION" -n "$win_name"
	if [ -n "$run" ]; then
		tmux send-keys -t "$SESSION:$win_name" \
			"cd \"$WS\" && export DISPLAY=${DISPLAY_VAR} LOG_DIR=\"$LOG_DIR\" ROS_LOG_DIR=\"$LOG_DIR/ros\" && $(source_overlay) && ${venv_part}source \"$TMUX_DIR/log.sh\" && set -o pipefail && run_logged $win_name $command" C-m
	else
		tmux send-keys -t "$SESSION:$win_name" \
			"cd \"$WS\" && export DISPLAY=${DISPLAY_VAR} LOG_DIR=\"$LOG_DIR\" ROS_LOG_DIR=\"$LOG_DIR/ros\" && $(source_overlay) && ${venv_part}$command"
	fi
	log "Pane $win_name: $command"
}

tmux new-session -d -s "$SESSION" -n "monitor"
tmux send-keys -t "$SESSION:monitor" "cd \"$WS\" && export DISPLAY=${DISPLAY_VAR} && $(source_overlay)" C-m
tmux send-keys -t "$SESSION:monitor" "echo 'SESSION=$SESSION CONTROLLER=$CONTROLLER EVENT=$EVENT CONES=$CONES CAN=$CAN IFACE=$CAN_IFACE'; echo \"LOG_DIR=$LOG_DIR\"; echo 'Watch: ros2 topic echo /mission_supervisor/status; ros2 topic hz /cone_array /path /ackermann_cmd /vcu2ai'; echo 'VCU keys: a=ASMS t=TSMS 4=AMI trackdrive (wait 5s AS_READY) g=RES go'; echo 'Control is gated by mission_supervisor until AS_DRIVING'; echo \"Attach: tmux attach -t $SESSION\""

# --- Sim ---
start_component "sim" 1 \
	"ros2 launch simulation dynamic_event.launch.py autostart:=true event:=${EVENT}"

source_overlay_here
if ! wait_for_topic "/clock" "$CLOCK_TIMEOUT_S"; then
	log "Gazebo did not publish /clock. See $LOG_DIR/sim.log"
	echo "Gazebo failed to start. Logs: $LOG_DIR/sim.log" >&2
	exit 1
fi
# Bridge advertises /odom before the model exists. Wait for a pose sample
# (spawn is delayed 12s in dynamic_event.launch.py).
log "Waiting up to 30s for an /odom message (vehicle spawn)"
if timeout 30 ros2 topic echo /odom --once >/dev/null 2>&1; then
	log "Got /odom (vehicle spawned)"
else
	log "ERROR: no /odom data — vehicle spawn likely failed. See $LOG_DIR/sim.log"
	echo "Vehicle did not spawn (/odom silent). Logs: $LOG_DIR/sim.log" >&2
	exit 1
fi

# --- Virtual CAN / VCU (manual AS arming) ---
if [ "$CAN" != "0" ]; then
	start_component "vcu" 1 \
		"python3 \"$WS/src/fsai_api/scripts/vcu_sim.py\" ${CAN_IFACE}"
	start_component "can" 1 \
		"ros2 run fsai_api ackermann_can ${CAN_IFACE}"
	start_component "wheels" 1 \
		"python3 \"$WS/src/fsai_api/scripts/wheel_speed_controller.py\"" \
		"venv"
	start_component "candump" 1 \
		"candump -td ${CAN_IFACE}"

	if [ "$CONTROLLER" = "mppi" ]; then
		CTRL_NODE="mppi_controller"
	else
		CTRL_NODE="pure_pursuit_controller"
	fi
	if [ "$CONES" = "perfect" ]; then
		PATH_NODE="perfect_path"
		REQUIRE_PERCEPTION="false"
	else
		PATH_NODE="track_pathfinder"
		REQUIRE_PERCEPTION="true"
	fi
	start_component "supervisor" 1 \
		"ros2 run mission_supervisor mission_supervisor --ros-args -p use_sim_time:=true -p controller_node:=${CTRL_NODE} -p path_node:=${PATH_NODE} -p require_perception:=${REQUIRE_PERCEPTION}"
	start_component "laps" 1 \
		"ros2 run mission_supervisor lap_counter --ros-args -p use_sim_time:=true"
	start_component "vcu2ai" 1 \
		"ros2 topic echo --no-arr /vcu2ai"
fi

# --- Path source ---
PATHFINDER_PARAMS="$WS/install/path_planning/share/path_planning/config/pathfinder_params.yaml"
[ -f "$PATHFINDER_PARAMS" ] || PATHFINDER_PARAMS="$WS/src/path_planning/config/pathfinder_params.yaml"

if [ "$CONES" = "perfect" ]; then
	start_component "path" 1 \
		"ros2 run simulation perfect_path --ros-args -p use_sim_time:=true -p track:=${EVENT}"
else
	start_component "yolo" 1 \
		"ros2 launch ultralytics_ros predict_with_cloud.launch.xml use_sim_time:=true sim:=true device:=cuda:0 yolo_model:=conev11n.pt" \
		"venv"
	start_component "path" 1 \
		"ros2 run path_planning pathfinder.py --ros-args -p use_sim_time:=true --params-file ${PATHFINDER_PARAMS}" \
		"venv"
fi

# --- Controller (one of) ---
MPPI_PARAMS="$WS/install/control/share/control/config/mppip.yaml"
PP_PARAMS="$WS/install/control/share/control/config/ppp.yaml"
[ -f "$MPPI_PARAMS" ] || MPPI_PARAMS="$WS/src/control/config/mppip.yaml"
[ -f "$PP_PARAMS" ] || PP_PARAMS="$WS/src/control/config/ppp.yaml"

LOCAL_FRAME_ARGS=""
if [ "$CONES" = "perfect" ] && [ "$CONTROLLER" = "mppi" ]; then
	LOCAL_FRAME_ARGS="-p use_local_frame:=false"
fi

REMAP=""
if [ "$CAN" != "0" ]; then
	# Supervisor is the only node allowed to publish /ackermann_cmd_controller.
	REMAP="-r /ackermann_cmd:=/ackermann_cmd_planner"
fi

if [ "$CONTROLLER" = "mppi" ]; then
	start_component "control" 1 \
		"ros2 run control mppi_ros_modified.py --ros-args -p use_sim_time:=true --params-file ${MPPI_PARAMS} ${LOCAL_FRAME_ARGS} ${REMAP}" \
		"venv"
else
	start_component "control" 1 \
		"ros2 run control pure_pursuit.py --ros-args -p use_sim_time:=true --params-file ${PP_PARAMS} ${LOCAL_FRAME_ARGS} ${REMAP}" \
		"venv"
fi

if [ "${VIZ:-0}" = "1" ]; then
	start_component "viz" 1 "rviz2"
fi

log "tmux session '$SESSION' started. Attach: tmux attach -t $SESSION"
echo "tmux session '$SESSION' started (EVENT=$EVENT CONES=$CONES CONTROLLER=$CONTROLLER CAN=$CAN)"
echo "Logs: $LOG_DIR"
echo "Master log: $LOGFILE"
if [ "$CAN" != "0" ]; then
	echo "VCU is AS_OFF until you arm it in the vcu pane: a, t, 4, wait 5s, g"
fi
if [ "${DETACH:-0}" = "1" ] || [ ! -t 1 ]; then
	echo "Detached. Attach with: tmux attach -t $SESSION"
	exit 0
fi

tmux attach -t "$SESSION"
