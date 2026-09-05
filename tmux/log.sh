# shellcheck shell=bash
# Sourced by startup.sh (and each tmux pane).
# log_setup  — mkdir ~/colcon_ws/logs/<stamp>, export LOG_DIR
# run_logged <name> <cmd...>  — unbuffered cmd, wall-clock stamp, tee to pane + $LOG_DIR/<name>.log

log_setup() {
	local root="${WS:-${COLCON_WS:-$HOME/colcon_ws}}"
	root="$(cd "$root" && pwd)"
	mkdir -p "$root/logs"
	if [ -z "${LOG_DIR:-}" ]; then
		LOG_DIR="$root/logs/$(date +%Y%m%d_%H%M%S)"
	fi
	mkdir -p "$LOG_DIR/ros"
	ln -sfn "$(basename "$LOG_DIR")" "$root/logs/latest"
	export LOG_DIR
	echo "[log_setup] $LOG_DIR" >&2
}

# One pipeline: node stdout/stderr → stamp → tmux pane and file.
# Stamp is wall-clock epoch + per-file line counter (join key across nodes).
run_logged() {
	local name="$1"
	shift
	if [ -z "${LOG_DIR:-}" ]; then
		echo "run_logged: LOG_DIR unset (source log.sh and log_setup first)" >&2
		return 1
	fi
	local log="$LOG_DIR/${name}.log"
	export PYTHONUNBUFFERED=1
	stdbuf -oL -eL "$@" 2>&1 | python3 -u -c '
import sys, time
n = 0
for line in sys.stdin:
    n += 1
    sys.stdout.write(f"{time.time():.6f} #{n} {line}")
    sys.stdout.flush()
' | tee -a "$log"
}
