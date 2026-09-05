#!/bin/bash
# Diagnose wedged nvidia_uvm (nvidia-smi ok, torch CUDA dead).
# Does not run sudo. Prints the two commands you run yourself, then
# how to check torch again. Writes $LOG_DIR/gpu.log when LOG_DIR is set.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
export WS="${WS:-$ROOT}"
# shellcheck source=gpu.sh
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/gpu.sh"
if [ -n "${LOG_DIR:-}" ]; then
	mkdir -p "$LOG_DIR"
fi

echo "nvidia-smi:"
nvidia-smi -L 2>&1 || true
echo
echo "torch probe:"
if gpu_probe; then
	echo "CUDA is usable. No module reload needed."
	gpu_write_log "reload helper: CUDA already ok"
	exit 0
fi
echo
gpu_recover_text
gpu_write_log "reload helper: CUDA dead, printed recover recipe"
exit 1
