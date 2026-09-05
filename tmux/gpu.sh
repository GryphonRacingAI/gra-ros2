# shellcheck shell=bash
# Sourced by startup.sh. Probe torch CUDA; write $LOG_DIR/gpu.log
# with the nvidia_uvm reload recipe. Does not run sudo.
_GPU_SH_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

gpu_recover_text() {
	local ws="${WS:-/data/colcon_ws}"
	local helper="${_GPU_SH_DIR}/reload_nvidia_uvm.sh"
	cat <<EOF
GPU recover — nvidia_uvm wedged
================================
nvidia-smi can still list the GPU while torch.cuda.is_available() is False
("CUDA unknown error"). That is nvidia_uvm, not a missing driver.

Copy-paste in your own terminal (root is required to unload the module):

  cd ${ws}
  sudo rmmod nvidia_uvm
  sudo modprobe nvidia_uvm
  ${ws}/ros_venv/bin/python3 -c "import torch; assert torch.cuda.is_available(); torch.zeros(1, device='cuda'); print('cuda ok', torch.cuda.get_device_name(0))"

Then re-run ./tmux/startup.sh (or wait for predict_node respawn if the stack is up).

Helper: ${helper}
EOF
}

gpu_write_log() {
	local extra="${1:-}"
	if [ -z "${LOG_DIR:-}" ]; then
		return 0
	fi
	{
		echo "=== $(date -Iseconds) ==="
		echo "WS=${WS:-}"
		nvidia-smi 2>&1 || true
		echo
		if [ -n "$extra" ]; then
			echo "$extra"
			echo
		fi
		gpu_recover_text
	} >>"$LOG_DIR/gpu.log"
}

gpu_probe() {
	local py="${WS:-/data/colcon_ws}/ros_venv/bin/python3"
	[ -x "$py" ] || py=python3
	"$py" - <<'PY'
import sys
try:
    import torch
except Exception as e:
    print("torch import failed:", e)
    sys.exit(1)
ok = False
err = ""
try:
    ok = bool(torch.cuda.is_available())
    if ok:
        torch.zeros(1, device="cuda")
except Exception as e:
    ok = False
    err = f"{type(e).__name__}: {e}"
print("torch", getattr(torch, "__version__", "?"))
print("cuda_built", getattr(torch.version, "cuda", None))
print("is_available", torch.cuda.is_available())
print("device_count", torch.cuda.device_count())
if err:
    print("error", err)
sys.exit(0 if ok else 1)
PY
}

gpu_require() {
	local out
	if out=$(gpu_probe 2>&1); then
		gpu_write_log "probe: OK"$'\n'"$out"
		return 0
	fi
	gpu_write_log "probe: FAIL"$'\n'"$out"
	echo "$out" >&2
	gpu_recover_text >&2
	echo "Wrote $LOG_DIR/gpu.log" >&2
	return 1
}
