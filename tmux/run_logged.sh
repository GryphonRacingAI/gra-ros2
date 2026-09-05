h#!/bin/bash
# Run a command with unbuffered stdout/stderr, wall-clock stamps, and tee.
#
#   run_logged.sh <logfile> -- <command> [args...]
#
# Pipeline:
#   stdbuf -o0 -e0 cmd 2>&1 | stamp_log.py | tee -a logfile
#
# -o0/-e0: no libc block buffering (line-buffering would hold \\r status).
# python3 -u + stdout.flush: stamp_log writes each line to disk via tee
# immediately. tmux pane sees the same stream as the file.

set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [ "$#" -lt 1 ]; then
	echo "usage: run_logged.sh <logfile> -- <command> [args...]" >&2
	exit 2
fi

LOGFILE="$1"
shift
if [ "${1:-}" = "--" ]; then
	shift
fi
if [ "$#" -lt 1 ]; then
	echo "usage: run_logged.sh <logfile> -- <command> [args...]" >&2
	exit 2
fi

mkdir -p "$(dirname "$LOGFILE")"
export PYTHONUNBUFFERED=1

# Prefer stdbuf; fall back to unbuffered python/tee only if missing.
if command -v stdbuf >/dev/null 2>&1; then
	stdbuf -o0 -e0 "$@" 2>&1 \
		| stdbuf -o0 python3 -u "$ROOT/stamp_log.py" \
		| stdbuf -o0 tee -a "$LOGFILE"
else
	"$@" 2>&1 \
		| python3 -u "$ROOT/stamp_log.py" \
		| tee -a "$LOGFILE"
fi
