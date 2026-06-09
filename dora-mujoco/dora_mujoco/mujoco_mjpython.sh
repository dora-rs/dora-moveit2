#!/bin/bash
# Wrapper script to run MuJoCo simulation with mjpython (required on macOS)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec /opt/miniconda3/envs/dorobot/bin/mjpython "$SCRIPT_DIR/main.py" "$@"
