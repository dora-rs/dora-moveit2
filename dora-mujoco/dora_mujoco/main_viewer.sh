#!/usr/bin/env bash
# Wrapper to launch the MuJoCo node with mjpython (required for viewer on macOS).
# Usage in dora YAML:
#   path: dora-mujoco/dora_mujoco/main_viewer.sh
DIR="$(cd "$(dirname "$0")" && pwd)"
exec mjpython "$DIR/main.py" "$@"
