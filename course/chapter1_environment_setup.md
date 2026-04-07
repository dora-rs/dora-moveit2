# Chapter 1: Environment Setup

## ROS2 MoveIt2 vs Dora-MoveIt2

| Feature | ROS2 MoveIt2 | Dora-MoveIt2 | Status |
|---------|-------------|--------------|--------|
| Framework install | `apt install ros-humble-moveit` | `pip install -e dora_moveit/` | ✅ |
| Simulation | Gazebo / MuJoCo plugin | MuJoCo via dora-mujoco | ✅ |
| Visualization | RViz2 | MuJoCo viewer (built-in) | ✅ |
| Build system | colcon / CMake | pip / setuptools | ✅ |
| Launch system | ros2 launch (XML/Python) | `dora start <yaml>` | ✅ |
| Package management | rosdep / apt | pip / PyPI | ✅ |

## What's Covered

- Python-only install (no C++ compilation)
- MuJoCo simulation with viewer
- Dora dataflow YAML replaces ROS2 launch files
- Examples install with `pip install -e examples/<name>/`

## Gaps

- **No RViz equivalent**: MuJoCo viewer shows robot but no interactive markers, planning visualizations, or TF frames overlay
- **No rosbag equivalent**: No built-in data recording/playback (dora has logging but not the same)

## Code Reference

```bash
# Dora-MoveIt2 setup
pip install -e dora_moveit/
pip install -e examples/move_group_demo/
cd examples/move_group_demo
dora up && dora start dataflows/moveit_example_mujoco.yml
```
