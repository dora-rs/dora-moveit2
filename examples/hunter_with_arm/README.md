# Hunter SE + GEN72 Arm Demo

Example application using dora-moveit library for the Hunter SE mobile robot with a GEN72 7-DOF arm.

## Setup

```bash
# Install the dora-moveit library (from repo root)
pip install -e dora_moveit/

# Install this example app
pip install -e examples/hunter_with_arm/

# Or add to PYTHONPATH:
export PYTHONPATH=/path/to/dora_moveit:/path/to/examples/hunter_with_arm:$PYTHONPATH
```

## Run

```bash
cd examples/hunter_with_arm

# MoveGroup API demo (recommended)
dora up
dora start dataflows/movegroup_mujoco.yml

# Choreographed multi-view capture
dora start dataflows/hunter_arm_mujoco.yml
```

## Structure

```
hunter_arm_demo/
  config/gen72.py          - GEN72 robot configuration
  nodes/
    example_movegroup.py   - MoveGroup API example script
    multi_view_capture_node.py - Choreographed capture workflow
    vehicle_controller.py  - Hunter SE vehicle control
    planner.py             - Wrapper for dora-moveit planner
    ik_solver.py           - Wrapper for dora-moveit IK solver
    planning_scene.py      - Wrapper for dora-moveit scene manager
    trajectory_executor.py - Wrapper for dora-moveit executor
  robot_control/           - GEN72 physical robot control (Realman SDK)
models/                    - MuJoCo XML models, URDF, meshes
dataflows/                 - Dora dataflow YAML configurations
```

Library operators are used via thin wrappers that import from `dora_moveit.*`.
Robot config is injected via `ROBOT_CONFIG_MODULE` environment variable in each dataflow YAML.
