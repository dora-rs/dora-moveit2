# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Dora-MoveIt is a **reusable robotics motion planning library** built on the **dora-rs** dataflow framework. It provides IK solving, collision-aware motion planning, trajectory execution, and a high-level MoveGroup API (similar to ROS MoveIt). It works with any robot arm — robot-specific configuration is injected via the `ROBOT_CONFIG_MODULE` environment variable.

## Repository Structure

```
dora-moveit/                     # Repository root
├── dora_moveit/                 # LIBRARY PACKAGE (pip install -e .)
│   ├── dora_moveit/             # Python package
│   │   ├── config.py            # RobotConfig protocol + load_config()
│   │   ├── ik_solver/           # IK solver operators
│   │   ├── motion_planner/      # OMPL planner + planning scene
│   │   ├── trajectory_execution/# Trajectory executor
│   │   ├── collision_detection/ # Collision checking library
│   │   └── workflow/            # MoveGroup API + motion commander
│   └── pyproject.toml
├── examples/
│   └── hunter_with_arm/         # EXAMPLE APP (Hunter SE + GEN72)
│       ├── hunter_arm_demo/     # App Python package
│       │   ├── config/gen72.py  # GEN72Config class
│       │   ├── nodes/           # App operators + library wrappers
│       │   └── robot_control/   # GEN72 physical robot (Realman SDK)
│       ├── models/              # MuJoCo XML, URDF, meshes
│       └── dataflows/           # Dora dataflow YAMLs
├── dora-mujoco/                 # MuJoCo simulation package
├── dora-moveit/                 # (legacy, being replaced by dora_moveit/)
└── dora-control-lebai/          # LM3 robot (separate, future migration)
```

## Running

```bash
# Install library
pip install -e dora_moveit/
# Or: add dora_moveit/ and examples/hunter_with_arm/ to PYTHONPATH

# Run example
cd examples/hunter_with_arm
dora up
dora start dataflows/movegroup_mujoco.yml   # MoveGroup API demo
dora start dataflows/hunter_arm_mujoco.yml  # Choreographed capture
dora stop
```

## Key Design: Robot Config Injection

Library operators are robot-agnostic. Robot config is loaded at runtime via:

```python
from dora_moveit.config import load_config
config = load_config()  # reads ROBOT_CONFIG_MODULE env var
```

Each dataflow YAML sets the env var per operator:
```yaml
env:
  ROBOT_CONFIG_MODULE: "hunter_arm_demo.config.gen72"
```

The config class must satisfy the `RobotConfig` protocol (see `dora_moveit/config.py`): `NUM_JOINTS`, `JOINT_LOWER_LIMITS`, `JOINT_UPPER_LIMITS`, `LINK_TRANSFORMS`, `COLLISION_GEOMETRY`, `HOME_CONFIG`, `SAFE_CONFIG`, `NAMED_POSES`, etc.

## MoveGroup API (High-Level)

```python
from dora_moveit.workflow.move_group import MoveGroup

group = MoveGroup("gen72")
group.set_named_target("home")
group.go(wait=True)
group.go([-1.57, -0.5, 0.0, 0.0, 0.0, 0.5, 0.0], wait=True)
success, traj = group.plan(joint_goal)
group.execute(traj, wait=True)
group.shutdown()
```

Single-threaded polling design (all dora Node ops on one thread to avoid "Already borrowed" errors).

## Library Operators

| Operator | Package Path | Role |
|----------|-------------|------|
| **ik_solver** | `dora_moveit.ik_solver.ik_op` | TracIK-inspired multi-start IK |
| **planner** | `dora_moveit.motion_planner.planner_ompl_with_collision_op` | RRT-Connect with collision |
| **planning_scene** | `dora_moveit.motion_planner.planning_scene_op` | Scene state manager |
| **trajectory_executor** | `dora_moveit.trajectory_execution.trajectory_executor` | Quintic interpolation |
| **collision_lib** | `dora_moveit.collision_detection.collision_lib` | Geometric collision checking |

Example apps use **thin wrapper scripts** to run library operators:
```python
# hunter_arm_demo/nodes/planner.py
from dora_moveit.motion_planner.planner_ompl_with_collision_op import main
main()
```

## Code Patterns

- **Operator structure**: Each operator has a `main()` function with dora `Node` event loop. Inputs: PyArrow arrays. Outputs: `pa.array()`.
- **Config loading**: `from dora_moveit.config import load_config` — cached per-process.
- **Package imports**: Library code uses `from dora_moveit.xxx import ...` (no `sys.path` hacks).
- **Hunter SE joint array**: MuJoCo sends 20 qpos (freejoint+arm); arm joints at indices 13:20.
