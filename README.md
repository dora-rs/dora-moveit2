# dora-moveit2

A reusable robot motion planning library for the [dora-rs](https://github.com/dora-rs/dora) dataflow framework. Provides MoveIt-style IK solving, collision-aware OMPL planning, trajectory execution, and a high-level MoveGroup API — for any robot arm.

## Features

- **MoveGroup API** — High-level interface similar to ROS MoveIt's MoveGroupCommander
- **IK Solver** — TracIK-inspired multi-start inverse kinematics
- **Motion Planner** — RRT-Connect (OMPL) with geometric collision detection
- **Planning Scene** — Obstacle management and scene state broadcasting
- **Trajectory Executor** — Quintic spline interpolation with smooth execution
- **Robot-agnostic** — Bring your own robot config via `ROBOT_CONFIG_MODULE` env var

## Quick Start

```bash
# Install the library
pip install -e dora_moveit/

# Install an example app
pip install -e examples/move_group_demo/

# Run the ROS MoveIt-style example (GEN72 arm in MuJoCo)
cd examples/move_group_demo
dora up
dora start dataflows/moveit_example_mujoco.yml
```

## Examples

| Example | Robot | Description |
|---------|-------|-------------|
| [move_group_demo](examples/move_group_demo/) | GEN72 7-DOF | ROS MoveIt-style API demo — named poses, joint goals, Cartesian paths, collision objects |
| [hunter_with_arm](examples/hunter_with_arm/) | Hunter SE + GEN72 | Mobile robot with arm for multi-view inspection |

See the [examples README](examples/README.md) for details and a guide to creating your own.

## Repository Structure

```
dora_moveit/           # Library package (pip install -e dora_moveit/)
  dora_moveit/
    config.py          # RobotConfig protocol + load_config()
    ik_solver/         # IK solver operators
    motion_planner/    # OMPL planner + planning scene
    trajectory_execution/  # Trajectory executor
    collision_detection/   # Collision checking
    workflow/          # MoveGroup API + motion commander
examples/              # Example applications
  move_group_demo/     # ROS MoveIt-style MoveGroup API example
  hunter_with_arm/     # Hunter SE mobile robot + GEN72 7-DOF arm
dora-mujoco/           # MuJoCo simulation node
```

## Usage

### MoveGroup API (ROS MoveIt equivalent)

```python
from dora_moveit.workflow.move_group import MoveGroup

group = MoveGroup("gen72")
scene = group.get_planning_scene_interface()

# 1. Named pose
group.set_named_target("home")
group.go(wait=True)

# 2. Joint-space goal
group.go([1.57, -0.785, 0.0, -1.57, 0.0, 0.785, 0.0], wait=True)

# 3. Cartesian pose goal (IK solved internally)
group.set_pose_target([0.15, 0.1, 0.6, 0, 0, 0])
group.go(wait=True)

# 4. Cartesian path (straight line in workspace)
waypoints = [[0.15, 0.1, 0.5, 0, 0, 0], [0.15, 0.2, 0.5, 0, 0, 0]]
trajectory, fraction = group.compute_cartesian_path(waypoints, eef_step=0.01)
group.execute(trajectory, wait=True)

# 5. Collision objects
scene.add_box("obstacle", [0.4, 0.0, 0.5], [0.1, 0.1, 0.5])
group.set_named_target("home")
group.go(wait=True)  # planner avoids the box
scene.remove_world_object("obstacle")

group.shutdown()
```

### Robot Config Injection

Library operators are robot-agnostic. Define a config class for your robot:

```python
# my_app/config/my_robot.py
import numpy as np

class MyRobotConfig:
    NUM_JOINTS = 6
    JOINT_LOWER_LIMITS = np.array([...])
    JOINT_UPPER_LIMITS = np.array([...])
    LINK_TRANSFORMS = [...]
    COLLISION_GEOMETRY = [...]
    HOME_CONFIG = np.array([...])
    SAFE_CONFIG = np.array([...])
    NAMED_POSES = {"home": np.array([...])}
```

Set the env var in your dataflow YAML:

```yaml
- id: planner
  path: ../my_app/nodes/planner.py
  env:
    ROBOT_CONFIG_MODULE: "my_app.config.my_robot"
```

See [`dora_moveit/dora_moveit/config.py`](dora_moveit/dora_moveit/config.py) for the full `RobotConfig` protocol and [`examples/`](examples/) for a complete working example.

## Adding a New Robot

See the [examples README](examples/README.md) for a step-by-step guide to creating a new example application with your own robot.

## License

MIT
