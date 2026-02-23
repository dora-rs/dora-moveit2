# Examples

Example applications built with the [dora-moveit](../dora_moveit/) library.

## Available Examples

| Example | Robot | Description |
|---------|-------|-------------|
| [move_group_demo](move_group_demo/) | GEN72 7-DOF | ROS MoveIt-style API demo (named poses, joint goals, Cartesian paths, collision objects) |
| [hunter_with_arm](hunter_with_arm/) | Hunter SE + GEN72 7-DOF | Mobile robot with arm for multi-view inspection |

## Creating a New Example

Each example is a self-contained application that imports `dora_moveit` as a library.

### Minimal structure

```
examples/
  my_robot/
    pyproject.toml              # depends on dora-moveit
    my_robot_demo/
      __init__.py
      config/
        __init__.py
        my_robot.py             # RobotConfig class (joints, limits, geometry)
      nodes/
        __init__.py
        planner.py              # thin wrapper: from dora_moveit.motion_planner... import main; main()
        ik_solver.py            # thin wrapper
        planning_scene.py       # thin wrapper
        trajectory_executor.py  # thin wrapper
        my_app_node.py          # your application logic
    models/                     # URDF, MuJoCo XML, meshes
    dataflows/                  # Dora dataflow YAMLs
```

### Key steps

1. **Define your robot config** — create a class satisfying the `RobotConfig` protocol (see `dora_moveit/dora_moveit/config.py`):
   ```python
   class MyRobotConfig:
       NUM_JOINTS = 6
       JOINT_LOWER_LIMITS = np.array([...])
       JOINT_UPPER_LIMITS = np.array([...])
       LINK_TRANSFORMS = [...]
       COLLISION_GEOMETRY = [...]
       HOME_CONFIG = np.array([...])
       SAFE_CONFIG = np.array([...])
       NAMED_POSES = {"home": np.array([...])}
       # ... see RobotConfig protocol for full list
   ```

2. **Create thin wrapper scripts** for library operators:
   ```python
   # my_robot_demo/nodes/planner.py
   from dora_moveit.motion_planner.planner_ompl_with_collision_op import main
   main()
   ```

3. **Set `ROBOT_CONFIG_MODULE`** in your dataflow YAML:
   ```yaml
   - id: planner
     path: ../my_robot_demo/nodes/planner.py
     env:
       ROBOT_CONFIG_MODULE: "my_robot_demo.config.my_robot"
   ```

4. **Install and run**:
   ```bash
   pip install -e dora_moveit/
   pip install -e examples/my_robot/
   cd examples/my_robot
   dora up && dora start dataflows/my_dataflow.yml
   ```

See [hunter_with_arm](hunter_with_arm/) for a complete working example.
