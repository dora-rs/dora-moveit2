# Chapter 5: Python MoveGroup API

## ROS2 vs Dora Comparison

| Feature | ROS2 MoveIt2 | Dora-MoveIt2 | Status |
|---------|-------------|--------------|--------|
| MoveGroupCommander | `moveit_commander.MoveGroupCommander` | `MoveGroup` class | ✅ |
| Named target | `set_named_target("home")` | `set_named_target("home")` | ✅ |
| Joint goal | `go(joint_goal)` | `go(joint_goal)` | ✅ |
| Pose target | `set_pose_target(pose)` | `set_pose_target(pose)` | ✅ |
| Cartesian path | `compute_cartesian_path()` | `compute_cartesian_path()` | ✅ |
| Planning scene | `PlanningSceneInterface` | `get_planning_scene_interface()` | ✅ |
| Plan/execute split | `plan()` + `execute()` | `plan()` + `execute()` | ✅ |
| FK/IK | `compute_fk()` / `compute_ik()` | `compute_fk()` / `compute_ik()` | ✅ |
| Stop motion | `stop()` | `stop()` | ✅ |

## API Parity

Dora-MoveIt2's `MoveGroup` class provides all 5 core MoveIt features:

```python
from dora_moveit.workflow.move_group import MoveGroup

group = MoveGroup("gen72")
scene = group.get_planning_scene_interface()

# 1. Named pose
group.set_named_target("home")
group.go(wait=True)

# 2. Joint-space goal
group.go([1.57, -0.785, 0.0, -1.57, 0.0, 0.785, 0.0], wait=True)

# 3. Cartesian pose goal
group.set_pose_target([0.15, 0.1, 0.6, 0, 0, 0])
group.go(wait=True)

# 4. Cartesian path
waypoints = [[0.15, 0.1, 0.5, 0, 0, 0]]
trajectory, fraction = group.compute_cartesian_path(waypoints, eef_step=0.01)
group.execute(trajectory, wait=True)

# 5. Collision objects
scene.add_box("obstacle", [0.4, 0.0, 0.5], [0.1, 0.1, 0.5])
group.set_named_target("home")
group.go(wait=True)
scene.remove_world_object("obstacle")

group.shutdown()
```

## No Gaps for Single-Arm

The single-arm MoveGroup API has full feature parity with ROS2 MoveIt2's Python interface.

## Code Reference

- `dora_moveit/workflow/move_group.py` — MoveGroup + PlanningSceneInterface
