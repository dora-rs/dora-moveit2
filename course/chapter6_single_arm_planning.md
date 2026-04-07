# Chapter 6: Single-Arm Motion Planning

## ROS2 vs Dora Comparison

| Feature | ROS2 MoveIt2 | Dora-MoveIt2 | Status |
|---------|-------------|--------------|--------|
| RRT-Connect | OMPL library | Custom implementation | ✅ |
| RRT | OMPL | Custom implementation | ✅ |
| RRT* | OMPL | Custom implementation | ✅ |
| PRM | OMPL | Planned | ⚠️ |
| Collision checking | FCL library | Custom geometric checker | ✅ |
| Self-collision | ACM-filtered | Skip-adjacent pairs | ✅ |
| Environment collision | FCL broadphase + narrowphase | Direct sphere/box/cylinder | ✅ |
| Planning scene | MoveIt PlanningScene | PlanningSceneOperator | ✅ |
| Trajectory smoothing | Time-optimal parameterization | Shortcut smoothing | ✅ |
| Trajectory execution | FollowJointTrajectory action | TrajectoryExecutor operator | ✅ |

## Architecture

```
User (MoveGroup API)
  → plan_request → Planner (RRT-Connect)
  → trajectory → Executor (interpolation)
  → joint_commands → MuJoCo (simulation)
  → joint_positions → Planning Scene (state tracking)
```

## Code Reference

- `dora_moveit/motion_planner/planner_ompl_with_collision_op.py` — OMPL planner
- `dora_moveit/collision_detection/collision_lib.py` — Collision checking
- `dora_moveit/trajectory_execution/trajectory_executor.py` — Trajectory execution
- `dora_moveit/motion_planner/planning_scene_op.py` — Scene management
