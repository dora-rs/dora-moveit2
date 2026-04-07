# Chapter 7: Dual-Arm Coordination

**This is the largest gap between ROS2 MoveIt2 and Dora-MoveIt2.**

## ROS2 vs Dora Comparison

| Feature | ROS2 MoveIt2 | Dora-MoveIt2 | Status |
|---------|-------------|--------------|--------|
| Multi-group planning | `MoveGroupInterface` per group | `DualMoveGroup` | ❌ New |
| Synchronized motion | Action server coordination | 14D unified planner | ❌ New |
| Inter-arm collision | FCL + ACM | `check_inter_arm_collision()` | ❌ New |
| Independent arm motion | Separate planning groups | `go_left()` / `go_right()` | ❌ New |
| Dual Cartesian path | Per-group Cartesian planning | `compute_dual_cartesian_path()` | ❌ New |
| Object handoff | Attach/detach across groups | Scene tracks per-arm attachment | ❌ New |

## Development Items

### 1. DualArmConfig Protocol (config.py)
- `ARM_CHAINS`, `ARM_BASE_TRANSFORMS`, per-chain transforms/limits/poses
- `is_dual_arm(config)` helper
- Backward compatible with single-arm configs

### 2. Inter-Arm Collision Detection (collision_lib.py)
- `check_inter_arm_collision()` method
- Checks all left × right link pairs (skip base-adjacent)

### 3. 14D Motion Planner (planner_ompl_with_collision_op.py)
- Detect dual-arm mode from `{"mode": "dual_arm"}` in plan request
- Concatenated 14D joint space (left 7D + right 7D)
- `is_state_valid()` checks self-collision per arm + inter-arm collision

### 4. Dual Trajectory Executor (trajectory_executor.py)
- Detect 14D trajectories
- Split into left 7D + right 7D commands
- Output 14 actuator values

### 5. Dual IK Solver (ik_op.py)
- Accept `{"left_pose": [6D], "right_pose": [6D]}` requests
- Solve each arm independently
- Return `{"left_joints": [7D], "right_joints": [7D]}`

### 6. DualMoveGroup API (dual_move_group.py)
```python
from dora_moveit.workflow.dual_move_group import DualMoveGroup

group = DualMoveGroup()

# Synchronized motion
group.go(left_joints=[...], right_joints=[...])

# Independent motion
group.go_left([...])
group.go_right([...])

# Named poses
group.set_named_target(left_name="home", right_name="home")
group.go()
```

## Demo Sequence

1. Both arms go home (synchronized)
2. Left arm picks up ball
3. Right arm moves to receive position
4. Handoff: left places, right picks
5. Right arm places on plate
6. Both arms return home
