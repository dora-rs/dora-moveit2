# Chapter 4: MoveIt Setup Assistant / Planning Groups

## ROS2 vs Dora Approach

| Feature | ROS2 MoveIt2 | Dora-MoveIt2 | Status |
|---------|-------------|--------------|--------|
| Setup assistant GUI | MoveIt Setup Assistant | N/A (Python config) | ⚠️ No GUI |
| Planning group definition | SRDF `<group>` | `ARM_CHAINS` in config | ❌ New |
| Self-collision matrix (ACM) | Auto-generated | `_compute_self_collision_pairs()` | ✅ |
| Inter-arm collision | ACM + planning group | `check_inter_arm_collision()` | ❌ New |
| End-effector definition | SRDF `<end_effector>` | Implicit (last link in chain) | ✅ |

## Planning Groups in Dora-MoveIt2

ROS2 MoveIt2 defines planning groups in SRDF:
```xml
<group name="left_arm">
  <chain base_link="left_base" tip_link="left_ee"/>
</group>
```

Dora-MoveIt2 defines groups via config:
```python
ARM_CHAINS = ["left_arm", "right_arm"]
ARM_BASE_TRANSFORMS = {...}
LINK_TRANSFORMS_PER_CHAIN = {...}
```

## Gap: No GUI Setup Tool

MoveIt Setup Assistant provides a GUI for configuring planning groups, collision matrices, and named poses. Dora-MoveIt2 requires manual Python config authoring. This is acceptable for developers but less accessible for new users.

## Gap: Multi-Group Planning

ROS2 MoveIt2 supports planning for multiple groups simultaneously. Dora-MoveIt2 needs:
- 14D unified planner (7 joints × 2 arms)
- Inter-arm collision checking
- `DualMoveGroup` API
