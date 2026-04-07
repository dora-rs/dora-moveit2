# Chapter 3: Dual-Arm URDF / Robot Configuration

## ROS2 vs Dora Approach

| Feature | ROS2 MoveIt2 | Dora-MoveIt2 | Status |
|---------|-------------|--------------|--------|
| Robot description | URDF/XACRO | Python config class | ✅ Single arm |
| Joint limits | URDF `<limit>` tags | `JOINT_LOWER_LIMITS` / `JOINT_UPPER_LIMITS` | ✅ |
| Collision geometry | URDF `<collision>` meshes | `COLLISION_GEOMETRY` tuples | ✅ |
| Named poses | SRDF `<group_state>` | `NAMED_POSES` dict | ✅ |
| Dual-arm description | XACRO macros × 2 | `DualArmConfig` protocol | ❌ New |
| Planning groups | SRDF `<group>` | `ARM_CHAINS` list | ❌ New |
| Base transforms | TF tree / URDF fixed joints | `ARM_BASE_TRANSFORMS` dict | ❌ New |

## Dora-MoveIt2 Config Protocol

Single-arm configs implement `RobotConfig` protocol (see `dora_moveit/config.py`).

For dual-arm, configs additionally provide:
- `ARM_CHAINS: List[str]` — e.g. `["left_arm", "right_arm"]`
- `ARM_BASE_TRANSFORMS: Dict[str, Dict]` — base frame per arm
- `LINK_TRANSFORMS_PER_CHAIN: Dict[str, List[Dict]]` — FK chain per arm
- `HOME_CONFIG_PER_CHAIN: Dict[str, np.ndarray]`

## Gap: No XACRO Equivalent

ROS2 uses XACRO macros to instantiate the same arm twice with different prefixes. Dora-MoveIt2's Python config achieves the same by reusing data structures per chain.

## Code Reference

```python
# examples/dual_gen72/dual_gen72_demo/config/dual_gen72.py
class DualGEN72Config:
    ARM_CHAINS = ["left_arm", "right_arm"]
    NUM_JOINTS = 7  # per arm
    ARM_BASE_TRANSFORMS = {
        "left_arm": {"xyz": [-0.3, 0, 0.8], "rpy": [0, 0, 0]},
        "right_arm": {"xyz": [0.3, 0, 0.8], "rpy": [0, 0, 3.14159]},
    }
```
