# Release Notes

## v0.3.7 — Nano Pick-and-Place with Joint-Space Goals

### Changes
- **Joint-space pick-and-place**: Rewrote `nano_pick_place_example.py` to use direct joint goals instead of IK pose targets, ensuring reliable reach to ball and plate positions
- **Red ball under gripper**: Repositioned target ball to [-0.025, 0.17, 0.055] directly under the arm's reachable workspace
- **Green place plate**: Added green cylinder at [0.10, 0.09, 0.04] as the placement target
- **FK-scanned joint configs**: Pick config q=[0.13, 0.39, -1.18, 1.5, 0, 0], Place config q=[-1.97, 0.13, -1.45, 0, 0, 0]

## v0.3.6 — Nano Pick-and-Place Demo

### Changes
- **Pick-and-place demo**: Rewrote `nano_pick_place_example.py` as full 10-step sequence (approach, lower, grasp, lift, move, place, release, retreat) using IK-reachable positions
- **Robot body raised**: Added z_offset=0.035 in URDF→MJCF conversion so wheels just touch the ground plane

## v0.3.5 — Nano Full Robot Model (Mobile Base + Arm)

### New Features
- **Full ADORA1 Nano MJCF model** (`nano_full.xml`): Complete 3-wheel omnidirectional mobile base + SO_ARM100 6-DOF arm with all 42 mesh geometries, auto-converted from ground-truth URDF
- **URDF→MJCF conversion script** (`scripts/convert_urdf_to_mjcf.py`): Automated pipeline using MuJoCo's URDF compiler with mesh decimation for oversized STLs and post-processing for actuators/scene setup
- **`ARM_QPOS_START` config field**: Robot configs can now specify where arm joints start in the qpos array, supporting robots with mobile bases (wheels before arm joints)

### Library Changes
- `trajectory_executor.py`: Uses `ARM_QPOS_START` from config for arm joint extraction (backward-compatible with Hunter model)
- `planning_scene_op.py`: Same `ARM_QPOS_START` support for robot state updates
- `move_group.py`: `_extract_arm_joints()` accepts `arm_qpos_start` parameter

### Model Details
- 45 STL meshes (3 omni wheels decimated from 314K→199K faces for MuJoCo's 200K limit)
- 3 passive wheel joints (continuous, no actuators) + 6 arm revolute joints (position actuators)
- qpos layout: wheels[0:3], arm[3:9], target_freejoint[9:16]

## v0.3.0 — Nano Pick-and-Place + Arbitrary Axis Support

### New Features
- **ADORA1 Nano example** (`examples/nano_pick_place/`): Full MoveGroup API demo for the SO_ARM100 6-DOF arm with MuJoCo simulation
- **Generalized rotation axes**: IK solver now supports arbitrary joint rotation axes via Rodrigues' formula (not just pure X/Y/Z), enabling robots with angled joints like the Nano's wrist

### Library Changes
- `advanced_ik_solver.py`: `_rot_axis()` upgraded from pure-axis dispatch to Rodrigues' formula with cardinal fast path
- `ik_op.py`: Same generalization for `NumericalIKSolver.forward_kinematics()`

### Nano Example Contents
- `NanoConfig` with per-joint axis vectors (including angled wrist joints 5 & 6)
- Standalone arm-only MuJoCo XML model with STL meshes
- 5-step MoveGroup demo: named pose, joint goal, Cartesian pose, Cartesian path, collision avoidance
- Dora dataflow YAML with all library operators

## v0.2.0 — UR5e Pick-and-Place + 6-DOF Generalization

### New Features
- UR5e + Robotiq 2F-85 example with pick-and-place demo
- Library generalized for variable DOF (no more hardcoded 7)
- Per-joint rotation axes in LINK_TRANSFORMS
- EE_OFFSET support in FK/IK

## v0.1.0 — Initial Release

- GEN72 7-DOF arm support
- MoveGroup API (5 core ROS MoveIt features)
- OMPL RRT-Connect motion planner with collision detection
- TracIK-inspired multi-strategy IK solver
- MuJoCo simulation integration via dora-rs dataflow
