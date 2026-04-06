# Release Notes

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
