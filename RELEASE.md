# Release Notes

## v0.5.2 — Dora-MoveIt2 PPT Courseware

### New Features
- **PPT courseware generator** (`course-script/generate_ppt.py`): Python-pptx script that generates a 26-slide presentation from the Dora-MoveIt2 course content, with dark theme, comparison tables, code blocks, and architecture diagrams
- **PPT courseware** (`course-script/DoraMoveIt2_课件.pptx`): Ready-to-use 26-slide Chinese courseware covering all 8 chapters of the Dora-MoveIt2 course

## v0.5.1 — Dora-MoveIt2 Course Script

### New Features
- **Dora-MoveIt2 course script** (`course-script/DoraMoveIt2.md`): 8-chapter Chinese course script mirroring the ROS2 MoveIt2 version, adapted for dora-rs + GEN72. Covers environment setup, dataflow basics, robot config, MoveGroup API, single-arm planning, dual-arm coordination, and final demo with complete runnable code examples.

## v0.5.0 — Dual-Arm Coordination & Course Gap Analysis

### New Features
- **`course/` directory**: 8-chapter gap analysis mapping ROS2 MoveIt2 dual-arm manipulation course to dora-moveit2, with development roadmap
- **`DualArmConfig` protocol** (`config.py`): Optional multi-arm fields (`ARM_CHAINS`, `ARM_BASE_TRANSFORMS`, per-chain transforms/limits/poses) with backward-compatible helpers `is_dual_arm()` and `get_arm_config()`
- **Inter-arm collision detection** (`collision_lib.py`): `check_inter_arm_collision()` checks all left×right link pairs
- **14D motion planner** (`planner_ompl_with_collision_op.py`): Dual-arm mode via `{"mode": "dual_arm"}` in plan request, concatenated 14D joint space
- **Dual trajectory executor** (`trajectory_executor.py`): Detects dual-arm config, extracts per-chain joints, outputs 14D commands
- **Dual IK solver** (`ik_op.py`): Accepts JSON `{"left_pose": [6D], "right_pose": [6D]}` requests, solves per-arm, returns combined solution
- **`DualMoveGroup` API** (`workflow/dual_move_group.py`): High-level dual-arm interface with `go()`, `go_left()`, `go_right()`, `set_named_target()`, and scene access
- **Dual GEN72 example** (`examples/dual_gen72/`): Complete example with MuJoCo model (two GEN72 arms on table), config, dataflow, and pick-and-place demo

### Bug Fixes
- Fixed `collision_lib.py` `is_state_valid()` referencing non-existent `result.link1`/`result.link2`/`result.object_name` — corrected to `result.object_a`/`result.object_b`

### Library Changes
- `config.py`: Added `DualArmConfig` protocol, `is_dual_arm()`, `get_arm_config()` (backward compatible)
- `collision_lib.py`: Added `check_inter_arm_collision()` method, fixed attribute bugs
- `planning_scene_op.py`: `RobotState` gains `joint_positions_per_chain`, `update_robot_state()` gains `chain_name` param, dual-arm qpos extraction in `main()`
- `planner_ompl_with_collision_op.py`: Dual-arm mode support with 14D planning, `_setup_dual_robot()`, dynamic metadata
- `trajectory_executor.py`: Dual-arm initialization, per-chain joint extraction, 14D idle home config
- `ik_op.py`: JSON dual-arm request parsing alongside single-arm path

## v0.4.0 — LeKiwi Pick-and-Place Demo

### New Features
- **LeKiwi example** (`examples/lekiwi_pick_place/`): Full MoveGroup API demo for the SO_ARM100 6-DOF arm on LeKiwi omnidirectional mobile base
- **Third-party MJCF model**: Uses SIGRobotics-UIUC/LeKiwi-sim MuJoCo model (Apache 2.0) with modular `<attach>` composition
- **`ARM_ACTUATOR_START` config field**: Trajectory executor pads arm commands into full actuator array, supporting robots with non-arm actuators before arm actuators (e.g., wheel velocity actuators)
- **Top-down pick-and-place**: Ball and plate on pedestals at arm-reachable height; gripper approaches from above, lowers to grasp/place, lifts back up — natural motion path

### Model Details
- 3 omni wheel velocity actuators + 6 arm position actuators (9 total)
- 2 cameras (front + wrist)
- qpos layout: freejoint[0:7], wheels[7:10], arm[10:16]
- Ball on pedestal at [0.30, 0, 0.174], plate on pedestal at [0.30, -0.18, 0.153]

### Library Changes
- `trajectory_executor.py`: Added `_pad_arm_command()` helper and `ARM_ACTUATOR_START`/`NUM_ACTUATORS` config support (backward-compatible)

## v0.3.8 — Ball & Plate Farther from Body, Doubled Ball Size

### Changes
- **Ball far front-left**: Repositioned to [-0.04, 0.25, 0.06] — much farther from body
- **Plate far front-right**: Repositioned to [0.10, 0.25, 0.04] — ~14cm separation from ball
- **Ball doubled in size**: Sphere radius increased from 0.012 to 0.024
- **Updated joint configs**: Pick q=[-0.069, 1.158, -1.553, 1.5, 0, 0], Place q=[0.621, 2.0, -1.868, -0.136, 0, 0]

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
