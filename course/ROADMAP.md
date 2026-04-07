# Dora-MoveIt2 Dual-Arm Development Roadmap

## Phase 1: Foundation (v0.5.0) — Current Release

- [x] `DualArmConfig` protocol extension in `config.py`
- [x] Inter-arm collision detection in `collision_lib.py`
- [x] 14D unified motion planner
- [x] Dual trajectory executor
- [x] Dual IK solver
- [x] `DualMoveGroup` API
- [x] Dual GEN72 MuJoCo model and example
- [x] Course gap analysis (this directory)

## Phase 2: Coordination Primitives (v0.6.0)

- [ ] Synchronized dual Cartesian paths
- [ ] Master-slave mode (one arm follows the other with offset)
- [ ] Grasp planning with approach/retreat vectors
- [ ] Object handoff protocol (attach left → detach left → attach right)
- [ ] Impedance control for contact tasks

## Phase 3: Task-Level Planning (v0.7.0)

- [ ] Task and Motion Planning (TAMP) integration
- [ ] Symbolic action sequences (pick, place, handoff)
- [ ] Constraint-based motion (keep object upright during handoff)
- [ ] Bimanual manipulation primitives (carry large object together)

## Phase 4: Real Hardware (v0.8.0)

- [ ] Realman GEN72 SDK integration (dual)
- [ ] Calibration: arm-to-arm base transform estimation
- [ ] Force/torque sensing for contact detection
- [ ] Safety: workspace limits, collision stop, e-stop
- [ ] Teleoperation with dual input devices

## Phase 5: Advanced (v1.0.0)

- [ ] Dynamic TF tree equivalent for dora
- [ ] RViz-like visualization tool
- [ ] MoveIt2 SRDF import/export
- [ ] Behavior tree integration for complex tasks
- [ ] Multi-robot (beyond dual-arm) support
