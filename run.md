# Run — Copy/Paste Commands for All MoveIt Demos

Repo root: `/Users/nupylot/Public/github_dora_nav_moveit/dora-moveit2`

All commands below assume you start from the repo root unless noted. Stop any demo with `dora stop` in another terminal (or `Ctrl+C` then `dora stop`).

## Course Quickstart — Following `course-script/DoraMoveIt课件_v1.md`

| Course Chapter | Run this demo |
|----------------|---------------|
| Ch1 (env smoke test) | Demo #2 below |
| Ch5 MoveGroup API   | Demo #2 below (runs the full 5-feature MoveGroup walkthrough) |
| Ch6.4 single-arm grasping | Demo #11 below |
| Ch6.5 single-arm avoidance | Demo #12 below |
| Ch7/Ch8 dual-arm + final demo | Demo #8 below |
| Ch8.6 hardware mirror (template only) | Demo #13 below |

Students who want to write their own MoveGroup script (Ch5/Ch6 custom code) use the **Write your own MoveGroup script** section further down.

## 0. One-Time Setup (Install Library + Sim + All Examples)

```bash
cd /Users/nupylot/Public/github_dora_nav_moveit/dora-moveit2
pip install -e dora_moveit/
pip install -e dora-mujoco/
pip install -e examples/move_group_demo/
pip install -e examples/hunter_with_arm/
pip install -e examples/dual_gen72/
pip install -e examples/lekiwi_pick_place/
pip install -e examples/nano_pick_place/
```

> **Note:** `pip install -e dora_moveit/` pulls `dora-rs` transitively. If the `dora` CLI is missing, install it standalone: `pip install dora-rs` or `cargo install dora-cli`.

## 1. UR5e Pick-and-Place (move_group_demo)

```bash
cd /Users/nupylot/Public/github_dora_nav_moveit/dora-moveit2/examples/move_group_demo
dora up
dora start dataflows/ur5e_example_mujoco.yml
# when done:
dora stop
```

## 2. GEN72 MoveGroup Example (move_group_demo) — Course Ch1, Ch5

```bash
cd /Users/nupylot/Public/github_dora_nav_moveit/dora-moveit2/examples/move_group_demo
dora up
dora start dataflows/moveit_example_mujoco.yml
dora stop
```

## 3. Standalone UR5e Pick-and-Place Visualization (no dora)

```bash
cd /Users/nupylot/Public/github_dora_nav_moveit/dora-moveit2/examples/move_group_demo/models
mjpython pick_and_place_demo.py
```

## 4. Hunter SE + GEN72 — MoveGroup API Demo

```bash
cd /Users/nupylot/Public/github_dora_nav_moveit/dora-moveit2/examples/hunter_with_arm
dora up
dora start dataflows/movegroup_mujoco.yml
dora stop
```

## 5. Hunter SE + GEN72 — Choreographed Multi-View Capture

```bash
cd /Users/nupylot/Public/github_dora_nav_moveit/dora-moveit2/examples/hunter_with_arm
dora up
dora start dataflows/hunter_arm_mujoco.yml
dora stop
```

## 6. Standalone GEN72 Arm (no Hunter vehicle)

```bash
cd /Users/nupylot/Public/github_dora_nav_moveit/dora-moveit2/examples/hunter_with_arm
dora up
dora start dataflows/gen72_mujoco.yml
dora stop
```

## 7. Physical GEN72 Arm via Realman SDK (requires hardware)

```bash
cd /Users/nupylot/Public/github_dora_nav_moveit/dora-moveit2/examples/hunter_with_arm
dora up
dora start dataflows/gen72_real.yml
dora stop
```

## 8. Dual GEN72 — Two 7-DOF Arms — Course Ch7, Ch8

```bash
cd /Users/nupylot/Public/github_dora_nav_moveit/dora-moveit2/examples/dual_gen72
dora up
dora start dataflows/dual_gen72_mujoco.yml
dora stop
```

## 9. LeKiwi Pick-and-Place (SO_ARM100)

```bash
cd /Users/nupylot/Public/github_dora_nav_moveit/dora-moveit2/examples/lekiwi_pick_place
dora up
dora start dataflows/lekiwi_pick_place_mujoco.yml
dora stop
```

## 10. Nano Pick-and-Place (ADORA1 Nano 6-DOF)

```bash
cd /Users/nupylot/Public/github_dora_nav_moveit/dora-moveit2/examples/nano_pick_place
dora up
dora start dataflows/nano_pick_place_mujoco.yml
dora stop
```

## 11. GEN72 Single-Arm Grasping — Course Ch6.4

```bash
cd /Users/nupylot/Public/github_dora_nav_moveit/dora-moveit2/examples/move_group_demo
dora up
dora start dataflows/single_arm_grasping_mujoco.yml
dora stop
```

## 12. GEN72 Single-Arm Avoidance — Course Ch6.5

```bash
cd /Users/nupylot/Public/github_dora_nav_moveit/dora-moveit2/examples/move_group_demo
dora up
dora start dataflows/single_arm_avoidance_mujoco.yml
dora stop
```

## Write Your Own MoveGroup Script (Course Ch5 / Ch6)

For students who want to run custom code (as the course shows in Ch5 and Ch6), edit this file:

```
examples/move_group_demo/move_group_demo/nodes/my_custom_demo.py
```

Then launch the custom dataflow:

```bash
cd /Users/nupylot/Public/github_dora_nav_moveit/dora-moveit2/examples/move_group_demo
dora up
dora start dataflows/custom_demo_mujoco.yml
dora stop
```

**No hot reload.** After every edit: `Ctrl+C` → `dora stop` → relaunch with `dora start`.

## 13. Dual GEN72 — Real Hardware Template (Course Ch8.6)

> ⚠ **Hardware required — template only.** Does **not** run in simulation. Fails loudly at startup until you (1) install the Realman SDK, (2) fill in the stub driver at `examples/dual_gen72/dual_gen72_demo/nodes/realman_dual_driver.py`, and (3) set `GEN72_LEFT_IP` / `GEN72_RIGHT_IP` in the YAML.

```bash
cd /Users/nupylot/Public/github_dora_nav_moveit/dora-moveit2/examples/dual_gen72
dora up
dora start dataflows/dual_gen72_real.yml
dora stop
```

## Alternative: PYTHONPATH instead of `pip install -e`

```bash
export PYTHONPATH=/Users/nupylot/Public/github_dora_nav_moveit/dora-moveit2/dora_moveit:\
/Users/nupylot/Public/github_dora_nav_moveit/dora-moveit2/dora-mujoco:\
/Users/nupylot/Public/github_dora_nav_moveit/dora-moveit2/examples/move_group_demo:\
/Users/nupylot/Public/github_dora_nav_moveit/dora-moveit2/examples/hunter_with_arm:\
/Users/nupylot/Public/github_dora_nav_moveit/dora-moveit2/examples/dual_gen72:\
/Users/nupylot/Public/github_dora_nav_moveit/dora-moveit2/examples/lekiwi_pick_place:\
/Users/nupylot/Public/github_dora_nav_moveit/dora-moveit2/examples/nano_pick_place:$PYTHONPATH
```

## Quick Reference — Every Dataflow

| # | Example | Dataflow | Robot |
|---|---------|----------|-------|
| 1 | move_group_demo   | `dataflows/ur5e_example_mujoco.yml`     | UR5e 6-DOF + Robotiq 2F-85 |
| 2 | move_group_demo   | `dataflows/moveit_example_mujoco.yml`   | GEN72 7-DOF |
| 3 | hunter_with_arm   | `dataflows/movegroup_mujoco.yml`        | Hunter SE + GEN72 |
| 4 | hunter_with_arm   | `dataflows/hunter_arm_mujoco.yml`       | Hunter SE + GEN72 (multi-view) |
| 5 | hunter_with_arm   | `dataflows/gen72_mujoco.yml`            | GEN72 standalone |
| 6 | hunter_with_arm   | `dataflows/gen72_real.yml`              | Physical GEN72 (hardware) |
| 7 | dual_gen72        | `dataflows/dual_gen72_mujoco.yml`       | Dual GEN72 (14-DOF) |
| 8 | lekiwi_pick_place | `dataflows/lekiwi_pick_place_mujoco.yml`| LeKiwi + SO_ARM100 |
| 9 | nano_pick_place   | `dataflows/nano_pick_place_mujoco.yml`  | ADORA1 Nano 6-DOF |
| 11| move_group_demo   | `dataflows/single_arm_grasping_mujoco.yml`  | GEN72 — Course Ch6.4 |
| 12| move_group_demo   | `dataflows/single_arm_avoidance_mujoco.yml` | GEN72 — Course Ch6.5 |
| 13| dual_gen72        | `dataflows/dual_gen72_real.yml`             | Dual GEN72 physical — ⚠ template, requires hardware + SDK |

## Troubleshooting

- `ModuleNotFoundError` → re-run the install block in section 0 in the same Python env as `dora`.
- `version mismatch: message format vX is not compatible with vY` → match the `dora-rs` Python package to the `dora` CLI (`dora --version`).
- `MODEL_NAME` path errors → verify the `.xml` under the example's `models/` directory exists.
