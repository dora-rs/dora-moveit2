# ROS2 MoveIt2 Dual-Arm Course → Dora-MoveIt2 Gap Analysis

This directory maps a ROS2 MoveIt2 dual-arm manipulation course to dora-moveit2, identifying gaps, equivalences, and development items per chapter.

**Target hardware**: Dual Realman GEN72 7-DOF arms

## Chapters

| Chapter | Topic | Status |
|---------|-------|--------|
| [1](chapter1_environment_setup.md) | Environment Setup | ✅ Mostly covered |
| [2](chapter2_ros2_basics.md) | ROS2 Basics → Dora Dataflow | ⚠️ Partial (no TF tree) |
| [3](chapter3_dual_arm_urdf.md) | Dual-Arm URDF / Config | ❌ Needs dual-arm config |
| [4](chapter4_setup_assistant.md) | Setup Assistant / Planning Groups | ❌ Needs multi-group |
| [5](chapter5_python_api.md) | Python MoveGroup API | ✅ Fully covered |
| [6](chapter6_single_arm_planning.md) | Single-Arm Planning | ✅ Fully covered |
| [7](chapter7_dual_arm_coordination.md) | Dual-Arm Coordination | ❌ Largest gap |
| [8](chapter8_final_demo.md) | Final Demo & Debug | ⚠️ Needs dual-arm demo |

See also: [ROADMAP.md](ROADMAP.md) for development phases.
